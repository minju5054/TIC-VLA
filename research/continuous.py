"""One outstanding IPC request; all simulator calls remain on the calling thread."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import time
import numpy as np
from research.control import validate_chunk, waypoint_command
from research.loop import InferenceClient, MotionHistory
from research.records import clocks, write_json


class OneRequest:
    """Only immutable request data / PNG bytes cross into the IPC worker."""
    def __init__(self, client):
        self.client = client
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="ticvla-ipc")
        self.future = None

    def submit(self, request, png):
        if self.future is not None:
            raise RuntimeError("An outstanding response must be consumed before submitting")
        self.future = self.executor.submit(self.client.predict, request, png)

    def done(self):
        return self.future is not None and self.future.done()

    def take(self):
        if not self.done():
            raise RuntimeError("No ready response")
        result = self.future.result()  # Already done: never wait on the simulation thread.
        self.future = None
        return result

    def close(self):
        # A failed simulation closes the socket first to release a pending receive.
        self.client.close()
        self.executor.shutdown(wait=True, cancel_futures=True)


@dataclass
class ActiveControl:
    source_request_id: int | None = None
    command: tuple = (0.0, 0.0)  # Bootstrap stationary target; not an invented OLD chunk.
    wheels: list | None = None

    def accept(self, request_id, command, wheels):
        expected = 1 if self.source_request_id is None else self.source_request_id + 1
        if request_id != expected:
            raise ValueError("Control acceptance must follow request order")
        self.source_request_id, self.command, self.wheels = request_id, tuple(command), list(wheels)

    def old_record(self):
        return {"old_control_source_request_id": self.source_request_id,
                "old_controller_command": list(self.command) if self.source_request_id else None,
                "old_wheel_commands": self.wheels}


class PhysicsPacer:
    """Cap physics speed; no catch-up bursts after render/IPC/OS stalls."""
    def __init__(self, dt, target_rtf, now=time.monotonic, sleep=time.sleep):
        if not np.isfinite(target_rtf) or target_rtf <= 0:
            raise ValueError("target_real_time_factor must be positive")
        self.period, self.now, self.sleep = dt / target_rtf, now, sleep

    def step(self, callback):
        start = self.now()
        result = callback()
        remaining = self.period - (self.now() - start)
        if remaining > 0:
            self.sleep(remaining)  # Only cap this physics tick to target real time.
        return result


def next_observation_time(observation_time, application_time, interval):
    nominal = observation_time + interval
    # Late response: next capture may occur immediately at the actual application
    # state. There is no catch-up queue; that observation starts a new cadence.
    return max(nominal, application_time)


def run_model_loop(sim, cfg, records, mode, evidence_analyzer=None):
    settings = cfg["simulation"]
    if settings["pause_physics_during_inference"] or not settings["real_time_pacing"]:
        raise ValueError("Continuous mode requires physics enabled and real-time pacing")
    if settings["control_hz"] != round(1/sim.dt) or mode not in ("static", "dynamic"):
        raise ValueError("Continuous mode requires per-tick control and static/dynamic mode")
    if settings["predictions"] < 4:
        raise ValueError("Validation requires bootstrap plus at least three handoffs")
    interval = 1 / settings["inference_hz"]
    if abs(interval/sim.dt-round(interval/sim.dt)) > 1e-6:
        raise ValueError("Observation period must be an integer physics-step count")
    if mode == "dynamic":
        from research.pedestrian import Pedestrian
        sim.pedestrian = Pedestrian(sim, cfg["pedestrian"])
        # A runtime-created rigid body has no PhysX pose until the first step.
        # Initialize before any observation, with no OLD and no request pending.
        sim.apply(0.0, 0.0)
        setup_state = sim.step()
        write_json(records.path/"raw/pedestrian_initialization.json", {
            "reason": "Register runtime-created rigid body in PhysX before bootstrap observation",
            "robot_state_after_initialization_tick": setup_state,
            "human_state_after_initialization_tick": sim.pedestrian.snapshot(),
            "command": [0.0, 0.0], "old_control_source_request_id": None,
            "pending_request_id": None, "excluded_from_handoff_metrics": True})
    history = MotionHistory(sim.state())
    active = ActiveControl()
    pacer = PhysicsPacer(sim.dt, settings["target_real_time_factor"])
    yaw_filter = None
    pending = OneRequest(InferenceClient(cfg, records))

    def human():
        return sim.pedestrian.snapshot() if sim.pedestrian else None

    def tick():
        sim.apply(*active.command)
        state = sim.step()
        history.update(state)
        return state

    try:
        for rid in range(1, settings["predictions"]+1):
            _, observation, rgb_path = sim.capture(f"rgb_{rid:06d}.png")
            observation["event_source"] = "simulator_main_thread_post_capture"
            human_observation = human()
            request = {"type": "predict", "request_id": rid, "observation": observation,
                       "instruction": cfg["instruction"],
                       "previous_waypoints_text": history.prompt(observation["sim_time"])}
            old = active.old_record()
            png = rgb_path.read_bytes()
            submit = {**clocks(), "event_source": "simulator_main_thread_pre_executor_submit"}
            sim.pending_request_id = rid
            pending.submit(request, png)
            while not pending.done():
                pacer.step(tick)
            detected = sim.state()
            detected["event_source"] = "simulator_main_thread_future_done"
            human_detected = human()
            response = pending.take()
            response["inference_start"]["event_source"] = "inference_service_pre_predict_async"
            response["ready"]["event_source"] = "inference_service_post_CUDA_sync_and_CPU_copy"
            chunk = validate_chunk(np.asarray(response.pop("raw_action"), dtype=np.float32))
            command, yaw_filter, index = waypoint_command(chunk, yaw_filter, **cfg["controller"])
            wheels = sim.apply(*command)
            application = sim.state()
            application["event_source"] = "simulator_main_thread_post_wheel_target_application"
            human_application = human()
            active.accept(rid, command, wheels)
            if sim.pedestrian and "route_switch" in cfg:
                sim.pedestrian.on_request_accepted(rid, application)
            sim.active_control_source_request_id = rid
            sim.pending_request_id = None
            metadata = {**request, **response, **old, "bootstrap": rid == 1,
                "request_submit": submit, "response_detected": detected,
                "agent_state_at_response_detected": detected,
                "application": application, "agent_pose_at_application": application["pose"],
                "agent_pose_at_observation": observation["pose"], "agent_pose_at_ready": None,
                "ready_pose_note": "No exact simulator sample at the service-ready event; never re-anchor FRESH.",
                "human_at_observation": human_observation, "human_at_response_detected": human_detected,
                "human_at_application": human_application,
                "rgb_observation_reference": str(rgb_path.relative_to(records.path)),
                "controller_command": list(command), "wheel_commands": wheels,
                "lookahead_index": index, "yaw_filter_state": yaw_filter,
                "clock_policy": "same-host monotonic durations; wall clocks retained; main-thread physics continues",
                "fresh_consumption": "unchanged native lookahead controller; no transport compensation/reconciliation"}
            records.request(rid, chunk, metadata)
            print("CONTINUOUS_PREDICTION_PASS", rid, "obs", observation["sim_time"],
                  "switch", application["sim_time"], "old_source", old["old_control_source_request_id"], flush=True)
            # Also give the last accepted command the normal remainder of its period.
            target = next_observation_time(observation["sim_time"], application["sim_time"], interval)
            while sim.state()["sim_time"] + 1e-7 < target:
                pacer.step(tick)
        sim.state_file.flush()
        if sim.pedestrian:
            sim.pedestrian.file.flush()
        if evidence_analyzer is None:
            from research.analyze_handoffs import analyze
            evidence_analyzer = analyze
        summary = evidence_analyzer(records.path)
        if not summary["continuous_handoff_validated"]:
            raise RuntimeError("Saved evidence did not validate at least three OLD handoff cycles")
        extra = {}
        if "route_switch" in cfg and mode == "static":
            import json
            from research.route_geometry import chunk_context, static_route_gate
            contexts = []
            for path in sorted((records.path/"raw/requests").glob("*.json")):
                event = json.loads(path.read_text())
                _, context = chunk_context(np.load(path.with_suffix(".npy")), event, cfg,
                                           cfg["route_switch"]["robot_footprint_radius_m"])
                contexts.append(context)
            gate = static_route_gate(contexts, cfg)
            write_json(records.path/"derived/route_gate.json", gate)
            extra = {"route_choice_validated": gate["valid"], "route_choice_decision": gate["decision"]}
        return {"continuous_handoff_validated": True, **extra,
                "successful_predictions": settings["predictions"],
                "native_tensor_shape": [1, 30, 2], "stored_action_shape": [30, 2], "all_finite": True,
                "transport_decision": summary["transport_decision"],
                "derived_summary": "derived/handoff/summary.json"}
    finally:
        pending.close()
