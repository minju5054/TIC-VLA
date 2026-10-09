"""Sequential request/response control, with frozen physics during inference."""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import numpy as np
from research.control import rotation_wxyz, waypoint_command, validate_chunk
from research.ipc import send, receive
from research.records import clocks, write_json

REPO = Path(__file__).resolve().parents[1]


class InferenceClient:
    def __init__(self, cfg, records):
        self.sock, child = socket.socketpair()
        self.sock.settimeout(cfg["inference"]["startup_timeout_seconds"])
        env = os.environ.copy()
        # Do not inherit Isaac 6's setup variables into the independent Isaac 5 process.
        for key in ["LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH", "PYTHONEXE", "ISAAC_PATH",
                    "EXP_PATH", "CARB_APP_PATH"]:
            env.pop(key, None)
        env.update(PYTHONNOUSERSITE="1", PYTHONUNBUFFERED="1", CUDA_VISIBLE_DEVICES="0",
                   HF_HOME=str(Path(cfg["inference"]["base_model"]).parent/"hf-cache"),
                   HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1")
        self.log = (records.path/"inference.log").open("x")
        # Pass the already snapshotted config, not a mutable source file.
        config_path = records.path/"runtime-config.json"
        write_json(config_path, cfg) # JSON is valid YAML for both runtimes.
        self.process = subprocess.Popen([cfg["inference"]["python"], str(REPO/"research/inference_service.py"),
             "--fd", str(child.fileno()), "--config", str(config_path), "--run-dir", str(records.path)],
             env=env, pass_fds=(child.fileno(),), stdout=self.log, stderr=subprocess.STDOUT)
        child.close()
        try:
            hello, _ = receive(self.sock)
            if hello.get("type") != "ready":
                raise RuntimeError(f"Inference startup failed: {hello}")
            write_json(records.path/"inference-runtime.json", hello)
            self.sock.settimeout(cfg["inference"]["request_timeout_seconds"])
        except BaseException:
            self.close()
            raise

    def predict(self, metadata, png):
        send(self.sock, metadata, png)
        response, _ = receive(self.sock)
        if response.get("type") != "prediction" or response.get("request_id") != metadata["request_id"]:
            raise RuntimeError(f"Invalid inference response: {response}")
        return response

    def close(self):
        try:
            send(self.sock, {"type": "shutdown"})
        except (OSError, ValueError):
            pass
        self.sock.close()
        try:
            self.process.wait(timeout=40)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.log.close()


class MotionHistory:
    def __init__(self, initial):
        self.previous = initial
        self.deltas = []
        self.next_time = 1.0

    def update(self, state):
        if state["sim_time"] + 1e-7 >= self.next_time:
            delta = rotation_wxyz(self.previous["quaternion_wxyz"]).T @ (np.array(state["position"])-self.previous["position"])
            self.deltas.append(delta.tolist())
            self.previous = state
            self.next_time += 1.0

    def prompt(self, sim_time):
        nonzero = [p for p in self.deltas if any(abs(x) >= 1e-6 for x in p)]
        if not nonzero:
            return f"From 0.0s to current timestamp time is {sim_time:.1f}s. No waypoints available."
        points = ", ".join(f"({x:.2f}, {y:.2f}, {z:.2f})" for x, y, z in nonzero)
        return (f"From 0.0s to current timestamp time is {sim_time:.1f}s. (a list of waypoints 1s in between): {points}\n"
                "Each waypoint (x, y, z) is the displacement over the previous 1.0s. x is forward, y is left, z is up.")


def run_model_loop(sim, cfg, records, mode):
    if not cfg["simulation"]["pause_physics_during_inference"]:
        raise ValueError("Only sequential frozen-physics inference is implemented")
    if cfg["simulation"]["control_hz"] != round(1/sim.dt):
        raise ValueError("This harness applies wheel targets at every physics step")
    count = 1 if mode == "inference" else cfg["simulation"]["predictions"]
    ratio = 1/cfg["simulation"]["inference_hz"]/sim.dt
    if abs(ratio-round(ratio)) > 1e-6:
        raise ValueError("Inference period must be an integer physics-step count")
    interval = round(ratio)
    if mode == "dynamic":
        from research.pedestrian import Pedestrian
        sim.pedestrian = Pedestrian(sim, cfg["pedestrian"])
    initial = sim.state()
    history = MotionHistory(initial)
    yaw_filter = None
    successes = []
    client = InferenceClient(cfg, records)
    try:
        for rid in range(1, count+1):
            _, observation, rgb_path = sim.capture(f"rgb_{rid:06d}.png")
            request = {"type": "predict", "request_id": rid, "observation": observation,
                       "instruction": cfg["instruction"], "previous_waypoints_text": history.prompt(observation["sim_time"])}
            response = client.predict(request, rgb_path.read_bytes())
            received = clocks()
            at_receive = sim.state()
            if abs(at_receive["sim_time"]-observation["sim_time"]) > 1e-9:
                raise RuntimeError("Physics advanced while awaiting inference")
            chunk = validate_chunk(np.array(response.pop("raw_action"), dtype=np.float32))
            command, yaw_filter, lookahead_index = waypoint_command(chunk, yaw_filter, **cfg["controller"])
            application = None
            wheels = None
            if mode != "inference":
                wheels = sim.apply(*command)
                application = sim.state() # timestamp after API application, before next physics step
            metadata = {**request, **response, "receive": received,
                "rgb_observation_reference": str(rgb_path.relative_to(records.path)),
                "agent_pose_at_observation": observation["pose"], "agent_pose_at_ready": None,
                "ready_pose_note": "No simultaneous simulator sample at service ready; physics frozen throughout wait.",
                "agent_state_at_receive": at_receive,
                "application": application, "agent_pose_at_application": application["pose"] if application else None,
                "controller_command": list(command), "lookahead_index": lookahead_index,
                "yaw_filter_state": yaw_filter, "wheel_commands": wheels,
                "clock_policy": "same-host wall/monotonic clocks; physics frozen during sequential IPC wait"}
            records.request(rid, chunk, metadata)
            print("PREDICTION_PASS", rid, chunk.shape, "command", command, "sim_time", observation["sim_time"], flush=True)
            if mode != "inference":
                for _ in range(interval):
                    sim.apply(*command)
                    state = sim.step()
                    history.update(state)
                after = sim.state()
                delta = float(np.linalg.norm(np.array(after["position"][:2])-observation["position"][:2]))
                successes.append({"request_id": rid, "displacement_after_application": delta,
                                  "pose_after_execution": after["pose"]})
            else:
                successes.append({"request_id": rid, "displacement_after_application": None})
        final = sim.state()
        total = float(np.linalg.norm(np.array(final["position"][:2])-initial["position"][:2]))
        if mode != "inference" and (len(successes) < 2 or sum(x["displacement_after_application"] > .001 for x in successes) < 2):
            raise RuntimeError("Fewer than two predictions caused measured motion")
        dynamic = sim.pedestrian.evidence() if sim.pedestrian else None
        if dynamic and dynamic["displacement"] <= .1:
            raise RuntimeError("Pedestrian did not move")
        return {"successful_predictions": len(successes), "native_tensor_shape": [1, 30, 2],
                "stored_action_shape": [30, 2], "all_finite": True, "robot_displacement": total,
                "robot_yaw_change": math.atan2(math.sin(final["pose"][2]-initial["pose"][2]), math.cos(final["pose"][2]-initial["pose"][2])),
                "initial": initial, "final": final, "per_request_execution": successes, "pedestrian": dynamic}
    finally:
        client.close()
