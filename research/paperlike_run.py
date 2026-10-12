"""One receipt-gated no-human / fixed-human positive control. No search/retry.

Isaac6 physics/render + separate unchanged DynaNav TICVLA in Isaac5 Python.
Synchronous action calls freeze physics like upstream on_update. All new data
are exclusive-created. Historical runners retain their defaults.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import RunRecords, clocks, provenance, write_json
from research.hospital_episode import digest, validate_config
from research.paperlike_control import PaperlikeController, SimulationScheduler, termination, human_path_metrics
from research.loop import InferenceClient, MotionHistory
from research.ipc import send, receive

POLICY = ROOT/"configs/research/paperlike_human_positive_control.json"


def freeze(out):
    out.mkdir(parents=True, exist_ok=False)
    policy = json.loads(POLICY.read_text())
    cfg = yaml.safe_load((ROOT/policy["baseline_config"]).read_text())
    cfg["paperlike"] = policy
    cfg["simulation"].update(control_hz=30, inference_hz=10, pause_physics_during_inference=True)
    del cfg["simulation"]["predictions"]
    cfg["hospital_validation"]["horizon_policy"] = "Goal <=1.5m or 85 simulation seconds; no prediction-count extension"
    human = json.loads((ROOT/policy["manual_human_receipt"]).read_text())["human"]
    asset = json.loads((ROOT/policy["manual_human_receipt"]).parent.joinpath("human_asset.json").read_text())
    b = copy.deepcopy(cfg)
    b["stationary_human"] = {**human, "prim": "/World/PositiveControlHuman", "asset": asset["asset"],
                            "radius_m": .25, "capsule_cylinder_height_m": 1.2}
    b["occlusion"] = {"visibility_rule": {
        "clearly_visible_min_fraction": .002, "clearly_visible_min_bbox_height": 80,
        "clearly_visible_min_bbox_width": 20, "hidden_max_pixels": 0}}
    for stage, c in [("A", cfg), ("B", b)]:
        validate_config(c)
        write_json(out/f"config-{stage}.json", c)
    sources = [ROOT/policy["baseline_run"], (ROOT/policy["manual_human_receipt"]).parent]
    protected = {str(f): digest(f) for p in sources for f in p.rglob("*") if f.is_file()}
    code = [*ROOT.joinpath("research").rglob("*.py"), *ROOT.joinpath("DynaNav").rglob("*.py"),
            ROOT/policy["baseline_config"], POLICY, ROOT/"scripts/isaac6_python.sh",
            ROOT/"README.md", ROOT/"rl/README.md", ROOT/"DynaNav/configs/benchmark_full.yaml"]
    weights = {}
    for name, path in [("base", Path(cfg["inference"]["base_model"])/"model.safetensors"),
                       ("checkpoint", Path(cfg["inference"]["checkpoint"]))]:
        with path.open("rb") as f:
            weights[name] = {"path": str(path), "sha256": hashlib.file_digest(f, "sha256").hexdigest()}
    write_json(out/"protocol.json", {"policy": policy, "human": human,
        "authorized_runs": {s: str(ROOT/"outputs"/(out.name+f"-stage-{s.lower()}")) for s in ["A", "B"]},
        "config_sha256": {s: digest(out/f"config-{s}.json") for s in ["A", "B"]},
        "code_sha256": {str(p): digest(p) for p in code}, "protected_sha256": protected,
        "weights": weights, "retuning_permitted": False, **provenance(ROOT), **clocks()})


def verify(out, stage):
    receipt = json.loads((out/"protocol.json").read_text())
    for group in ["code_sha256", "protected_sha256"]:
        for path, expected in receipt[group].items():
            if digest(path) != expected:
                raise ValueError("Frozen source changed: "+path)
    if digest(out/f"config-{stage}.json") != receipt["config_sha256"][stage]:
        raise ValueError("Frozen config changed")
    if stage == "B":
        result = json.loads((out/"evaluation-A.json").read_text())
        if result["stage_a_classification"] != "PAPERLIKE_NO_HUMAN_CONTROL_PASS":
            raise ValueError("PAPERLIKE_NO_HUMAN_CONTROL_FAILED; Stage B forbidden")
    return receipt, json.loads((out/f"config-{stage}.json").read_text())


def human_preflight(sim, cfg, out):
    from research.blind_corner_queries import Queries
    from research.nova_response import read_csv
    import carb
    from omni.physx import get_physx_scene_query_interface
    p = cfg["paperlike"]
    h = cfg["stationary_human"]
    # Use the already initialized PhysX scene. Do not force-load/reset it.
    # Floor rays exclude this already-authored actor and the robot, matching
    # the original static-scene-only preflight's Hospital geometry scope.
    query = Queries.__new__(Queries)
    query.carb = carb
    query.q = get_physx_scene_query_interface()
    def floor(xy, door_sill=False):
        hits = []
        def hit(h):
            if str(h.collision).startswith("/Root/"):
                hits.append((float(h.distance), list(h.position), list(h.normal), str(h.collision)))
            return True
        query.q.raycast_all(carb.Float3(*map(float, xy[:2]), .3), carb.Float3(0, 0, -1), .6, hit, True)
        if not hits:
            return False, ""
        _, pos, normal, path = min(hits)
        return abs(pos[2]) < .02 and normal[2] > .9, path
    query.floor = floor
    query.floor_prim = floor([19., 0.])[1]
    geo = query.human(h["position"], p["human_radius_m"], h["yaw"])
    e = json.loads((ROOT/p["baseline_run"]/"raw/requests/request_000019.json").read_text())
    heading = e["agent_pose_at_observation"][2]
    bypass = query.bypass(h["position"], p["robot_radius_m"], heading)
    radius = p["robot_radius_m"]+p["human_radius_m"]
    initial = float(np.linalg.norm(np.array(sim.state()["position"][:2])-h["position"][:2])-radius)
    receipt = json.loads((out/"protocol.json").read_text())
    routes = {}
    for label, directory in [("historical", ROOT/p["baseline_run"]), ("stage_a", Path(receipt["authorized_runs"]["A"]))]:
        rows = read_csv(directory/"robot_state.csv")
        xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
        routes[label] = human_path_metrics(xy, h["position"], radius, heading)
    checks = {"floor_support": geo["floor_ok"], "no_static_overlap": not geo["overlap_paths"],
              "initial_clearance_positive": initial > 0, "local_bypass": bypass["pass"],
              "no_human_route_relevant": any(r["minimum_clearance_m"] <= 0 for r in routes.values())}
    result = {"geometry": geo, "bypass": bypass, "human": h, "initial_clearance_m": initial,
              "reference_heading_rad": heading, "combined_radius_m": radius, "routes": routes,
              "checks": checks, "pass": all(checks.values()), "model_calls": 0}
    write_json(sim.records.path/"human_preflight.json", result)
    if not result["pass"]:
        raise RuntimeError("HUMAN_PREFLIGHT_FAILED")


class PaperlikeClient(InferenceClient):
    def buffer(self, request, png):
        send(self.sock, {**request, "type": "buffer_image"}, png)
        response, _ = receive(self.sock)
        if response != {"type": "image_buffered", "observation_id": request["observation_id"]}:
            raise RuntimeError("Invalid sensor-only IPC response")


def model_loop(sim, cfg, records, initial_capture):
    c = PaperlikeController()
    scheduler = SimulationScheduler()
    history = MotionHistory(sim.state())
    p = cfg["paperlike"]
    rid = oid = 0
    (records.path/"raw/observations").mkdir()
    events = (records.path/"raw/control.jsonl").open("x")
    sensors = (records.path/"raw/sensors.jsonl").open("x")
    client = PaperlikeClient(cfg, records)
    start_clock = clocks()
    try:
        while True:
            state = sim.state()
            why, distance = termination(state["position"], cfg["scene"]["goal"], state["sim_time"], p["goal_radius_m"], p["timeout_sim_s"])
            if why:
                break
            if sim.tick % 2:
                raise RuntimeError("Control update not on 30 Hz boundary")
            frame = sim.tick//2
            c.before_update(frame, state["position"])
            metadata = None
            if scheduler.due(frame):
                oid += 1
                _, obs, rgb = initial_capture if oid == 1 else sim.capture(f"rgb_{oid:06d}.png")
                archived = records.path/"raw/observations"/rgb.name
                os.link(rgb, archived)  # Same immutable PNG bytes; exclusive destination.
                rgb = archived
                request = {"type": "predict", "observation_id": oid, "observation": obs,
                    "model_step": frame, "instruction": cfg["instruction"],
                    "previous_waypoints_text": history.prompt(obs["sim_time"])}
                submitted = clocks()
                if c.mode == "BACKUP":
                    client.buffer(request, rgb.read_bytes())
                else:
                    rid += 1
                    request["request_id"] = rid
                    response = client.predict(request, rgb.read_bytes())
                    detected = sim.state()
                    chunk = np.asarray(response.pop("raw_action"), dtype=np.float32)
                    decision = c.accept(chunk, rid, frame, obs["position"],
                                        response["reasoning_new_start_step"], response["kv_cache_available"])
                    metadata = {**request, **response, **decision, "request_submit": submitted,
                        "response_detected": detected, "agent_pose_at_observation": obs["pose"],
                        "agent_pose_at_ready": None, "bootstrap": rid == 1,
                        "ready_pose_note": "No simultaneous simulator read; physics frozen across call",
                        "rgb_observation_reference": str(rgb.relative_to(records.path)),
                        "human_at_observation": sim.pedestrian.snapshot() if sim.pedestrian else None}
                    print("PAPERLIKE_PREDICTION", rid, "sim", obs["sim_time"], "mode", c.mode, flush=True)
                if sim.state()["sim_time"] != obs["sim_time"] or sim.state()["pose"] != obs["pose"]:
                    raise RuntimeError("Synchronous model/capture unexpectedly advanced physics")
                sensors.write(json.dumps({"observation_id": oid, "request_id": rid if metadata else None,
                    "observation": obs, "rgb": str(rgb.relative_to(records.path)), "mode": c.mode,
                    "buffer_only": metadata is None}, allow_nan=False)+"\n")
                sensors.flush()
            applied = c.output(1/30)
            # Bypass the historical direct-target adapter only at this explicit
            # path. Keep target and actually applied commands distinct in CSV.
            sim.adapter.target = c.target.copy()
            sim.adapter.applied = applied.copy()
            wheels = sim.adapter._push()
            sim.command = list(map(float, c.target))
            sim.wheel_command = wheels
            sim.active_control_source_request_id = c.active_request_id
            app = sim.state()
            if metadata:
                metadata.update(application=app, agent_pose_at_application=app["pose"],
                    controller_target_vw=list(map(float, c.target)), applied_vw=applied.tolist(),
                    wheel_commands=wheels, controller_mode=c.mode, command_variant="paperlike",
                    clock_policy="physics frozen during synchronous action/capture; host monotonic separate from sim",
                    outstanding_request_count=1, queued_request_count=0, catch_up=False)
                records.request(rid, chunk, metadata)
            for _ in range(2):
                after = sim.step()
                history.update(after)
                if sim.hospital_diagnostics.errors:
                    raise RuntimeError("Contact logging invalid")
                _, goal_dist = termination(after["position"], cfg["scene"]["goal"], after["sim_time"])
                human = sim.pedestrian.snapshot() if sim.pedestrian else None
                events.write(json.dumps({"control_frame": frame, "tick": after["tick"], "sim_time": after["sim_time"],
                    "mode": c.mode, "target_vw": list(map(float, c.target)), "applied_vw": applied.tolist(),
                    "slew_internal_vw": c.filtered.tolist(), "wheel_command": wheels,
                    "active_request_id": c.active_request_id, "goal_distance_m": goal_dist,
                    "human": human, **clocks()}, allow_nan=False)+"\n")
                if goal_dist <= p["goal_radius_m"] or after["sim_time"]+1e-5 >= p["timeout_sim_s"]:
                    break
            events.flush()
        final = sim.state()
        sim.adapter.stop()
        return {"runtime_status": "PASS", "termination": why, "successful_predictions": rid,
            "sensor_observations": oid, "goal_distance_m": distance, "final": final,
            "loop_start": start_clock, "loop_end": clocks(), "model_calls": rid,
            "outstanding_request_max": 1, "queued_requests": 0, "catch_up_bursts": 0,
            "contact_logger_errors": sim.hospital_diagnostics.errors}
    finally:
        events.close()
        sensors.close()
        client.close()


def execute(out, stage, preflight=False):
    receipt, cfg = verify(out, stage)
    if not preflight:
        gate = json.loads((out/f"preflight-{stage}"/"review.json").read_text())
        if not gate["visual_pass"] or not gate["technical_pass"]:
            raise ValueError("Pre-model visual / technical gate not passed")
    destination = out/f"preflight-{stage}" if preflight else Path(receipt["authorized_runs"][stage])
    records = RunRecords(destination, {"stage": stage, "model_free_preflight": preflight, "config": cfg,
        "command_variant": "paperlike", "command_argv": sys.argv, "protocol_sha256": digest(out/"protocol.json"),
        "config_sha256": receipt["config_sha256"][stage], **provenance(ROOT)})
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 1920, "height": 1080})
    sim = None
    code = 1
    try:
        import inspect
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        from research.stationary_human import StationaryHuman
        factory = (lambda s: StationaryHuman(s, cfg["stationary_human"])) if stage == "B" else None
        sim = HospitalLightsSimulation(cfg, records, actor_factory=factory)
        sim.phase = "paperlike_"+stage
        sim.adapter.controller = DifferentialController(name="paperlike_official_constants",
            wheel_radius=.152, wheel_base=.413)
        write_json(destination/"paperlike_wheel_mapping.json", {"controller_radius_m": .152,
            "controller_base_m": .413, "audited_asset_radius_m": cfg["robot"]["wheel_radius"],
            "audited_asset_base_m": cfg["robot"]["wheel_base"], "asset_modified": False,
            "source": str(Path(inspect.getfile(DifferentialController))),
            "source_sha256": digest(inspect.getfile(DifferentialController))})
        write_json(destination/"simulation-runtime.json", {"python": sys.version, "numpy": np.__version__,
            "isaac_version": (Path(os.environ["ISAAC_PATH"])/"VERSION").read_text().strip()})
        if stage == "B":
            human_preflight(sim, cfg, out)
        initial = sim.capture("rgb_000001.png")
        if stage == "B":
            visibility = json.loads((destination/"raw/visibility/rgb_000001.png.json").read_text())
            if visibility["state"] != "VISIBLE":  # Existing helper's name for CLEAR criterion.
                raise RuntimeError("FROZEN_HUMAN_NOT_CLEAR_IN_MODEL_RGB")
        if preflight:
            # Simulator-only wheel executability check. Fresh navigation run
            # starts from its own reset; this is not a policy navigation trial.
            initial_state = sim.state()
            if stage == "A":
                for v, w in [(.4, 0.), (0., .3)]:
                    for _ in range(60):
                        sim.apply(v, w)
                        sim.step()
                final = sim.state()
                moved = np.linalg.norm(np.asarray(final["position"][:2])-initial_state["position"][:2])
                yaw = float(np.arctan2(np.sin(final["pose"][2]-initial_state["pose"][2]),
                                      np.cos(final["pose"][2]-initial_state["pose"][2])))
                if moved < .2 or yaw < .1:
                    raise RuntimeError("OFFICIAL_CONSTANTS_WHEEL_CHECK_FAILED")
            else:
                moved = yaw = None
            from research.analyze_nova_response import contacts
            sim.hospital_diagnostics.file.flush()
            if sim.hospital_diagnostics.errors or contacts(destination):
                raise RuntimeError("PREFLIGHT_CONTACT_OR_LOGGER_FAILURE")
            write_json(destination/"summary.json", {"status": "PASS", "model_calls": 0,
                "simulator_startup": "PASS", "initial_state": initial_state,
                "wheel_check_displacement_m": moved, "wheel_check_yaw_rad": yaw,
                "contact_logger_errors": sim.hospital_diagnostics.errors})
        else:
            result = model_loop(sim, cfg, records, initial)
            write_json(destination/"summary.json", result)
        code = 0
    except Exception as exc:
        traceback.print_exc()
        write_json(destination/"failure.json", {"status": "FAIL", "error": repr(exc), **clocks()})
    finally:
        if sim:
            sim.close()
        app.close(exit_code=code)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--phase", choices=["freeze", "preflight", "run"], required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--stage", choices=["A", "B"], default="A")
    a = p.parse_args()
    out = Path(a.output_dir).resolve()
    if a.phase == "freeze":
        freeze(out)
    else:
        execute(out, a.stage, a.phase == "preflight")


if __name__ == "__main__":
    main()
