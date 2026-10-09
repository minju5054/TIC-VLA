"""Saved continuous-handoff clocks / measured states; no model or Isaac dependencies."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from research.control import validate_chunk
from research.records import write_json
from research.analyze_chunk_geometry import project_world, wrap

NOISE_M = 1e-6  # Floating-point measurement floor, not a hard-case threshold.


def metrics(event):
    obs, detected, app = (event[k] for k in ("observation", "response_detected", "application"))
    start, ready = event["inference_start"], event["ready"]
    action = (ready["monotonic_ns"] - start["monotonic_ns"]) / 1e9
    handoff = (app["monotonic_ns"] - obs["monotonic_ns"]) / 1e9
    response_wall = (detected["monotonic_ns"] - obs["monotonic_ns"]) / 1e9
    pending_sim = detected["sim_time"] - obs["sim_time"]
    if action < 0 or handoff <= 0 or response_wall <= 0 or pending_sim < 0:
        raise ValueError("Invalid event clock ordering")
    d = np.asarray(app["position"]) - obs["position"]
    result = {"request_id": event["request_id"], "bootstrap": event["bootstrap"],
        "old_control_source_request_id": event["old_control_source_request_id"],
        "action_wall_s": action, "observation_to_switch_wall_s": handoff,
        "observation_to_response_wall_s": response_wall,
        "ready_to_detection_wall_s": (detected["monotonic_ns"]-ready["monotonic_ns"])/1e9,
        "pending_sim_s": pending_sim, "pending_physics_ticks": detected["tick"]-obs["tick"],
        "robot_translation_m": float(np.linalg.norm(d)),
        "robot_translation_xy_m": float(np.linalg.norm(d[:2])),
        "robot_yaw_change_deg": float(np.degrees(wrap(app["pose"][2]-obs["pose"][2]))),
        "achieved_rtf": pending_sim/response_wall, "human_translation_m": None}
    if event["human_at_observation"] is not None:
        result["human_translation_m"] = float(np.linalg.norm(
            np.asarray(event["human_at_application"]["position"])-event["human_at_observation"]["position"]))
    return result


def validate_pending(event, rows):
    """Verify command targets on every actual tick, including service action time."""
    rid = event["request_id"]
    selected = [r for r in rows if r["pending_request_id"] == str(rid)]
    obs, det = event["observation"], event["response_detected"]
    if [int(r["tick"]) for r in selected] != list(range(obs["tick"]+1, det["tick"]+1)):
        raise ValueError(f"Request {rid}: missing/extra pending ticks")
    expected_id = event["old_control_source_request_id"]
    command = [0.0, 0.0] if event["bootstrap"] else event["old_controller_command"]
    wheels = [0.0]*4 if event["bootstrap"] else event["old_wheel_commands"]
    start, ready = event["inference_start"]["monotonic_ns"], event["ready"]["monotonic_ns"]
    fully_inside_action = 0
    for row in selected:
        if row["active_control_source_request_id"] != (str(expected_id) if expected_id else ""):
            raise ValueError(f"Request {rid}: OLD command source changed while pending")
        actual = [float(row[k]) for k in ("command_v", "command_w", "command_fl", "command_rl", "command_fr", "command_rr")]
        if not np.allclose(actual, [*command, *wheels], rtol=0, atol=1e-12):
            raise ValueError(f"Request {rid}: OLD targets changed while pending")
        tick_start, tick_end = int(row["physics_step_start_monotonic_ns"]), int(row["monotonic_ns"])
        if not obs["monotonic_ns"] <= tick_start <= tick_end <= det["monotonic_ns"]:
            raise ValueError(f"Request {rid}: pending tick clocks out of bounds")
        fully_inside_action += start <= tick_start and tick_end <= ready
    return {"old_held_all_pending_ticks": True,
            "physics_ticks_wholly_inside_action_call": int(fully_inside_action)}


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"min": float(a.min()), "median": float(np.median(a)),
            "mean": float(a.mean()), "max": float(a.max())}


def analyze(run_dir, output_dir=None):
    run_dir = Path(run_dir).resolve()
    output_dir = Path(output_dir).resolve() if output_dir else run_dir/"derived/handoff"
    if output_dir.exists():
        raise FileExistsError(output_dir)
    # Only derived/ may receive analysis files inside a source run.
    if (output_dir == run_dir or run_dir in output_dir.parents) and run_dir/"derived" not in output_dir.parents:
        raise ValueError("Analysis output inside a source run must be under derived/")
    metadata = json.loads((run_dir/"metadata.json").read_text())
    cfg = metadata["config"]
    if cfg["simulation"]["pause_physics_during_inference"]:
        raise ValueError("This analysis requires continuous evidence")
    runtime = json.loads((run_dir/"inference-runtime.json").read_text())
    if not runtime["strict_checkpoint"] or runtime["simulation_app_started"]:
        raise ValueError("Expected strict checkpoint and isolated inference process")
    paths = sorted((run_dir/"raw/requests").glob("*.json"))
    if len(paths) != cfg["simulation"]["predictions"]:
        raise ValueError("Incomplete request evidence")
    with (run_dir/"robot_state.csv").open() as f:
        rows = list(csv.DictReader(f))
    events, measurements, chunks = [], [], []
    source_paths = [run_dir/p for p in ("metadata.json", "inference-runtime.json", "robot_state.csv")]
    for rid, path in enumerate(paths, 1):
        event = json.loads(path.read_text())
        chunk = validate_chunk(np.load(path.with_suffix(".npy"), allow_pickle=False))
        if (event["request_id"] != rid or event["bootstrap"] != (rid == 1)
                or event["old_control_source_request_id"] != (rid-1 if rid > 1 else None)
                or event["native_tensor_shape"] != [1, 30, 2]
                or event["native_frame"] != "observation_body"
                or event["native_axes"] != ["forward", "left"] or event["native_units"] != "meters"
                or event["agent_pose_at_observation"] != event["observation"]["pose"]
                or event["agent_pose_at_ready"] is not None):
            raise ValueError(f"Request {rid}: invalid native/anchor/bootstrap contract")
        if rid > 1 and (event["old_controller_command"] != events[-1]["controller_command"]
                        or event["old_wheel_commands"] != events[-1]["wheel_commands"]):
            raise ValueError("OLD must come from previous accepted prediction")
        keys = ("observation", "request_submit", "inference_start", "ready", "response_detected", "application")
        times = [event[k]["monotonic_ns"] for k in keys]
        if times != sorted(times) or any("event_source" not in event[k] for k in keys):
            raise ValueError("Unordered/missing event clocks")
        if event["application"]["tick"] != event["response_detected"]["tick"]:
            raise ValueError("Unexpected unlogged physics between detection and application")
        for phase in ("observation", "response_detected", "application"):
            human = event[f"human_at_{phase}"]
            if metadata["mode"] == "dynamic":
                if (human is None or human["event_source"] != "simulator_main_thread_PhysX"
                        or human["sim_time"] != event[phase]["sim_time"]
                        or not np.isfinite(human["position"]).all()):
                    raise ValueError("Missing measured PhysX event state")
        rgb = run_dir/event["rgb_observation_reference"]
        if not rgb.is_file() or rgb.stat().st_size == 0:
            raise ValueError("Missing observed RGB")
        measurements.append({**metrics(event), **validate_pending(event, rows)})
        events.append(event)
        chunks.append(chunk)
        source_paths += [path, path.with_suffix(".npy"), rgb]
    if metadata["mode"] == "dynamic":
        source_paths.append(run_dir/"raw/pedestrian_state.csv")
    nonbootstrap = measurements[1:]
    if len(nonbootstrap) < 3:
        raise ValueError("Fewer than three non-bootstrap requests")
    cycles = sum(m["pending_physics_ticks"] > 0 and m["old_held_all_pending_ticks"] for m in nonbootstrap)
    robot_moves = [m for m in nonbootstrap if m["pending_sim_s"] > 0 and m["robot_translation_m"] > NOISE_M]
    joint_moves = [m for m in robot_moves if metadata["mode"] != "dynamic" or m["human_translation_m"] > NOISE_M]
    summary = {"number_of_predictions": len(events), "number_of_bootstrap_requests": 1,
        "number_of_nonbootstrap_handoffs": len(nonbootstrap), "verified_pending_OLD_cycles": cycles,
        "nonbootstrap_requests_with_ticks_wholly_inside_action_call": sum(
            m["physics_ticks_wholly_inside_action_call"] > 0 for m in nonbootstrap),
        "continuous_handoff_validated": cycles >= 3,
        "transport_decision": "MEASURABLE ACTUAL-LATENCY TRANSPORT" if joint_moves else "NO MEASURABLE ACTUAL-LATENCY TRANSPORT",
        "nonbootstrap_joint_transport_requests": [m["request_id"] for m in joint_moves],
        "measurement_noise_floor_m": NOISE_M, "bootstrap": measurements[0],
        "aggregates_exclude_bootstrap": True,
        "definitions": {"robot_translation_m": "Euclidean world XYZ obs-to-application; XY also retained",
                        "yaw": "wrapped actual robot yaw difference, degrees; not waypoint yaw",
                        "achieved_rtf": "pending simulation advance / observation-to-response monotonic duration",
                        "physics_ticks_wholly_inside_action_call": "tick start >= service start and measured tick end <= service ready"}}
    for key in ("action_wall_s", "observation_to_switch_wall_s", "pending_sim_s", "robot_translation_m",
                "robot_translation_xy_m", "achieved_rtf"):
        summary[key] = stats([m[key] for m in nonbootstrap])
    summary["robot_abs_yaw_deg"] = stats([abs(m["robot_yaw_change_deg"]) for m in nonbootstrap])
    if metadata["mode"] == "dynamic":
        summary["human_translation_m"] = stats([m["human_translation_m"] for m in nonbootstrap])
    summary["largest_robot_transport_request_id"] = max(nonbootstrap, key=lambda m: m["robot_translation_m"])["request_id"]
    hashes = {str(p.relative_to(run_dir)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    output_dir.mkdir(parents=True, exist_ok=False)
    write_json(output_dir/"metadata.json", {"source_run": str(run_dir), "source_sha256": hashes,
        "analysis_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "simulation_code_provenance": "source metadata.json contains Git SHA, diff and source hashes"})
    write_json(output_dir/"request_metrics.json", measurements)
    write_json(output_dir/"summary.json", summary)
    with (output_dir/"request_metrics.csv").open("x") as f:
        writer = csv.DictWriter(f, fieldnames=list(measurements[0]))
        writer.writeheader()
        writer.writerows(measurements)
    transforms = []
    for event, chunk in zip(events, chunks):
        rid = event["request_id"]
        with (output_dir/f"request_{rid:06d}_world.npy").open("xb") as f:
            np.save(f, project_world(chunk, event["agent_pose_at_observation"]), allow_pickle=False)
        transforms.append({"request_id": rid, "observation_pose_xy_yaw": event["agent_pose_at_observation"],
            "rule": "world = R(yaw_observation) @ native_forward_left + observation_xy; meters"})
    write_json(output_dir/"world_transforms.json", transforms)
    plot(events, measurements, output_dir, metadata["mode"])
    if any(hashlib.sha256((run_dir/p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
        raise RuntimeError("Source evidence changed during saved-only analysis")
    return summary


def plot(events, measurements, output, mode):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figures = output/"figures"
    figures.mkdir()

    def save(fig, name):
        fig.savefig(figures/(name+".png"), dpi=170, bbox_inches="tight")
        fig.savefig(figures/(name+".pdf"), bbox_inches="tight")
        plt.close(fig)

    fig, axes = plt.subplots(len(events), 1, figsize=(10, 12), layout="constrained")
    keys = ("observation", "request_submit", "inference_start", "ready", "response_detected", "application")
    colors = ("black", "gray", "orange", "green", "purple", "red")
    for ax, event in zip(axes, events):
        origin = event["observation"]["monotonic_ns"]
        values = [(event[k]["monotonic_ns"]-origin)/1e9 for k in keys]
        ax.plot([0, values[-1]], [0, 0], color="0.8", lw=6)
        for j, (key, x, color) in enumerate(zip(keys, values, colors)):
            ax.scatter(x, j%3*.17, color=color, label=key, marker="|", s=160)
            ax.plot([x, x], [0, j%3*.17], color=color, lw=.6)
        ax.set_yticks([])
        ax.set_ylabel(f"R{event['request_id']}" + (" bootstrap" if event["bootstrap"] else " OLD held"))
        ax.set_ylim(-.15, .5)
        ax.grid(axis="x", alpha=.25)
    axes[0].legend(ncol=3, loc="lower left", bbox_to_anchor=(0, 1.36))
    axes[0].set_title("Event clocks relative to observation (seconds); independent row scales", pad=10)
    axes[-1].set_xlabel("Wall duration from observation [s], same-host monotonic clock")
    save(fig, "request_timeline")
    nonbootstrap = measurements[1:]
    for key, label, filename in [("robot_translation_m", "Robot XYZ translation [m]", "robot_transport"),
                                  ("human_translation_m", "Measured PhysX human translation [m]", "human_transport")]:
        if key == "human_translation_m" and mode != "dynamic":
            continue
        fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
        ax.bar([m["request_id"] for m in nonbootstrap], [m[key] for m in nonbootstrap])
        ax.set(xlabel="Request ID (bootstrap excluded)", ylabel=label, title=f"{mode}: observation to application")
        ax.grid(axis="y", alpha=.25)
        save(fig, filename)
    fig, ax = plt.subplots(figsize=(6, 4), layout="constrained")
    for m in nonbootstrap:
        ax.scatter(m["action_wall_s"], m["robot_translation_m"])
        ax.annotate(str(m["request_id"]), (m["action_wall_s"], m["robot_translation_m"]), xytext=(4, 3), textcoords="offset points")
    ax.set(xlabel="Actual service action-call latency [s]", ylabel="Robot obs-to-application translation [m]",
           title=f"{mode}: descriptive only, {len(nonbootstrap)} handoffs")
    ax.grid(alpha=.25)
    save(fig, "action_latency_vs_transport")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True, help="Fresh derived output directory; never overwritten")
    args = parser.parse_args()
    print(json.dumps(analyze(args.run_dir, args.output_dir), indent=2))
