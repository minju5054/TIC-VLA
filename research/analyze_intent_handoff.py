"""Saved-only route intent and RAW handoff diagnostics; no Isaac/model imports."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from research.records import write_json
from research.control import validate_chunk, rotation_wxyz
from research.analyze_handoffs import metrics as transport_metrics, validate_pending
from research.route_geometry import (chunk_context, static_route_gate, temporal_overlap, boundary_seam,
                                     rotation, numerical_sign, rectangle, segment_intersects)

DEFAULT_CONFIG = REPO/"configs/research/intent_handoff_analysis.yaml"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_run(path):
    path = Path(path).resolve()
    meta = json.loads((path/"metadata.json").read_text())
    summary = json.loads((path/"summary.json").read_text())
    runtime = json.loads((path/"inference-runtime.json").read_text())
    if (summary["status"] != "PASS" or not summary.get("continuous_handoff_validated")
            or meta["config"]["simulation"]["pause_physics_during_inference"]
            or not runtime["strict_checkpoint"] or runtime["simulation_app_started"]):
        raise ValueError("Expected valid continuous real-model evidence")
    with (path/"robot_state.csv").open() as f:
        robot = list(csv.DictReader(f))
    events, chunks = [], []
    sources = [path/p for p in ["metadata.json", "summary.json", "inference-runtime.json", "robot_state.csv"]]
    files = sorted((path/"raw/requests").glob("*.json"))
    if len(files) != meta["config"]["simulation"]["predictions"]:
        raise ValueError("Incomplete request evidence")
    for rid, file in enumerate(files, 1):
        e = json.loads(file.read_text())
        c = validate_chunk(np.load(file.with_suffix(".npy"), allow_pickle=False))
        if (e["request_id"] != rid or e["bootstrap"] != (rid == 1)
                or e["native_frame"] != "observation_body" or e["native_axes"] != ["forward", "left"]
                or e["native_units"] != "meters" or e["native_tensor_shape"] != [1, 30, 2]
                or e["agent_pose_at_observation"] != e["observation"]["pose"]
                or e["agent_pose_at_ready"] is not None):
            raise ValueError("Invalid native/observation contract")
        times = [e[k]["monotonic_ns"] for k in ["observation", "request_submit", "inference_start", "ready", "response_detected", "application"]]
        if times != sorted(times):
            raise ValueError("Event clocks are not ordered")
        if rid > 1 and (e["old_control_source_request_id"] != rid-1 or e["old_controller_command"] != events[-1]["controller_command"]):
            raise ValueError("OLD must be previous accepted controller target")
        validate_pending(e, robot)
        image = path/e["rgb_observation_reference"]
        if not image.is_file():
            raise ValueError("Missing raw RGB")
        events.append(e); chunks.append(c)
        sources += [file, file.with_suffix(".npy"), image]
    for name in ["raw/pedestrian_state.csv", "raw/reveal_event.json", "raw/pedestrian_initialization.json", "route_scene_geometry.json"]:
        if (path/name).is_file():
            sources.append(path/name)
    if meta["mode"] == "dynamic" and not (path/"raw/pedestrian_state.csv").is_file():
        raise ValueError("Missing measured human CSV")
    return {"path": path, "metadata": meta, "cfg": meta["config"], "events": events,
            "chunks": chunks, "robot": robot, "sources": sources}


def characterize(run, variant, config, radius):
    cfg, events = run["cfg"], run["events"]
    worlds, contexts = [], []
    for chunk, event in zip(run["chunks"], events):
        world, context = chunk_context(chunk, event, cfg, radius)
        context["rgb_path"] = str(run["path"]/event["rgb_observation_reference"])
        context["observation_sim_time"] = event["observation"]["sim_time"]
        context["geometrically_occluded_by_central_blocker"] = None
        human = event["human_at_observation"]
        if human and "route_switch" in cfg:
            camera = np.asarray(event["observation"]["position"])+rotation_wxyz(event["observation"]["quaternion_wxyz"])@cfg["camera"]["translation"]
            block = next(o for o in cfg["scene"]["obstacles"] if o["name"] == "central_blocker")
            context["camera_world_xyz"] = camera.tolist()
            context["human_world_xyz"] = human["position"]
            context["geometrically_occluded_by_central_blocker"] = segment_intersects(camera[:2], human["position"][:2], rectangle(block))
        worlds.append(world); contexts.append(context)
    times = np.array([float(r["sim_time"]) for r in run["robot"]])
    executed = np.array([[float(r["x"]), float(r["y"])] for r in run["robot"]])
    if not np.isfinite(times).all() or not np.isfinite(executed).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Measured robot ticks must be strictly ordered")
    pairs, details = [], []
    for i in range(1, len(events)):
        old, fresh = events[i-1], events[i]
        oc, fc = contexts[i-1], contexts[i]
        ot, ft = old["observation"]["sim_time"], fresh["observation"]["sim_time"]
        overlap = temporal_overlap(worlds[i-1], worlds[i], ot, ft, config["native_waypoint_dt"], config["minimum_tangent_displacement_m"])
        seam = boundary_seam(worlds[i], fresh["observation"], fresh["application"], times, executed,
                             dt=config["native_waypoint_dt"], window=config["executed_tangent_window_s"],
                             tangent_epsilon=config["minimum_tangent_displacement_m"])
        latency = transport_metrics(fresh)
        pending = validate_pending(fresh, run["robot"])
        local_delta = overlap["delta"]@rotation(fresh["agent_pose_at_observation"][2]) if overlap else None
        old_w, fresh_w = old["controller_command"][1], fresh["controller_command"][1]
        old_sign, fresh_sign = [numerical_sign(w, config["controller_zero_tolerance_rad_s"]) for w in [old_w, fresh_w]]
        reversal = {oc["route_side"], fc["route_side"]} == {"LEFT", "RIGHT"}
        eligible = fresh["request_id"] > config["warmup_last_request_id"]
        if not eligible:
            category = "EXCLUDED STARTUP/WARM-UP"
        elif fc["predicted_static_obstacle_intersection"]:
            category = "D. VLA FAILURE / INVALID HARD-CASE CANDIDATE"
        elif reversal:
            category = "A. Strong intent-switch candidate"
        elif oc["route_side"] != fc["route_side"] or old_sign*fresh_sign == -1:
            category = "B. Intent revision but no route reversal"
        elif latency["robot_translation_m"] > config["numerical_position_tolerance_m"]:
            category = "C. Transport without route-side revision"
        else:
            category = "E. No qualifying event"
        candidate = (eligible and reversal and not fc["predicted_static_obstacle_intersection"]
                     and not oc["predicted_static_obstacle_intersection"]
                     and pending["physics_ticks_wholly_inside_action_call"] > 0
                     and latency["robot_translation_m"] > config["numerical_position_tolerance_m"]
                     and seam["raw_boundary_position_gap_m"] is not None
                     and seam["raw_boundary_position_gap_m"] > config["numerical_position_tolerance_m"])
        row = {"run_variant": variant, "old_request_id": old["request_id"], "fresh_request_id": fresh["request_id"],
            "old_observation_sim_time": ot, "fresh_observation_sim_time": ft, "switch_sim_time": fresh["application"]["sim_time"],
            "old_route_side": oc["route_side"], "fresh_route_side": fc["route_side"], "route_side_reversal": reversal,
            "old_gate_y": oc["gate_y"], "fresh_gate_y": fc["gate_y"],
            "delta_gate_y": fc["gate_y"]-oc["gate_y"] if fc["gate_y"] is not None and oc["gate_y"] is not None else None,
            "aligned_overlap_rmse_m": overlap["rmse"] if overlap else None,
            "mean_abs_lateral_revision_m": float(np.abs(local_delta[:, 1]).mean()) if overlap else None,
            "max_abs_lateral_revision_m": float(np.abs(local_delta[:, 1]).max()) if overlap else None,
            "mean_abs_tangent_revision_deg": overlap["tangent_mean_deg"] if overlap else None,
            "max_abs_tangent_revision_deg": overlap["tangent_max_deg"] if overlap else None,
            "old_v": old["controller_command"][0], "fresh_v": fresh["controller_command"][0],
            "delta_v": fresh["controller_command"][0]-old["controller_command"][0],
            "old_w": old_w, "fresh_w": fresh_w, "delta_w": fresh_w-old_w,
            "old_w_sign": old_sign, "fresh_w_sign": fresh_sign, "controller_turn_sign_reversal": old_sign*fresh_sign == -1,
            "observation_to_switch_sim_s": fresh["application"]["sim_time"]-ft,
            "observation_to_switch_wall_s": latency["observation_to_switch_wall_s"],
            "action_wall_s": latency["action_wall_s"], "robot_transport_obs_to_switch_m": latency["robot_translation_m"],
            "executed_tangent_window_displacement_m": seam["executed_window_displacement_m"],
            "fresh_tangent_segment_length_m": seam["fresh_tangent_segment_length_m"],
            **{k: seam[k] for k in ["raw_boundary_position_gap_m", "raw_executed_to_fresh_tangent_gap_deg", "raw_robot_heading_to_fresh_tangent_gap_deg"]},
            "human_position_at_fresh_observation": fresh["human_at_observation"]["position"] if fresh["human_at_observation"] else None,
            "human_position_at_switch": fresh["human_at_application"]["position"] if fresh["human_at_application"] else None,
            "fresh_static_obstacle_clearance_m": fc["minimum_static_obstacle_clearance_m"],
            "fresh_static_obstacle_intersection": fc["predicted_static_obstacle_intersection"],
            "old_static_obstacle_intersection": oc["predicted_static_obstacle_intersection"],
            "classification": category, "eligible_after_warmup": eligible,
            "reconciliation_relevant_candidate": bool(candidate),
            "physics_ticks_wholly_inside_action_call": pending["physics_ticks_wholly_inside_action_call"],
            "old_rgb_path": oc["rgb_path"], "fresh_rgb_path": fc["rgb_path"],
            "tangent_note": "derived geometric tangent; not native TIC-VLA yaw"}
        pairs.append(row)
        details.append({"old_request_id": old["request_id"], "fresh_request_id": fresh["request_id"], "seam": seam,
                        "alignment": {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in overlap.items()} if overlap else None})
    return {**run, "variant": variant, "worlds": worlds, "contexts": contexts, "pairs": pairs, "details": details}


def distributions(rows):
    result = {}
    for key in ["raw_boundary_position_gap_m", "raw_executed_to_fresh_tangent_gap_deg", "raw_robot_heading_to_fresh_tangent_gap_deg", "delta_v", "delta_w"]:
        a = np.array([abs(r[key]) for r in rows if r[key] is not None])
        result[key] = {"n": len(a), "median": float(np.median(a)) if len(a) else None, "max": float(a.max()) if len(a) else None}
    return result


def analyze(static_run, left_run, right_run, easy_run, output, config_path=DEFAULT_CONFIG):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    for source in [static_run, left_run, right_run, easy_run]:
        if source and (Path(source).resolve() == output or Path(source).resolve() in output.parents):
            raise ValueError("Intent analysis must be outside all source runs")
    config = yaml.safe_load(Path(config_path).read_text())
    static = load_run(static_run)
    radius = static["cfg"]["route_switch"]["robot_footprint_radius_m"]
    static = characterize(static, "static", config, radius)
    gate = static_route_gate(static["contexts"], static["cfg"])
    if bool(left_run) != bool(right_run):
        raise ValueError("Both mirrored variants must be supplied together")
    if gate["valid"] and not left_run:
        raise ValueError("Static route gate passed: both mirrored variants are required")
    if not gate["valid"] and left_run:
        raise ValueError("Static route gate failed: dynamic evidence must not be promoted")
    runs = [static]
    if left_run:
        if Path(left_run).resolve() == Path(right_run).resolve():
            raise ValueError("Mirrored variants must be distinct source runs")
        for path, name in [(left_run, "block_left"), (right_run, "block_right")]:
            run = load_run(path)
            if run["metadata"]["mode"] != "dynamic" or run["cfg"].get("run_variant") != name:
                raise ValueError("Source variant does not match its declared mirror")
            for key in ["scene", "robot", "camera", "controller", "inference", "instruction", "seed", "route_switch"]:
                if run["cfg"][key] != static["cfg"][key]:
                    raise ValueError("Mirror differs from the prespecified static scene: "+key)
            trigger = json.loads((run["path"]/"raw/reveal_event.json").read_text())
            if trigger["trigger_request_id"] != run["cfg"]["pedestrian"]["reveal_after_request_id"] or trigger["depends_on_prediction_content"]:
                raise ValueError("Reveal must use the prespecified request ID only")
            runs.append(characterize(run, name, config, radius))
    easy_status = "EASY CONTROL UNAVAILABLE"
    if easy_run:
        try:
            easy = load_run(easy_run)
            if easy["cfg"]["robot"] != static["cfg"]["robot"]:
                raise ValueError("Easy control robot differs; footprint cannot be reused")
            runs.append(characterize(easy, "easy_control", config, radius))
            easy_status = "VALID SAVED EASY CONTROL"
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            easy_status += ": "+str(exc)
    sources = {str(p): sha(p) for run in runs for p in run["sources"]}
    summary = {"static_gate": gate, "easy_control": easy_status, "runs": {},
        "dynamic_status": "BOTH MIRRORS ANALYZED" if left_run else "NOT RUN — NO VALID BASE ROUTE CHOICE",
        "timing_note": "waypoint timing is assigned from source-confirmed model target convention; no native timestamp channel",
        "classification_note": "B indicates a structural route-label or command-sign revision, not a thresholded claim of large geometry change"}
    for run in runs:
        eligible = [r for r in run["pairs"] if r["eligible_after_warmup"]]
        summary["runs"][run["variant"]] = {"source": str(run["path"]), "predictions": len(run["events"]),
            "route_contexts": run["contexts"], "all_nonbootstrap": distributions(run["pairs"]),
            "after_warmup": distributions(eligible), "after_warmup_count": len(eligible),
            "candidates": [r for r in eligible if r["reconciliation_relevant_candidate"]],
            "classification_counts": {c: sum(r["classification"] == c for r in eligible) for c in sorted({r["classification"] for r in eligible})}}
    dynamic_rows = [r for run in runs if run["variant"] in ("block_left", "block_right")
                    for r in run["pairs"] if r["eligible_after_warmup"]]
    summary["decisions"] = {
        "Q1": ("INSUFFICIENT EVIDENCE" if not gate["valid"] else
               "ROUTE INTENT REVERSAL OBSERVED" if any(r["route_side_reversal"] for r in dynamic_rows) else
               "INTENT REVISION WITHOUT ROUTE REVERSAL" if any(r["classification"].startswith("B.") for r in dynamic_rows) else
               "NO MEANINGFUL INTENT REVISION OBSERVED"),
        "combined": "RECONCILIATION-RELEVANT HARD-CASE CANDIDATE FOUND" if any(r["reconciliation_relevant_candidate"] for r in dynamic_rows) else "NO QUALIFYING CANDIDATE",
        "Q2_note": "Assess position/tangent/command distributions separately; no combined severity or automatic magnitude threshold.",
        "missing_condition": "valid base route choice; mirrored reveal and route conflict were not tested" if not gate["valid"] else None}
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/"metadata.json", {"source_sha256": sources, "analysis_config": config, "analysis_config_sha256": sha(config_path),
        "analysis_git_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
        "upstream_sha": subprocess.check_output(["git", "rev-parse", "upstream/main"], cwd=REPO, text=True).strip(),
        "analysis_code_sha256": {str(p): sha(p) for p in [Path(__file__), REPO/"research/route_geometry.py", REPO/"research/intent_handoff_plots.py"]}})
    rows = [row for run in runs for row in run["pairs"]]
    write_json(output/"pair_metrics.json", rows)
    with (output/"pair_metrics.csv").open("x") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader()
        writer.writerows({k: json.dumps(v) if isinstance(v, list) else v for k, v in row.items()} for row in rows)
    write_json(output/"summary.json", summary)
    for run in runs:
        target = output/"derived"/run["variant"]
        target.mkdir(parents=True)
        write_json(target/"chunk_contexts.json", run["contexts"])
        write_json(target/"pair_details.json", run["details"])
        write_json(target/"transforms.json", [{"request_id": e["request_id"], "anchor": e["agent_pose_at_observation"],
            "transform": "world = R(observation_yaw) @ native_forward_left + observation_xy"} for e in run["events"]])
        for e, world in zip(run["events"], run["worlds"]):
            with (target/f"request_{e['request_id']:06d}_world.npy").open("xb") as f:
                np.save(f, world, allow_pickle=False)
    from research.intent_handoff_plots import plot_all
    plot_all(runs, output, radius)
    if any(sha(p) != h for p, h in sources.items()):
        raise RuntimeError("Source evidence changed during saved-only analysis")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-run", required=True)
    parser.add_argument("--block-left-run")
    parser.add_argument("--block-right-run")
    parser.add_argument("--easy-control-run")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = parser.parse_args()
    summary = analyze(args.static_run, args.block_left_run, args.block_right_run, args.easy_control_run, args.output_dir, args.config)
    print(json.dumps({"static_gate": summary["static_gate"], "dynamic_status": summary["dynamic_status"], "easy_control": summary["easy_control"]}, indent=2))
