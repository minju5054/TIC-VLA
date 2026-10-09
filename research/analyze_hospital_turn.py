"""Saved-only official Hospital behavior characterization; no Isaac/model imports."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.analyze_intent_handoff import load_run
from research.analyze_handoffs import metrics as timing_metrics, validate_pending
from research.control import validate_chunk
from research.hospital_episode import validate_config, digest
from research.records import write_json, provenance

TANGENT_NOTE = "derived geometric tangent; not native TIC-VLA yaw"


def load_hospital_run(source):
    """Accept complete runs or the intact prefix of a recorded blank-RGB abort.

    Partial loading was added after the one primary run stopped. It does not
    change frozen geometry/phase rules, fill missing predictions, or relabel
    the run PASS. Other failures require investigation, not silent acceptance.
    """
    if (source/"summary.json").exists():
        run = load_run(source)
        run["completion"] = {"status": "COMPLETE", "failure": None}
        return run
    failure = json.loads((source/"failure.json").read_text())
    if failure["status"] != "FAIL" or not failure["error"].startswith("RuntimeError('Invalid/blank RGB:"):
        raise ValueError("Unsupported failed source; investigate before analysis")
    meta = json.loads((source/"metadata.json").read_text())
    runtime = json.loads((source/"inference-runtime.json").read_text())
    if (meta["mode"] != "static" or meta["config"]["simulation"]["pause_physics_during_inference"]
            or not runtime["strict_checkpoint"] or runtime["simulation_app_started"]):
        raise ValueError("Expected continuous real-model static evidence")
    with (source/"robot_state.csv").open() as f:
        robot = list(csv.DictReader(f))
    files = sorted((source/"raw/requests").glob("*.json"))
    if not 1 <= len(files) < meta["config"]["simulation"]["predictions"]:
        raise ValueError("Expected a nonempty incomplete prediction prefix")
    if {p.stem for p in files} != {p.stem for p in (source/"raw/requests").glob("*.npy")}:
        raise ValueError("Unpaired raw request files")
    events, chunks = [], []
    for rid, path in enumerate(files, 1):
        e = json.loads(path.read_text())
        chunk = validate_chunk(np.load(path.with_suffix(".npy"), allow_pickle=False))
        if (path.stem != f"request_{rid:06d}" or e["request_id"] != rid or e["bootstrap"] != (rid == 1)
                or e["native_frame"] != "observation_body" or e["native_axes"] != ["forward", "left"]
                or e["native_units"] != "meters" or e["native_tensor_shape"] != [1, 30, 2]
                or e["agent_pose_at_observation"] != e["observation"]["pose"] or e["agent_pose_at_ready"] is not None):
            raise ValueError("Invalid prefix native/observation contract")
        clocks = [e[k]["monotonic_ns"] for k in ["observation", "request_submit", "inference_start", "ready", "response_detected", "application"]]
        if clocks != sorted(clocks):
            raise ValueError("Invalid prefix clock ordering")
        if rid > 1 and (e["old_control_source_request_id"] != rid-1
                        or e["old_controller_command"] != events[-1]["controller_command"]
                        or e["old_wheel_commands"] != events[-1]["wheel_commands"]):
            raise ValueError("OLD target changed outside acceptance")
        validate_pending(e, robot)
        if not (source/e["rgb_observation_reference"]).is_file():
            raise ValueError("Missing prefix raw RGB")
        events.append(e); chunks.append(chunk)
    return {"path": source, "metadata": meta, "cfg": meta["config"], "events": events,
            "chunks": chunks, "robot": robot,
            "completion": {"status": "INCOMPLETE — ORIGINAL RUN FAILED", "failure": failure,
                "planned_predictions": meta["config"]["simulation"]["predictions"],
                "saved_predictions": len(events), "next_request_not_inferred": len(events)+1,
                "note": "Blank-RGB check stopped capture before saving the rejected image or invoking the next prediction; no rerun or fabricated data"}}


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def episode_local(world_xy, start_xy, start_yaw):
    """World XY -> episode start body frame, metres, +forward/+left."""
    return (np.asarray(world_xy)-start_xy) @ rotation(start_yaw)


def native_metrics(chunk, index, command, rules):
    p = np.asarray(chunk, dtype=float)
    if p.shape != (30, 2) or not np.isfinite(p).all() or not 0 <= index < len(p):
        raise ValueError("Invalid native chunk or lookahead index")
    segments = np.diff(p, axis=0)
    valid = np.linalg.norm(segments, axis=1) > rules["minimum_segment_length_m"]
    angles = np.unwrap(np.arctan2(segments[valid, 1], segments[valid, 0]))
    total = float(angles[-1]-angles[0]) if len(angles) >= 2 else None
    bearing = float(np.arctan2(p[index, 1], p[index, 0]))
    eps, aeps = rules["position_sign_tolerance_m"], rules["angle_sign_tolerance_rad"]
    indicators = [bool(p[-1, 1] > eps and p[:, 1].mean() > eps),
                  bool(p[index, 1] > eps and bearing > aeps),
                  bool(command[1] > rules["command_sign_tolerance_rad_s"]),
                  bool(total is not None and total > aeps)]
    return {"native_endpoint_forward_m": float(p[-1, 0]), "native_endpoint_left_m": float(p[-1, 1]),
        "native_max_left_m": float(p[:, 1].max()), "native_mean_left_m": float(p[:, 1].mean()),
        "lookahead_index_zero_based": index, "lookahead_forward_m": float(p[index, 0]),
        "lookahead_left_m": float(p[index, 1]), "lookahead_bearing_deg": float(np.degrees(bearing)),
        "controller_v": command[0], "controller_w": command[1],
        "derived_chunk_initial_tangent_deg": float(np.degrees(angles[0])) if len(angles) else None,
        "derived_chunk_terminal_tangent_deg": float(np.degrees(angles[-1])) if len(angles) else None,
        "derived_total_heading_change_deg": float(np.degrees(total)) if total is not None else None,
        "valid_tangent_segment_count": int(valid.sum()), "left_indicator_count": sum(indicators),
        "positive_lateral_future": indicators[0], "positive_lookahead": indicators[1],
        "positive_command": indicators[2], "positive_derived_turning": indicators[3],
        "weak_left_evidence": sum(indicators) >= 3, "consistent_left_indicators": all(indicators),
        "forward_dominant": bool(p[-1, 0] > abs(p[-1, 1])+eps
                                 and p[index, 0] > abs(p[index, 1])+eps and sum(indicators) < 3),
        "tangent_note": TANGENT_NOTE}


def consecutive_groups(ids, minimum=2):
    groups = []
    for rid in ids:
        if not groups or rid != groups[-1][-1]+1:
            groups.append([])
        groups[-1].append(rid)
    return [g for g in groups if len(g) >= minimum]


def phase_selection(rows, rules):
    eligible = [r for r in rows if r["request_id"] >= rules["first_primary_request_id"]
                and not r["degraded_near_black"]]
    groups = consecutive_groups([r["request_id"] for r in eligible if r["consistent_left_indicators"]],
                                rules["minimum_consecutive_left_requests"])
    first = groups[0][0] if groups else None
    forward = [r["request_id"] for r in eligible if r["forward_dominant"] and (first is None or r["request_id"] < first)]
    return {"stable_left_groups": groups, "first_stable_left_request": first,
        "transition_previous_request": first-1 if first else None,
        "first_forward_dominant_request": forward[0] if forward else None,
        "last_forward_dominant_request_before_turn": forward[-1] if forward else None,
        "weak_left_primary_requests": [r["request_id"] for r in eligible if r["weak_left_evidence"]]}


def nonfloor_contacts(entries, robot_path):
    """Measured near-horizontal normals identify vertical/steep surface contacts.

    This conservative diagnostic is not a complete collision classifier or map.
    A contact remains active until CONTACT_LOST, including sleeping bodies.
    """
    active, spans = {}, []
    for e in entries:
        pair = tuple(sorted([e["collider0"], e["collider1"]]))
        if "CONTACT_LOST" in e["type"]:
            if pair in active:
                active[pair]["end_sim_time"] = e["sim_time"]
                spans.append(active.pop(pair))
            continue
        environment = not all(e[k].startswith(robot_path+"/") for k in ("collider0", "collider1"))
        if environment and any(abs(c["normal"][2]) < .5 for c in e["contacts"]):
            if pair not in active:
                active[pair] = {"start_sim_time": e["sim_time"] if e["sim_time"] is not None else 0.,
                    "end_sim_time": None, "colliders": list(pair), "first_report": e}
    return spans+list(active.values())


def characterize(run, scene_diagnostics, contacts):
    cfg, events, robot = run["cfg"], run["events"], run["robot"]
    rules = cfg["hospital_validation"]
    ep = cfg["official_episode"]["episode"]
    rows, worlds = [], []
    for e, chunk, diag in zip(events, run["chunks"], scene_diagnostics):
        if abs(diag["observation_sim_time"]-e["observation"]["sim_time"]) > 1e-8:
            raise ValueError("RGB diagnostics do not match observation time")
        pose = e["agent_pose_at_observation"]
        latency = timing_metrics(e)
        rows.append({"request_id": e["request_id"], "observation_sim_time": e["observation"]["sim_time"],
            "robot_world_x": pose[0], "robot_world_y": pose[1], "robot_world_yaw_rad": pose[2],
            "excluded_startup": e["request_id"] < rules["first_primary_request_id"],
            **native_metrics(chunk, e["lookahead_index"], e["controller_command"], rules),
            "degraded_near_black": diag["degraded_near_black"], "rgb_p95_0_255": diag["rgb_p95_0_255"],
            "camera_near_scene_collision_paths": diag["camera_near_scene_collision_paths"],
            "rgb_path": str(run["path"]/e["rgb_observation_reference"]),
            "action_wall_s": latency["action_wall_s"],
            "observation_to_switch_wall_s": latency["observation_to_switch_wall_s"],
            "observation_to_switch_sim_s": e["application"]["sim_time"]-e["observation"]["sim_time"],
            "robot_transport_while_pending_m": latency["robot_translation_m"]})
        worlds.append(np.asarray(chunk)@rotation(pose[2]).T+pose[:2])
    phases = phase_selection(rows, rules)
    t = np.array([float(r["sim_time"]) for r in robot])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in robot])
    yaw = np.unwrap([float(r["yaw"]) for r in robot])
    if (not np.isfinite(np.column_stack([t, xy, yaw])).all() or np.any(np.diff(t) <= 0)):
        raise ValueError("Invalid measured robot history")
    local = episode_local(xy, ep["start"][:2], cfg["robot"]["start_yaw"])
    nonfloor = nonfloor_contacts(contacts, cfg["robot"]["prim"])
    intervals = []
    first = phases["first_stable_left_request"]
    for i in range(1, len(events)):
        old, fresh = events[i-1], events[i]
        indices = [np.flatnonzero(np.isclose(t, e["observation"]["sim_time"], atol=1e-8, rtol=0)) for e in (old, fresh)]
        if any(len(k) != 1 for k in indices):
            raise ValueError("Observation must exactly match a measured simulation tick")
        a, b = [int(k[0]) for k in indices]
        delta, dyaw = local[b]-local[a], float(yaw[b]-yaw[a])
        hit = any(c["start_sim_time"] <= t[b] and (c["end_sim_time"] is None or c["end_sim_time"] >= t[a]) for c in nonfloor)
        clean = (not hit and not any(r["degraded_near_black"] or r["camera_near_scene_collision_paths"] for r in rows[i-1:i+1]))
        eps, aeps = rules["position_sign_tolerance_m"], rules["angle_sign_tolerance_rad"]
        forward = clean and delta[0] > abs(delta[1])+eps
        left = (clean and first is not None and old["request_id"] >= first and dyaw > aeps
                and delta[1] > abs(delta[0])+eps)
        intervals.append({"old_request_id": old["request_id"], "fresh_request_id": fresh["request_id"],
            "start_sim_time": float(t[a]), "end_sim_time": float(t[b]),
            "episode_forward_displacement_m": float(delta[0]), "episode_left_displacement_m": float(delta[1]),
            "signed_yaw_change_deg": float(np.degrees(dyaw)), "translation_m": float(np.linalg.norm(delta)),
            "clean_visibility_and_no_nonfloor_contact": bool(clean), "forward_dominant_motion": bool(forward),
            "qualifying_left_motion": bool(left)})
    forward_intervals = [r for r in intervals if r["forward_dominant_motion"] and r["fresh_request_id"] >= rules["first_primary_request_id"]
                         and (first is None or r["fresh_request_id"] <= first)]
    actual_groups = consecutive_groups([r["fresh_request_id"] for r in intervals if r["qualifying_left_motion"]])
    prediction = first is not None
    actual = bool(actual_groups and forward_intervals)
    decision = ("TIC-VLA LEFT-TURN BEHAVIOR VALIDATED" if prediction and actual and phases["first_forward_dominant_request"] else
                "TIC-VLA LEFT-TURN PREDICTION OBSERVED; EXECUTION NOT VALIDATED" if prediction else
                "NO VALID TIC-VLA LEFT-TURN BEHAVIOR OBSERVED")
    goal_dist = np.linalg.norm(xy-np.asarray(ep["goal"][:2]), axis=1)
    # Descriptive one-second windows diagnose commanded motion with little progress.
    windows = []
    for start in np.arange(0, t[-1]-1+1e-8, 1.):
        sel = np.flatnonzero((t >= start-1e-8) & (t <= start+1+1e-8))
        a, b = sel[0], sel[-1]
        windows.append({"start_sim_time": float(t[a]), "end_sim_time": float(t[b]),
            "net_displacement_m": float(np.linalg.norm(xy[b]-xy[a])),
            "path_length_m": float(np.linalg.norm(np.diff(xy[sel], axis=0), axis=1).sum()),
            "mean_command_v": float(np.mean([float(robot[j]["command_v"]) for j in sel]))})
    summary = {"stage_2_decision": decision, "phases": phases, "prediction_count": len(rows),
        "first_primary_request_id": rules["first_primary_request_id"], "final_sim_time": float(t[-1]),
        "first_observation_sim_time": rows[0]["observation_sim_time"], "last_observation_sim_time": rows[-1]["observation_sim_time"],
        "actual_left_interval_groups_by_fresh_id": actual_groups,
        "forward_motion_interval_fresh_ids": [r["fresh_request_id"] for r in forward_intervals],
        "initial_world_xy_yaw_rad": [*xy[0].tolist(), float(yaw[0])],
        "final_world_xy_unwrapped_yaw_rad": [*xy[-1].tolist(), float(yaw[-1])],
        "final_episode_forward_left_m": local[-1].tolist(),
        "episode_forward_left_displacement_from_settled_start_m": (local[-1]-local[0]).tolist(),
        "signed_yaw_change_deg": float(np.degrees(yaw[-1]-yaw[0])),
        "cumulative_absolute_yaw_change_deg": float(np.degrees(np.abs(np.diff(yaw)).sum())),
        "maximum_signed_yaw_change_deg": float(np.degrees((yaw-yaw[0]).max())),
        "world_xy_displacement_m": (xy[-1]-xy[0]).tolist(),
        "executed_path_length_m": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
        "initial_goal_distance_m": float(goal_dist[0]), "minimum_goal_distance_m": float(goal_dist.min()),
        "final_goal_distance_m": float(goal_dist[-1]), "goal_note": "goal-distance diagnostic only; numeric goal is not model input",
        "nonfloor_contact_spans": nonfloor,
        "contact_note": "PhysX environment contact with |normal.z| < 0.5; conservative vertical/steep surface evidence, not a complete collision classifier",
        "degraded_near_black_request_ids": [r["request_id"] for r in rows if r["degraded_near_black"]],
        "camera_proximity_request_ids": [r["request_id"] for r in rows if r["camera_near_scene_collision_paths"]],
        "one_second_progress_windows": windows, "tangent_note": TANGENT_NOTE,
        "execution_note": "Conservative preregistered execution criterion: two clean consecutive positive-yaw, episode-left-dominant motion intervals after stable prediction, preceded by forward motion",
        "timing_context_nonbootstrap": {k: {"median": float(np.median([r[k] for r in rows[1:]])),
                                            "max": float(max(r[k] for r in rows[1:]))}
            for k in ["action_wall_s", "observation_to_switch_wall_s", "observation_to_switch_sim_s", "robot_transport_while_pending_m"]}}
    return rows, worlds, intervals, summary, (t, xy, local, yaw)


def analyze(run_dir, output_dir):
    source, output = Path(run_dir).resolve(), Path(output_dir).resolve()
    if output.exists():
        raise FileExistsError(output)
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("Analysis output must be separate from the source run")
    # Hash every file, including RGB, contacts, runtime identities and request arrays.
    before = {str(p): digest(p) for p in sorted(source.rglob("*")) if p.is_file()}
    run = load_hospital_run(source)
    validate_config(run["cfg"])
    receipt = run["metadata"]["official_freeze_receipt"]
    if receipt["config"] != run["cfg"] or not receipt["visual_semantic_review_pass"]:
        raise ValueError("Missing or mismatched pre-inference freeze")
    scene = json.loads((source/"hospital_scene.json").read_text())
    placement = json.loads((source/"hospital_start_audit.json").read_text())
    if (not placement["placement_valid"] or scene["custom_ground_walls_goal_obstacles_created"]
            or scene["source_root_world_transform"] != scene["composed_root_world_transform"]):
        raise ValueError("Official context audit failed")
    diags = [json.loads((source/"raw/scene_observations"/(Path(e["rgb_observation_reference"]).name+".json")).read_text()) for e in run["events"]]
    contacts = [json.loads(line) for line in (source/"raw/robot_contacts.jsonl").read_text().splitlines()]
    rows, worlds, intervals, summary, motion = characterize(run, diags, contacts)
    summary["run_completion"] = run["completion"]
    summary["stage_1_decision"] = "OFFICIAL HOSPITAL EPISODE CONTEXT VALIDATED"
    summary["source_run"] = str(source)
    summary["scene_audit"] = {k: scene[k] for k in ["hospital_url", "composed_meters_per_unit", "composed_up_axis", "source_default_prim"]}
    summary["raw_rgb_phase_references"] = {key: rows[rid-1]["rgb_path"] if rid else None for key, rid in {
        "early_forward": summary["phases"]["first_forward_dominant_request"],
        "last_forward_before_turn": summary["phases"]["last_forward_dominant_request_before_turn"],
        "first_stable_turn": summary["phases"]["first_stable_left_request"],
        "first_actual_turn_interval_end": summary["actual_left_interval_groups_by_fresh_id"][0][0] if summary["actual_left_interval_groups_by_fresh_id"] else None}.items()}
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/"metadata.json", {"source_files_sha256": before, "source_run": str(source),
        "analysis_config": run["cfg"]["hospital_validation"], "freeze_receipt": receipt, **provenance(ROOT)})
    write_json(output/"request_metrics.json", rows)
    with (output/"request_metrics.csv").open("x") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    write_json(output/"summary.json", summary)
    derived = output/"derived"
    derived.mkdir()
    write_json(derived/"actual_observation_intervals.json", intervals)
    write_json(derived/"world_transforms.json", [{"request_id": e["request_id"], "observation_pose": e["agent_pose_at_observation"],
        "transform": "world_xy = R(observation_yaw) @ native_forward_left + observation_xy; metres; no native yaw"} for e in run["events"]])
    for e, world in zip(run["events"], worlds):
        with (derived/f"request_{e['request_id']:06d}_world.npy").open("xb") as f:
            np.save(f, world, allow_pickle=False)
    from research.hospital_turn_plots import plot_all
    plot_all(output, run, rows, worlds, summary, motion)
    after = {str(p): digest(p) for p in sorted(source.rglob("*")) if p.is_file()}
    if before != after:
        raise RuntimeError("Source run changed during analysis")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    result = analyze(args.run_dir, args.output_dir)
    print(result["stage_1_decision"])
    print(result["stage_2_decision"])


if __name__ == "__main__":
    main()
