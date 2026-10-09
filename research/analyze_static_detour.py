"""Saved-only asymmetric static calibration; no Isaac/model imports or corrections."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from research.records import write_json, provenance
from research.analyze_intent_handoff import load_run, sha
from research.analyze_handoffs import metrics as transport_metrics
from research.analyze_chunk_geometry import project_world
from research.control import rotation_wxyz
from research.route_geometry import (rectangle, feasibility, route_side, scene_obstacles,
                                     segment_clearance, boundary_seam)


def obstacles(cfg):
    objects = scene_obstacles(cfg)
    blocker = next(o for o in objects if o["name"] == "offset_blocker")
    walls = [o for o in objects if o["name"] in ("LeftWall", "RightWall")]
    goal = next(o for o in objects if o["name"] == "GoalWall")
    return blocker, walls, goal


def path_context(world, cfg):
    """Continuous native-point segments; terminal goal contact is separate context."""
    blocker, walls, goal = obstacles(cfg)
    params = cfg["static_detour"]
    radius = params["robot_footprint_radius_m"]
    blocked = feasibility(world, [blocker], radius)
    wall = feasibility(world, walls, radius)
    terminal = feasibility(world, [goal], radius)
    avoid = feasibility(world, [blocker, *walls], radius)
    route = route_side(world, params["decision_gate_x"], blocker, radius, cfg["scene"]["corridor_half_width"])
    gate_side = route["route_side"]
    label = {"UNKNOWN/DOES_NOT_REACH": "DOES_NOT_REACH", "UNKNOWN/AMBIGUOUS": "AMBIGUOUS"}.get(gate_side, gate_side)
    if gate_side == "LEFT":
        label = "CENTER/BLOCKED" if avoid["predicted_static_obstacle_intersection"] else "LEFT_VALID"
    return {"route_side": label, "gate_route_side": gate_side, "gate_y": route["gate_y"], **avoid,
        "predicted_offset_blocker_intersection": blocked["predicted_static_obstacle_intersection"],
        "predicted_corridor_wall_intersection": wall["predicted_static_obstacle_intersection"],
        "predicted_terminal_goal_wall_intersection": terminal["predicted_static_obstacle_intersection"],
        "predicted_any_static_including_goal_intersection": bool(avoid["predicted_static_obstacle_intersection"] or terminal["predicted_static_obstacle_intersection"]),
        "minimum_offset_blocker_clearance_m": blocked["minimum_static_obstacle_clearance_m"],
        "minimum_terminal_goal_wall_clearance_m": terminal["minimum_static_obstacle_clearance_m"],
        "predicted_uninflated_blocker_intersection": feasibility(world, [blocker], 0)["predicted_static_obstacle_intersection"]}


def consecutive_validity(rows, first_id=4, required=2):
    """Count adjacent request IDs, excluding startup even if startup paths are valid."""
    longest, length, previous, pairs, valid_ids = 0, 0, None, [], []
    for row in rows:
        rid = row["request_id"]
        valid = rid >= first_id and row["route_side"] == "LEFT_VALID" and not row["predicted_static_obstacle_intersection"]
        if valid:
            valid_ids.append(rid)
            if length and rid == previous+1:
                pairs.append([previous, rid])
                length += 1
            else:
                length = 1
        else:
            length = 0
        previous = rid
        longest = max(longest, length)
    return {"valid_left_request_ids": valid_ids, "consecutive_valid_left_pairs": pairs,
            "longest_consecutive_valid_left_run": longest, "prediction_static_detour_validated": longest >= required}


def actual_context(xy, cfg):
    xy = np.asarray(xy, float)
    blocker, walls, goal = obstacles(cfg)
    radius = cfg["static_detour"]["robot_footprint_radius_m"]
    block = feasibility(xy, [blocker], radius)
    wall = feasibility(xy, walls, radius)
    terminal = feasibility(xy, [goal], radius)
    far = float(rectangle(blocker, radius)[2])
    passed = bool(xy[:, 0].max() > far)
    positive_left = bool((xy[:, 1]-xy[0, 1]).max() > 0)
    valid = passed and positive_left and not block["predicted_static_obstacle_intersection"] and not wall["predicted_static_obstacle_intersection"]
    return {"actual_final_robot_xy": xy[-1].tolist(), "actual_max_left_y": float(xy[:, 1].max()),
        "max_left_lateral_position_m": float(xy[:, 1].max()), "max_abs_lateral_position_m": float(np.abs(xy[:, 1]).max()),
        "actual_max_positive_left_displacement_m": float((xy[:, 1]-xy[0, 1]).max()),
        "actual_robot_min_clearance_to_offset_blocker_m": block["minimum_static_obstacle_clearance_m"],
        "actual_min_clearance_to_blocker": block["minimum_static_obstacle_clearance_m"],
        "actual_robot_intersects_inflated_blocker_proxy": block["predicted_static_obstacle_intersection"],
        "actual_robot_intersects_inflated_corridor_wall_proxy": wall["predicted_static_obstacle_intersection"],
        "actual_terminal_goal_wall_intersection": terminal["predicted_static_obstacle_intersection"],
        "actual_passed_blocker": passed, "actual_max_x": float(xy[:, 0].max()), "passing_center_x_m": far,
        "execution_static_detour_validated": valid}


def camera_context(event, cfg):
    """Measured observation body pose plus unchanged mounted-camera transform."""
    obs = event["observation"]
    xyz = np.asarray(obs["position"])+rotation_wxyz(obs["quaternion_wxyz"])@cfg["camera"]["translation"]
    blocker, _, _ = obstacles(cfg)
    center, half = np.array(blocker["position"]), np.array(blocker["size"])/2
    inside = bool(np.all(xyz > center-half) and np.all(xyz < center+half))
    return {"camera_world_x": float(xyz[0]), "camera_world_y": float(xyz[1]), "camera_world_z": float(xyz[2]),
            "camera_inside_physical_blocker": inside}


def characterize(run):
    cfg = run["cfg"]
    p = cfg["static_detour"]
    times = np.array([float(r["sim_time"]) for r in run["robot"]])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in run["robot"]])
    if len(times) < 2 or not np.isfinite(times).all() or not np.isfinite(xy).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Invalid measured robot timeline")
    rows, worlds, seams = [], [], []
    previous = None
    for chunk, event in zip(run["chunks"], run["events"]):
        pose = event["agent_pose_at_observation"]
        if not np.isfinite(pose).all() or event["human_at_observation"] is not None:
            raise ValueError("Invalid static observation")
        world = project_world(chunk, pose)
        context = path_context(world, cfg)
        seam = boundary_seam(world, event["observation"], event["application"], times, xy,
            dt=p["native_waypoint_dt"], window=p["executed_tangent_window_s"], tangent_epsilon=p["minimum_tangent_displacement_m"])
        timing = transport_metrics(event)
        command = event["controller_command"]
        row = {"request_id": event["request_id"], "observation_sim_time": event["observation"]["sim_time"],
            "bootstrap": event["bootstrap"], "eligible_after_startup": event["request_id"] >= p["first_primary_request_id"],
            "robot_observation_x": pose[0], "robot_observation_y": pose[1], "robot_observation_yaw": pose[2],
            **context, **camera_context(event, cfg),
            "predicted_max_left_m": float(chunk[:, 1].max()),
            "predicted_max_abs_lateral_m": float(np.abs(chunk[:, 1]).max()),
            "predicted_mean_lateral_m": float(chunk[:, 1].mean()),
            "predicted_max_world_y_m": float(world[:, 1].max()),
            "lookahead_lateral_m": float(chunk[event["lookahead_index"], 1]),
            "controller_v": command[0], "controller_w": command[1], "lookahead_index": event["lookahead_index"],
            "delta_v": command[0]-previous[0] if previous else None,
            "delta_w": command[1]-previous[1] if previous else None,
            "action_wall_s": timing["action_wall_s"], "observation_to_switch_wall_s": timing["observation_to_switch_wall_s"],
            "observation_to_switch_sim_s": event["application"]["sim_time"]-event["observation"]["sim_time"],
            "robot_transport_obs_to_switch_m": timing["robot_translation_m"],
            "robot_transport_xy_obs_to_switch_m": timing["robot_translation_xy_m"],
            "raw_boundary_position_gap_m": seam["raw_boundary_position_gap_m"],
            "raw_tangent_gap_deg": seam["raw_executed_to_fresh_tangent_gap_deg"],
            "executed_tangent_window_displacement_m": seam["executed_window_displacement_m"],
            "fresh_tangent_segment_length_m": seam["fresh_tangent_segment_length_m"],
            "rgb_path": str(run["path"]/event["rgb_observation_reference"])}
        rows.append(row); worlds.append(world); seams.append(seam)
        previous = command
    summary = {"number_of_predictions": len(rows), "number_after_startup": sum(r["eligible_after_startup"] for r in rows),
        **consecutive_validity(rows, p["first_primary_request_id"], p["minimum_consecutive_valid_left"]), **actual_context(xy, cfg)}
    predicted, executed = summary["prediction_static_detour_validated"], summary["execution_static_detour_validated"]
    summary["decision"] = ("VALID STATIC TIC-VLA DETOUR ESTABLISHED" if predicted and executed else
        "VALID PREDICTED DETOUR, EXECUTION NOT ESTABLISHED" if predicted else "STATIC DETOUR CALIBRATION FAILED")
    primary = [r for r in rows if r["eligible_after_startup"]]
    summary["primary_route_counts"] = {label: sum(r["route_side"] == label for r in primary) for label in sorted({r["route_side"] for r in rows})}
    summary["primary_inflated_blocker_intersection_count"] = sum(r["predicted_offset_blocker_intersection"] for r in primary)
    summary["primary_uninflated_blocker_intersection_count"] = sum(r["predicted_uninflated_blocker_intersection"] for r in primary)
    summary["camera_inside_blocker_request_ids"] = [r["request_id"] for r in rows if r["camera_inside_physical_blocker"]]
    last_second = xy[times >= times[-1]-1.0]
    summary["actual_last_second_xy_net_displacement_m"] = float(np.linalg.norm(last_second[-1]-last_second[0]))
    summary["actual_last_second_x_range_m"] = float(np.ptp(last_second[:, 0]))
    summary["secondary_timing_primary_window"] = {}
    for key in ["action_wall_s", "observation_to_switch_wall_s", "observation_to_switch_sim_s",
                "robot_transport_obs_to_switch_m", "raw_boundary_position_gap_m", "raw_tangent_gap_deg"]:
        values = [r[key] for r in primary if r[key] is not None]
        summary["secondary_timing_primary_window"][key] = {"defined_count": len(values),
            "median": float(np.median(values)) if values else None, "max": max(values) if values else None}
    blocker, _, _ = obstacles(cfg)
    rect = rectangle(blocker, p["robot_footprint_radius_m"])
    clearances = np.array([segment_clearance(point, point, rect) for point in xy])
    closest = int(clearances.argmin())
    passing = np.flatnonzero(xy[:, 0] > rect[2])
    references = {"request_1": rows[0]["rgb_path"], "request_3": rows[2]["rgb_path"],
        "first_primary_valid_detour": next((r["rgb_path"] for r in primary if r["route_side"] == "LEFT_VALID"), None)}
    for label, index in [("closest_approach", closest), ("first_passing", int(passing[0]) if len(passing) else None)]:
        if index is None:
            references[label] = None
        else:
            row = min(rows, key=lambda r: abs(r["observation_sim_time"]-times[index]))
            references[label] = {"actual_sim_time": float(times[index]), "actual_xy": xy[index].tolist(),
                "request_id": row["request_id"], "rgb_path": row["rgb_path"],
                "observation_time_error_s": abs(row["observation_sim_time"]-float(times[index])),
                "note": "nearest observation in simulation time; image is not an exact closest-approach capture"}
    summary["rgb_evidence"] = references
    return rows, worlds, seams, summary, times, xy, clearances


def analyze(run_dir, output_dir, freeze_receipt):
    run_dir, output_dir = Path(run_dir).resolve(), Path(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(output_dir)
    if run_dir in output_dir.parents:
        raise ValueError("Analysis must be outside the immutable source run")
    run = load_run(run_dir)
    if run["metadata"]["mode"] != "static" or run["cfg"].get("scenario") != "asymmetric_offset_detour" or "pedestrian" in run["cfg"]:
        raise ValueError("Expected no-human asymmetric static run")
    receipt_path = Path(freeze_receipt).resolve()
    receipt = json.loads(receipt_path.read_text())
    config_path = Path(receipt["config_path"])
    if (sha(config_path) != receipt["config_sha256"] or yaml.safe_load(config_path.read_text()) != run["cfg"]
            or not receipt["preflight_geometry_pass"] or not receipt["front_rgb_visual_review_pass"]
            or receipt["authorized_run_id"] != run_dir.name
            or receipt["created_monotonic_ns"] >= run["events"][0]["inference_start"]["monotonic_ns"]):
        raise ValueError("Config/preflight was not frozen before first inference")
    for name, digest in receipt["preflight_files_sha256"].items():
        if sha(name) != digest:
            raise ValueError("Preflight evidence changed: "+name)
    sources = sorted(p for p in run_dir.rglob("*") if p.is_file())
    sources += [receipt_path, config_path, *map(Path, receipt["preflight_files_sha256"])]
    before = {str(p): sha(p) for p in sources}
    rows, worlds, seams, summary, times, xy, clearance = characterize(run)
    output_dir.mkdir(parents=True, exist_ok=False)
    derived = output_dir/"derived"; derived.mkdir()
    transforms = []
    for event, world, seam in zip(run["events"], worlds, seams):
        rid = event["request_id"]
        with (derived/f"request_{rid:06d}_world.npy").open("xb") as f:
            np.save(f, world, allow_pickle=False)
        transforms.append({"request_id": rid, "observation_pose_xy_yaw": event["agent_pose_at_observation"],
            "transform": "world = R(observation_yaw) @ native_forward_left + observation_xy", "secondary_seam": seam})
    write_json(derived/"transforms_and_secondary_seams.json", transforms)
    with (output_dir/"request_metrics.csv").open("x") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    write_json(output_dir/"request_metrics.json", rows)
    columns = ["request_id", "route_side", "gate_y", "minimum_offset_blocker_clearance_m",
               "predicted_static_obstacle_intersection", "controller_v", "controller_w",
               "robot_observation_x", "robot_observation_y", "robot_observation_yaw"]
    def cell(value):
        return f"{value:.6f}" if isinstance(value, float) else str(value) if value is not None else "—"
    with (output_dir/"request_metrics.md").open("x") as f:
        f.write("|"+"|".join(columns)+"|\n|"+"|".join(["---"]*len(columns))+"|\n")
        for row in rows:
            f.write("|"+"|".join(cell(row[key]) for key in columns)+"|\n")
    from research.static_detour_plots import plot_all
    plot_all(output_dir/"figures", run["cfg"], rows, worlds, times, xy, clearance)
    after = {str(p): sha(p) for p in sources}
    if before != after:
        raise RuntimeError("Source evidence changed during saved-only analysis")
    summary["source_immutability_verified"] = True
    write_json(output_dir/"summary.json", summary)
    write_json(output_dir/"metadata.json", {"source_run": str(run_dir), "source_files_sha256": before,
        "analysis_script_sha256": sha(Path(__file__)),
        "freeze_receipt": str(receipt_path), "config_sha256": receipt["config_sha256"], **provenance(REPO),
        "analysis_semantics": {
            "native": "(30,2) forward/left meters, own observation frame, no native yaw/timestamps",
            "prediction_segments": "native world-projected points only; no origin appended; no spatial correspondence",
            "lateral_columns": "predicted_*left/lateral and lookahead are native observation-body left; world_y column is separate",
            "gate_and_actual": "world XY meters, world +Y is scene LEFT, observation yaw radians",
            "clearance": "minimum signed distance of continuous XY segments to radius-inflated axis-aligned rectangles; conservative at corners",
            "static_intersection_column": "blocker OR corridor walls; terminal goal-wall contact separately recorded",
            "tangent": "derived geometric tangent; not native TIC-VLA yaw",
            "secondary_seam": "nominal timing assigned from source-confirmed model convention; derived observation anchor at tau=0 only for seam",
            "transport": "XYZ Euclidean observation-to-switch translation; XY also recorded",
            "camera_diagnostic": "full observation quaternion wxyz maps mounted camera body translation into world; containment in physical 3D blocker only, no image alteration",
            "scope": "one prespecified static calibration; no dynamic intent/human/reconciliation/graph/navigation-performance claim"}})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--freeze-receipt", required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run_dir, args.output_dir, args.freeze_receipt), indent=2))


if __name__ == "__main__":
    main()
