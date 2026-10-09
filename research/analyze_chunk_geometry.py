"""Saved-only nominal-time geometry analysis. No Isaac, torch, or model imports.

User-facing request IDs and waypoint indices are 1-based. Array slices are
0-based: OLD[m:N] matches FRESH[0:N-m]. Only observation poses anchor points.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
DEFAULT_CONFIG = REPO / "configs/research/chunk_geometry_analysis.json"
TANGENT_NOTE = "derived geometric tangent; not native TIC-VLA yaw"
TIMING_NOTE = "waypoint timing is assigned from the source-confirmed model target convention"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def manifest(directory):
    return {str(p.relative_to(directory)): sha256(p)
            for p in sorted(directory.rglob("*")) if p.is_file()}


def write_json(path, value):
    with Path(path).open("x") as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write("\n")


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def project_world(points, observation_pose):
    pose = np.asarray(observation_pose, dtype=float)
    return np.asarray(points, dtype=float) @ rotation(pose[2]).T + pose[:2]


def to_local(world_points, observation_pose):
    pose = np.asarray(observation_pose, dtype=float)
    return (np.asarray(world_points) - pose[:2]) @ rotation(pose[2])


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def nominal_alignment(gap, point_count, dt, tolerance):
    if not np.isfinite(gap) or dt <= 0 or tolerance <= 0 or tolerance >= dt/2:
        raise ValueError("Invalid nominal alignment parameters")
    shift = int(round(gap / dt))
    error = abs(gap - shift * dt)
    if shift < 1 or point_count-shift < 2 or error > tolerance:
        raise ValueError(f"Unsupported nominal overlap: gap={gap}, shift={shift}, error={error}")
    return shift, error


def tangent_changes(old, fresh, zero_tolerance):
    old_segments, fresh_segments = np.diff(old, axis=0), np.diff(fresh, axis=0)
    valid = ((np.linalg.norm(old_segments, axis=1) > zero_tolerance)
             & (np.linalg.norm(fresh_segments, axis=1) > zero_tolerance))
    angles = np.rad2deg(wrap(np.arctan2(fresh_segments[:, 1], fresh_segments[:, 0])
                            - np.arctan2(old_segments[:, 1], old_segments[:, 0])))
    return angles, valid


def read_numeric_csv(path, required):
    with path.open() as f:
        reader = csv.DictReader(f)
        if not set(required).issubset(reader.fieldnames or []):
            raise ValueError(f"Missing CSV columns: {path}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"Empty CSV: {path}")
    data = {key: np.array([float(row[key]) for row in rows]) for key in required}
    if not all(np.isfinite(a).all() for a in data.values()):
        raise ValueError(f"Non-finite CSV values: {path}")
    if np.any(np.diff(data["sim_time"]) <= 0):
        raise ValueError(f"Non-increasing simulation times: {path}")
    return data


def load_run(path, cfg, expected_mode):
    """Validate every request and supporting record before any pair analysis."""
    path = Path(path).resolve(strict=True)
    hashes = manifest(path)
    meta = json.loads((path/"metadata.json").read_text())
    summary = json.loads((path/"summary.json").read_text())
    n, count = cfg["expected_points"], cfg["expected_requests"]
    if (summary.get("status") != "PASS" or summary.get("mode") != expected_mode
            or meta.get("mode") != expected_mode or summary.get("successful_predictions") != count):
        raise ValueError(f"Incomplete or wrong-mode run: {path}")
    if not meta["config"]["simulation"]["pause_physics_during_inference"]:
        raise ValueError("This characterization requires the declared frozen-physics run")
    expected = [f"request_{i:06d}" for i in range(1, count+1)]
    directory = path/"raw/requests"
    for ext in ("npy", "json"):
        if sorted(p.stem for p in directory.glob(f"*.{ext}")) != expected:
            raise ValueError(f"Expected exactly request IDs 1..{count}: {path}")
    robot = read_numeric_csv(path/"robot_state.csv", ["tick", "sim_time", "x", "y", "yaw"])
    if len(np.unique(robot["tick"])) != len(robot["tick"]):
        raise ValueError("Duplicate robot ticks")
    requests = []
    for rid, stem in enumerate(expected, 1):
        chunk = np.load(directory/(stem+".npy"), allow_pickle=False)
        event = json.loads((directory/(stem+".json")).read_text())
        if chunk.shape != (n, 2) or not np.issubdtype(chunk.dtype, np.floating) or not np.isfinite(chunk).all():
            raise ValueError(f"Invalid native chunk: {stem}")
        if (type(event.get("request_id")) is not int or event["request_id"] != rid
                or event.get("native_frame") != "observation_body"
                or event.get("native_axes") != ["forward", "left"]
                or event.get("native_units") != "meters" or event.get("native_yaw") is not None
                or event.get("raw_action_shape") != [n, 2]):
            raise ValueError(f"Invalid native metadata: {stem}")
        pose = np.asarray(event["agent_pose_at_observation"], dtype=float)
        time = float(event["observation"]["sim_time"])
        command = np.asarray(event["controller_command"], dtype=float)
        if (pose.shape != (3,) or not np.isfinite(pose).all() or not np.isfinite(time)
                or command.shape != (2,) or not np.isfinite(command).all()):
            raise ValueError(f"Invalid observation/controller context: {stem}")
        if not 0 <= event["lookahead_index"] < n:
            raise ValueError(f"Invalid lookahead index: {stem}")
        yaw_filter = event["yaw_filter_state"]
        if yaw_filter is not None and not np.isfinite(yaw_filter):
            raise ValueError("Non-finite yaw filter context")
        matches = np.flatnonzero(robot["tick"] == event["observation"]["tick"])
        if len(matches) != 1:
            raise ValueError(f"No unique robot tick: {stem}")
        j = matches[0]
        robot_pose = np.array([robot[key][j] for key in ("x", "y", "yaw")])
        if (abs(robot["sim_time"][j]-time) > cfg["time_alignment_tolerance_s"]
                or not np.allclose(pose, robot_pose, atol=cfg["pose_consistency_tolerance"], rtol=0)
                or not np.allclose(pose, event["observation"]["pose"], atol=cfg["pose_consistency_tolerance"], rtol=0)):
            raise ValueError(f"Observation pose/time disagrees with robot record: {stem}")
        image = (path/event["rgb_observation_reference"]).resolve()
        if path not in image.parents or not image.is_file() or image.stat().st_size == 0:
            raise ValueError(f"Missing archived RGB: {stem}")
        requests.append({"id": rid, "time": time, "pose": pose, "native": chunk,
                         "event": event, "command": command})
    if np.any(np.diff([r["time"] for r in requests]) <= 0):
        raise ValueError("Non-increasing request observation times")
    human = None
    if expected_mode == "dynamic":
        human = read_numeric_csv(path/"raw/pedestrian_state.csv", ["sim_time", "physx_x", "physx_y", "physx_z"])
    if manifest(path) != hashes:
        raise ValueError("Source changed during validation")
    return {"path": path, "metadata": meta, "requests": requests, "robot": robot,
            "human": human, "source_file_sha256": hashes, "config": cfg}


def human_at(run, time):
    """Time matching only: never spatial matching or extrapolating missing poses."""
    data = run["human"]
    if data is None:
        return {"position": None, "method": "no_pedestrian", "max_sample_time_offset_s": None}
    times = data["sim_time"]
    tolerance = run["config"]["time_alignment_tolerance_s"]
    matches = np.flatnonzero(np.abs(times-time) <= tolerance)
    if len(matches) > 1:
        raise ValueError("Ambiguous measured pedestrian time match")
    if len(matches) == 1:
        j = matches[0]
        return {"position": [float(data[key][j]) for key in ("physx_x", "physx_y", "physx_z")],
                "method": "exact_sim_time_match", "sample_times": [float(times[j])],
                "max_sample_time_offset_s": float(abs(times[j]-time))}
    if time < times[0] or time > times[-1]:
        return {"position": None, "method": "outside_measured_coverage_no_extrapolation",
                "max_sample_time_offset_s": None}
    right = int(np.searchsorted(times, time)); left = right-1
    alpha = float((time-times[left])/(times[right]-times[left]))
    return {"position": [float((1-alpha)*data[key][left]+alpha*data[key][right])
                         for key in ("physx_x", "physx_y", "physx_z")],
            "method": "linear_time_interpolation_between_measured_samples", "weight_right": alpha,
            "sample_times": [float(times[left]), float(times[right])],
            "max_sample_time_offset_s": float(max(time-times[left], times[right]-time))}


def characterize_pair(old, fresh, cfg):
    n, dt = cfg["expected_points"], cfg["native_waypoint_dt"]
    gap = fresh["time"]-old["time"]
    shift, residual = nominal_alignment(gap, n, dt, cfg["time_alignment_tolerance_s"])
    old_world = project_world(old["native"], old["pose"])
    fresh_world = project_world(fresh["native"], fresh["pose"])
    overlap = n-shift
    old_curve, fresh_curve = old_world[shift:], fresh_world[:overlap]
    distances = np.linalg.norm(fresh_curve-old_curve, axis=1)
    signed = fresh["native"][:overlap].astype(float)-to_local(old_curve, fresh["pose"])
    if not np.allclose(np.linalg.norm(signed, axis=1), distances, atol=1e-12, rtol=1e-12):
        raise AssertionError("World/FRESH-frame displacement norms disagree")
    tangent, valid = tangent_changes(old_curve, fresh_curve, cfg["tangent_zero_length_tolerance_m"])
    tangent_abs = np.abs(tangent[valid])
    horizons = np.arange(1, overlap+1)*dt
    oe, fe = old["event"], fresh["event"]
    row = {"old_request_id": old["id"], "fresh_request_id": fresh["id"],
           "old_observation_sim_time": old["time"], "fresh_observation_sim_time": fresh["time"],
           "observation_gap_s": gap, "nominal_index_shift": shift, "overlap_point_count": overlap,
           "time_alignment_residual_s": residual,
           "aligned_overlap_mean_m": float(np.mean(distances)),
           "aligned_overlap_rmse_m": float(np.sqrt(np.mean(distances**2))),
           "aligned_overlap_median_m": float(np.median(distances)),
           "aligned_overlap_p95_m": float(np.percentile(distances, 95, method="linear")),
           "aligned_overlap_max_m": float(np.max(distances)),
           "mean_abs_forward_revision_m": float(np.mean(np.abs(signed[:, 0]))),
           "max_abs_forward_revision_m": float(np.max(np.abs(signed[:, 0]))),
           "mean_signed_forward_revision_m": float(np.mean(signed[:, 0])),
           "mean_abs_lateral_revision_m": float(np.mean(np.abs(signed[:, 1]))),
           "max_abs_lateral_revision_m": float(np.max(np.abs(signed[:, 1]))),
           "mean_signed_lateral_revision_m": float(np.mean(signed[:, 1])),
           "mean_abs_tangent_change_deg": float(tangent_abs.mean()) if len(tangent_abs) else None,
           "max_abs_tangent_change_deg": float(tangent_abs.max()) if len(tangent_abs) else None,
           "initial_tangent_change_deg": float(tangent[0]) if valid[0] else None,
           "valid_tangent_segment_count": int(valid.sum()), "tangent_semantics": TANGENT_NOTE,
           "common_horizon_endpoint_shift_m": float(distances[-1]),
           "initial_overlap_shift_m": float(distances[0]),
           "max_displacement_fresh_horizon_s": float(horizons[np.argmax(distances)]),
           "old_controller_v": float(old["command"][0]), "fresh_controller_v": float(fresh["command"][0]),
           "delta_v": float(fresh["command"][0]-old["command"][0]),
           "old_controller_w": float(old["command"][1]), "fresh_controller_w": float(fresh["command"][1]),
           "delta_w": float(fresh["command"][1]-old["command"][1]),
           "old_lookahead_index": oe["lookahead_index"], "fresh_lookahead_index": fe["lookahead_index"],
           "lookahead_index_change": fe["lookahead_index"]-oe["lookahead_index"],
           "old_yaw_filter_state": oe["yaw_filter_state"], "fresh_yaw_filter_state": fe["yaw_filter_state"],
           "robot_translation_between_observations_m": float(np.linalg.norm(fresh["pose"][:2]-old["pose"][:2])),
           "robot_yaw_change_between_observations_deg": float(np.rad2deg(wrap(fresh["pose"][2]-old["pose"][2])))}
    detail = {"old_request_id": old["id"], "fresh_request_id": fresh["id"],
              "timing_semantics": TIMING_NOTE, "tangent_semantics": TANGENT_NOTE,
              "old_waypoint_indices_1based": list(range(shift+1, n+1)),
              "fresh_waypoint_indices_1based": list(range(1, overlap+1)),
              "fresh_nominal_horizon_s": horizons.tolist(),
              "old_absolute_nominal_time_s": (old["time"]+np.arange(shift+1, n+1)*dt).tolist(),
              "fresh_absolute_nominal_time_s": (fresh["time"]+horizons).tolist(),
              "pointwise_displacement_m": distances.tolist(),
              "signed_revision_fresh_frame_m": signed.tolist(),
              "old_segment_lengths_m": np.linalg.norm(np.diff(old_curve, axis=0), axis=1).tolist(),
              "fresh_segment_lengths_m": np.linalg.norm(np.diff(fresh_curve, axis=0), axis=1).tolist(),
              "tangent_change_deg": [float(a) if ok else None for a, ok in zip(tangent, valid)]}
    return row, detail


def analyze_run(run):
    rows, details = [], []
    for old, fresh in zip(run["requests"], run["requests"][1:]):
        row, detail = characterize_pair(old, fresh, run["config"])
        human = human_at(run, fresh["time"])
        relative = to_local(np.asarray(human["position"][:2]), fresh["pose"]) if human["position"] else None
        row.update(human_forward_m=float(relative[0]) if relative is not None else None,
                   human_left_m=float(relative[1]) if relative is not None else None,
                   human_distance_m=float(np.linalg.norm(relative)) if relative is not None else None,
                   human_match_method=human["method"],
                   human_max_sample_time_offset_s=human["max_sample_time_offset_s"])
        if run["human"] is not None and relative is None:
            raise ValueError("Missing measured human coverage at a FRESH observation")
        detail.update(human_at_old_observation=human_at(run, old["time"]), human_at_fresh_observation=human)
        rows.append(row); details.append(detail)
    return rows, details


def summarize(rows):
    def values(name):
        return np.array([r[name] for r in rows if r[name] is not None])
    def extreme(name, absolute=False):
        eligible = [r for r in rows if r[name] is not None]
        if not eligible:
            return None
        row = max(eligible, key=lambda r: abs(r[name]) if absolute else r[name])
        return {"pair": [row["old_request_id"], row["fresh_request_id"]], "metric": name, "value": row[name]}
    distribution = {"median_pair_rmse_m": float(np.median(values("aligned_overlap_rmse_m"))),
                    "max_pair_rmse_m": float(np.max(values("aligned_overlap_rmse_m"))),
                    "median_pair_mean_abs_lateral_revision_m": float(np.median(values("mean_abs_lateral_revision_m"))),
                    "max_point_abs_lateral_revision_m": float(np.max(values("max_abs_lateral_revision_m")))}
    for prefix, operation, field in [("median_pair_mean_abs_tangent_change_deg", np.median, "mean_abs_tangent_change_deg"),
                                      ("max_segment_abs_tangent_change_deg", np.max, "max_abs_tangent_change_deg")]:
        a = values(field); distribution[prefix] = float(operation(a)) if len(a) else None
    return {"pair_count": len(rows), "distribution": distribution,
            "ranking_by_aligned_overlap_rmse_m": [
                {"old_request_id": r["old_request_id"], "fresh_request_id": r["fresh_request_id"],
                 "aligned_overlap_rmse_m": r["aligned_overlap_rmse_m"]}
                for r in sorted(rows, key=lambda r: (-r["aligned_overlap_rmse_m"], r["old_request_id"]))],
            "largest": {"rmse": extreme("aligned_overlap_rmse_m"),
                        "lateral_max": extreme("max_abs_lateral_revision_m"),
                        "lateral_mean": extreme("mean_abs_lateral_revision_m"),
                        "tangent_max": extreme("max_abs_tangent_change_deg"),
                        "tangent_mean": extreme("mean_abs_tangent_change_deg"),
                        "endpoint": extreme("common_horizon_endpoint_shift_m"),
                        "absolute_delta_w": extreme("delta_w", absolute=True)}}


def write_run_results(run, output, rows, details):
    derived = output/"derived"; derived.mkdir()
    write_json(output/"pair_metrics.json", rows)
    with (output/"pair_metrics.csv").open("x", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    for request in run["requests"]:
        stem = f"request_{request['id']:06d}_world"
        with (derived/(stem+".npy")).open("xb") as f:
            np.save(f, project_world(request["native"], request["pose"]), allow_pickle=False)
        transform = np.eye(3); transform[:2, :2] = rotation(request["pose"][2]); transform[:2, 2] = request["pose"][:2]
        write_json(derived/(stem+".json"), {"source_run": str(run["path"]), "source_request_id": request["id"],
                   "source_npy_sha256": run["source_file_sha256"][f"raw/requests/request_{request['id']:06d}.npy"],
                   "anchor_field": "agent_pose_at_observation", "observation_sim_time": request["time"],
                   "observation_pose_xy_yaw": request["pose"].tolist(), "T_world_from_observation_body": transform.tolist(),
                   "frame": "world XY", "units": "meters", "native_origin_added": False,
                   "analysis_provenance": "../metadata.json"})
    for detail in details:
        write_json(derived/f"pair_{detail['old_request_id']:02d}_{detail['fresh_request_id']:02d}.json", detail)


def execute(run_dir, output_dir, config_path=DEFAULT_CONFIG, static_run_dir=None, make_figures=True):
    output = Path(output_dir).resolve()
    if output.exists():
        raise FileExistsError(f"Use a fresh analysis directory: {output}")
    for source in [run_dir, static_run_dir]:
        if source is not None and Path(source).resolve() in output.parents:
            raise ValueError("Analysis output must be outside every source run")
    cfg = json.loads(Path(config_path).read_text())
    if (cfg["expected_requests"] < 2 or cfg["expected_points"] < 3
            or cfg["tangent_zero_length_tolerance_m"] < 0):
        raise ValueError("Invalid analysis configuration")
    run = load_run(run_dir, cfg, "dynamic")
    rows, details = analyze_run(run)
    static, static_rows, static_details = None, None, None
    static_status = {"status": "NOT_REQUESTED"}
    if static_run_dir is not None:
        try:
            static = load_run(static_run_dir, cfg, "static")
            static_rows, static_details = analyze_run(static)
            static_status = {"status": "VALID", "source_run": str(static["path"])}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            static = None
            static_status = {"status": "UNAVAILABLE_OR_INVALID", "source_run": str(static_run_dir), "reason": str(exc)}
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()
    code_paths = [Path(__file__), REPO/"research/chunk_geometry_plots.py"]
    meta = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
            "source_run": str(run["path"]), "source_run_metadata_sha256": run["source_file_sha256"]["metadata.json"],
            "source_file_sha256": run["source_file_sha256"], "analysis_git_sha": git("rev-parse", "HEAD"),
            "upstream_sha": git("rev-parse", "upstream/main"), "git_status": git("status", "--short"),
            "git_diff_sha256": hashlib.sha256(git("diff", "HEAD").encode()).hexdigest(),
            "analysis_config": cfg, "analysis_config_path": str(Path(config_path).resolve()),
            "analysis_config_sha256": sha256(config_path), "script_sha256": sha256(__file__),
            "analysis_code_sha256": {str(p.relative_to(REPO)): sha256(p) for p in code_paths},
            "python": sys.version, "numpy": np.__version__, "command_argv": sys.argv,
            "anchor": "agent_pose_at_observation only", "timing_semantics": TIMING_NOTE,
            "tangent_semantics": TANGENT_NOTE, "request_id_convention": "1-based",
            "static_control": static_status}
    if static is not None:
        meta["static_control"].update(source_run_metadata_sha256=static["source_file_sha256"]["metadata.json"],
                                      source_file_sha256=static["source_file_sha256"])
    summary = {"status": "PASS", "dynamic": summarize(rows), "static_control_status": static_status["status"]}
    if static is not None:
        summary["static_control"] = summarize(static_rows)
    output.mkdir(parents=True, exist_ok=False)
    write_run_results(run, output, rows, details)
    if static is not None:
        secondary = output/"static_control"; secondary.mkdir()
        write_run_results(static, secondary, static_rows, static_details)
        write_json(secondary/"metadata.json", {"source_run": str(static["path"]), "analysis_metadata": "../metadata.json",
                                              "source_file_sha256": static["source_file_sha256"]})
    if make_figures:
        from research.chunk_geometry_plots import render_figures
        meta["matplotlib"] = render_figures(run, rows, details, output/"figures", static_rows)
    # Detect changes to ANY source file, including images and ancillary logs.
    for source in [run] + ([static] if static is not None else []):
        if manifest(source["path"]) != source["source_file_sha256"]:
            raise RuntimeError("Source files changed during analysis; no PASS summary written")
    summary["source_immutability"] = "PASS: complete pre/post source manifests identical"
    write_json(output/"metadata.json", meta)
    write_json(output/"summary.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--static-run-dir", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    try:
        result = execute(args.run_dir, args.output_dir, args.config, args.static_run_dir)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f"BLOCKED: {exc}\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
