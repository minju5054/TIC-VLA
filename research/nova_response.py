"""Pure saved-tick replay extraction and angular response metrics."""
import csv
import json
from pathlib import Path
import numpy as np
from research.hospital_episode import digest
from research.control import rotation_wxyz


def read_csv(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))


def extract_schedule(run_dir):
    path = Path(run_dir).resolve()
    rows = read_csv(path/"robot_state.csv")
    events = [json.loads((path/f"raw/requests/request_{i:06d}.json").read_text()) for i in range(1, 9)]
    begin, end = events[3]["application"], events[7]["observation"]
    selected = [r for r in rows if 0 < int(r["tick"]) <= end["tick"]]
    primary = [r for r in selected if int(r["tick"]) > begin["tick"]]
    if ([int(r["tick"]) for r in selected] != list(range(1, end["tick"]+1))
            or {int(r["active_control_source_request_id"]) for r in primary} != {4, 5, 6, 7}):
        raise ValueError("Missing ticks or unexpected active requests in C4-C7 schedule")
    times = np.array([float(r["sim_time"]) for r in rows[:end["tick"]+1]])
    if not np.allclose(np.diff(times), 1/60, atol=1e-7, rtol=0):
        raise ValueError("Source physics cadence is not the validated 60 Hz")
    commands = [{"source_tick": int(r["tick"]), "source_sim_time": float(r["sim_time"]),
        "source_request_id": int(r["active_control_source_request_id"]) if r["active_control_source_request_id"] else None,
        "target_v": float(r["command_v"]), "target_w": float(r["command_w"]),
        "primary_window": int(r["tick"]) > begin["tick"]} for r in selected]
    if not np.isfinite([[r["target_v"], r["target_w"]] for r in commands]).all():
        raise ValueError("Nonfinite saved command")
    sources = [path/"robot_state.csv", path/"metadata.json"]+[path/f"raw/requests/request_{i:06d}.json" for i in range(1, 9)]
    return {"source_run": str(path), "source_sha256": {str(p): digest(p) for p in sources},
        "primary_request_ids": [4, 5, 6, 7], "boundary_tick": begin["tick"], "end_tick_inclusive": end["tick"],
        "first_applied_primary_tick": begin["tick"]+1, "source_time_start": begin["sim_time"],
        "source_time_end": end["sim_time"], "primary_tick_count": len(primary), "commands": commands,
        "interval_convention": "Row n records commands applied on (t[n-1],t[n]]; final row ends at C8 observation before C8 inference",
        "preroll": "Replay exact saved ticks 1..C4 application first (zero/bootstrap and C1-C3 targets), from episode start/rest. Evaluate only C4-C7. Identical input history; actual states can diverge."}


def trace(rows, embodiment):
    t = np.array([float(r["sim_time"]) for r in rows])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    yaw = np.unwrap([float(r["yaw"]) for r in rows])
    target = np.array([float(r["command_w"] if embodiment == "jackal" else r["target_w"]) for r in rows])
    applied = target.copy() if embodiment == "jackal" else np.array([float(r["applied_w"]) for r in rows])
    if embodiment == "jackal":
        omega = np.array([(rotation_wxyz([float(r[k]) for k in ("qw", "qx", "qy", "qz")]).T
                           @np.array([float(r[k]) for k in ("wx_world", "wy_world", "wz_world")]))[2] for r in rows])
    else:
        omega = np.array([float(r["body_yaw_rate_measured"]) for r in rows])
    if not np.isfinite(np.column_stack([t, xy, yaw, target, applied, omega])).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Invalid measured trace")
    return {"time": t, "xy": xy, "yaw": yaw, "target_w": target, "applied_w": applied, "omega": omega}


def response_metrics(data, first, last, zero=1e-6):
    """first/last are inclusive command-row indices; first-1 is initial pose."""
    if not 1 <= first <= last < len(data["time"]):
        raise ValueError("Metrics need an initial boundary pose and at least one tick")
    sl = slice(first, last+1)
    dt = np.diff(data["time"][first-1:last+1]); duration = float(dt.sum())
    target, applied, omega = [data[k][sl] for k in ("target_w", "applied_w", "omega")]
    pose_rate = np.diff(data["yaw"][first-1:last+1])/dt
    delta = float(data["yaw"][last]-data["yaw"][first-1])
    it, ia = float(target@dt), float(applied@dt)
    xy = data["xy"][first-1:last+1]; initial_yaw = data["yaw"][first-1]
    rotation = np.array([[np.cos(initial_yaw), -np.sin(initial_yaw)], [np.sin(initial_yaw), np.cos(initial_yaw)]])
    local = (xy[-1]-xy[0])@rotation
    sign_valid = np.abs(applied) > zero
    def mean(a): return float(a@dt/duration)
    return {"duration_s": duration, "integral_target_w_rad": it, "integral_applied_w_rad": ia,
        "actual_delta_yaw_rad": delta, "actual_delta_yaw_deg": float(np.degrees(delta)),
        "response_ratio_target": delta/it if abs(it)>zero else None,
        "response_ratio_applied": delta/ia if abs(ia)>zero else None,
        "mean_target_w": mean(target), "mean_abs_target_w": mean(np.abs(target)),
        "mean_applied_w": mean(applied), "mean_abs_applied_w": mean(np.abs(applied)),
        "mean_measured_yaw_rate": mean(omega), "peak_measured_yaw_rate": float(np.max(np.abs(omega))),
        "yaw_rate_RMSE_vs_applied_w": float(np.sqrt(mean((omega-applied)**2))),
        "pose_yaw_rate_RMSE_vs_applied_w": float(np.sqrt(mean((pose_rate-applied)**2))),
        "physics_vs_pose_rate_RMSE": float(np.sqrt(mean((omega-pose_rate)**2))),
        "sign_agreement_fraction": float(dt[sign_valid][(np.sign(omega[sign_valid])==np.sign(applied[sign_valid]))].sum()/dt[sign_valid].sum()) if sign_valid.any() else None,
        "pose_sign_agreement_fraction": float(dt[sign_valid][(np.sign(pose_rate[sign_valid])==np.sign(applied[sign_valid]))].sum()/dt[sign_valid].sum()) if sign_valid.any() else None,
        "actual_path_length_m": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
        "actual_forward_displacement_m": float(local[0]), "actual_lateral_displacement_m": float(local[1]),
        "local_frame_note": "initial measured body frame of this interval: +forward/+left, metres; not episode-global frame"}
