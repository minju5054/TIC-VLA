"""Saved-only positive-control evaluation; no OLD/FRESH matching or ranking."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json, provenance
from research.hospital_episode import digest
from research.paperlike_control import project_observation, human_path_metrics
from research.nova_response import read_csv
from research.analyze_nova_response import contacts
from research.hospital_spatial_gate import spatial_gate, wall_inventory


def stats(a):
    a = np.asarray(a, float)
    return {"count": len(a), "min": float(a.min()), "median": float(np.median(a)),
            "mean": float(a.mean()), "max": float(a.max())} if len(a) else None


def record_evaluation(path, summary):
    """Reuse an identical gate receipt; never replace a historical decision."""
    if path.exists():
        if json.loads(path.read_text()) != summary:
            raise ValueError("Existing evaluation differs; historical gate cannot be overwritten")
    else:
        write_json(path, summary)


def load(run):
    meta = json.loads((run/"metadata.json").read_text())
    cfg = meta["config"]
    rows = read_csv(run/"robot_state.csv")
    t = np.array([float(r["sim_time"]) for r in rows])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    paths = sorted((run/"raw/requests").glob("*.json"))
    if len(paths) != len(list((run/"raw/requests").glob("*.npy"))):
        raise ValueError("Unpaired request artifacts")
    events, chunks, worlds = [], [], []
    for rid, path in enumerate(paths, 1):
        e = json.loads(path.read_text())
        chunk = np.load(path.with_suffix(".npy"), allow_pickle=False)
        if e["request_id"] != rid or e["agent_pose_at_observation"] != e["observation"]["pose"]:
            raise ValueError("Request ID / anchor mismatch")
        if e["native_units"] != "meters" or e["native_tensor_shape"] != [1, 30, 2]:
            raise ValueError("Native convention mismatch")
        obs, app = e["observation"], e["application"]
        if obs["tick"] != app["tick"] or obs["pose"] != app["pose"] or obs["sim_time"] != app["sim_time"]:
            raise ValueError("Synchronous action advanced physics")
        row = rows[obs["tick"]]
        np.testing.assert_array_equal(obs["pose"], [float(row[k]) for k in ["x", "y", "yaw"]])
        clocks = [e[k]["monotonic_ns"] for k in ["observation", "request_submit", "inference_start", "ready", "response_detected", "application"]]
        if clocks != sorted(clocks) or not (run/e["rgb_observation_reference"]).is_file():
            raise ValueError("Missing RGB / invalid clocks")
        worlds.append(project_observation(chunk, e))
        events.append(e)
        chunks.append(chunk)
    return {"path": run, "meta": meta, "cfg": cfg, "rows": rows, "t": t, "xy": xy,
            "events": events, "chunks": chunks, "worlds": worlds, "contacts": contacts(run)}


def analyze(setup, stage, output):
    receipt = json.loads((setup/"protocol.json").read_text())
    run = Path(receipt["authorized_runs"][stage])
    if output.exists() or run in output.parents:
        raise FileExistsError("Fresh derived directory outside source required")
    hashes = {str(p): digest(p) for p in run.rglob("*") if p.is_file()}
    r = load(run)
    cfg, p = r["cfg"], r["cfg"]["paperlike"]
    output.mkdir(parents=True, exist_ok=False)
    runtime = json.loads((run/"summary.json").read_text()) if (run/"summary.json").exists() else None
    inference = json.loads((run/"inference-runtime.json").read_text()) if (run/"inference-runtime.json").exists() else None
    technical = bool(runtime and runtime["runtime_status"] == "PASS" and inference and inference["strict_checkpoint"]
                     and not inference["simulation_app_started"] and not runtime["contact_logger_errors"]
                     and len(r["events"]) == runtime["successful_predictions"])
    goal = np.asarray(cfg["scene"]["goal"][:2])
    distance = np.linalg.norm(r["xy"]-goal, axis=1)
    entries = np.flatnonzero(distance <= p["goal_radius_m"])
    gate = spatial_gate(r["events"], cfg["spatial_turn_gate"])
    # Stage A deliberately requires full goal entry, a stronger prerequisite
    # than substantial progress alone; declared before either model run.
    gate_pass = bool(technical and len(entries) and gate["crossing_plus_one_observation"] and not r["contacts"])
    sensor = [json.loads(s) for s in (run/"raw/sensors.jsonl").read_text().splitlines()]
    times = np.array([e["observation"]["sim_time"] for e in r["events"]])
    gaps = np.diff(times)
    sensor_gaps = np.diff([s["observation"]["sim_time"] for s in sensor])
    call = [(e["ready"]["monotonic_ns"]-e["inference_start"]["monotonic_ns"])/1e9 for e in r["events"]]
    latency = [(e["application"]["monotonic_ns"]-e["observation"]["monotonic_ns"])/1e9 for e in r["events"]]
    app_wall = np.array([e["application"]["monotonic_ns"]/1e9 for e in r["events"]])
    app_time = np.array([e["application"]["sim_time"] for e in r["events"]])
    cadence = {"requested_sim_hz": 10, "observation_gap_sim_s": stats(gaps),
        "sensor_gap_sim_s": stats(sensor_gaps), "action_call_wall_s": stats(call),
        "action_call_nonbootstrap_wall_s": stats(call[1:]), "observation_to_application_wall_s": stats(latency),
        "simulation_observation_gaps_above_0_1s": int(sum(gaps > .1+p["time_tolerance_s"])),
        "action_calls_above_0_1_wall_s": int(sum(np.array(call) > .1)),
        "observation_to_application_above_0_1_wall_s": int(sum(np.array(latency) > .1)),
        "effective_updates_per_sim_s": float((len(times)-1)/(app_time[-1]-app_time[0])) if len(times)>1 else None,
        "effective_updates_per_wall_s": float((len(times)-1)/(app_wall[-1]-app_wall[0])) if len(times)>1 else None,
        "outstanding_max": 1, "queued": 0, "catch_up_bursts": 0,
        "execution": p["execution"], "wall_realtime_10hz_claim": False}
    human = cfg.get("stationary_human")
    passage = None
    local = full = None
    human_contacts = [s for s in r["contacts"] if human and human["prim"] in json.dumps(s)]
    structure = [s for s in r["contacts"] if s not in human_contacts]
    if human:
        rows = read_csv(run/"raw/human_state.csv")
        hp = np.array([[float(s[k]) for k in ["x", "y"]] for s in rows])
        ht = np.array([float(s["sim_time"]) for s in rows])
        np.testing.assert_array_equal(ht, r["t"])
        np.testing.assert_allclose(hp, np.tile(human["position"][:2], (len(hp), 1)), atol=1e-6, rtol=0)
        reference = json.loads((ROOT/p["baseline_run"]/"raw/requests/request_000019.json").read_text())
        heading = reference["agent_pose_at_observation"][2]
        radius = p["robot_radius_m"]+p["human_radius_m"]
        passage = human_path_metrics(r["xy"], hp[0], radius, heading)
        local = bool(technical and passage["downstream_index"] is not None and not human_contacts
                     and passage["minimum_clearance_m"] > 0 and
                     passage["net_progress_after_clearing_m"] >= p["minimum_post_human_progress_m"])
        full = bool(technical and len(entries) and not human_contacts)
    prediction = []
    for e, c, w in zip(r["events"], r["chunks"], r["worlds"]):
        metrics = human_path_metrics(w, human["position"], radius, heading) if human else {}
        vis = json.loads((run/"raw/visibility"/(Path(e["rgb_observation_reference"]).name+".json")).read_text()) if human else {}
        prediction.append({"request_id": e["request_id"], "sim_time": e["observation"]["sim_time"],
            "human_distance_m": float(np.linalg.norm(np.array(e["agent_pose_at_observation"][:2])-human["position"][:2])) if human else None,
            "human_visible_pixels": vis.get("human_visible_pixel_count"), "visibility": vis.get("state"),
            "predicted_human_clearance_m": metrics.get("minimum_clearance_m"), "predicted_side": metrics.get("side"),
            "centre_plane_lateral_m": metrics.get("centre_plane_lateral_m"),
            "mean_native_left_m": float(c[:, 1].mean()), "endpoint_native_left_m": float(c[-1, 1]),
            "lookahead_index": e["lookahead_index"], "lookahead_forward_m": float(c[e["lookahead_index"], 0]),
            "lookahead_left_m": float(c[e["lookahead_index"], 1]), "target_v": e["controller_command"][0],
            "target_w": e["controller_command"][1], "mode": e["controller_mode"],
            "accepted": e["command_accepted"]})
    summary = {"stage": stage, "run": str(run), "technical_pass": technical,
        "stage_a_classification": ("PAPERLIKE_NO_HUMAN_CONTROL_PASS" if gate_pass else "PAPERLIKE_NO_HUMAN_CONTROL_FAILED") if stage == "A" else None,
        "local_human_avoidance": ("LOCAL_HUMAN_AVOIDANCE_PASS" if local else "LOCAL_HUMAN_AVOIDANCE_FAIL") if human else "NOT_EXECUTED",
        "full_navigation": ("FULL_NAVIGATION_PASS" if full else "FULL_NAVIGATION_FAIL") if human else None,
        "goal_region_reached": bool(len(entries)), "goal_entry_sim_s": float(r["t"][entries[0]]) if len(entries) else None,
        "goal_distance_final_m": float(distance[-1]), "goal_distance_min_m": float(distance.min()),
        "corner_gate": gate, "prediction_count": len(r["events"]), "sensor_count": len(sensor),
        "termination": runtime["termination"] if runtime else "TECHNICAL_FAILURE", "cadence": cadence,
        "human_passage": passage, "human_contacts": human_contacts, "structural_contacts": structure,
        "path_length_m": float(np.linalg.norm(np.diff(r["xy"], axis=0), axis=1).sum()),
        "checkpoint_parity": "PUBLIC_CHECKPOINT_ONLY_RL_PARITY_UNRESOLVED"}
    write_json(output/"summary.json", summary)
    record_evaluation(setup/f"evaluation-{stage}.json", summary)
    write_json(output/"prediction_metrics.json", prediction)
    with (output/"prediction_metrics.csv").open("x") as f:
        writer = csv.DictWriter(f, fieldnames=list(prediction[0]))
        writer.writeheader(); writer.writerows(prediction)
    derived = output/"derived"; derived.mkdir()
    for e, w in zip(r["events"], r["worlds"]):
        with (derived/f"request_{e['request_id']:06d}_world.npy").open("xb") as f:
            np.save(f, w)
    plot(r, summary, output, receipt)
    unchanged = all(digest(path) == h for path, h in hashes.items())
    if not unchanged:
        raise RuntimeError("Source files changed during analysis")
    write_json(output/"validation.json", {"source_unchanged": unchanged, "source_run_sha256": hashes,
        "native_observation_pose_and_clock_checks": "PASS", **provenance(ROOT)})
    print(json.dumps({k: v for k, v in summary.items() if k not in ["structural_contacts", "human_contacts", "corner_gate"]}, indent=2))


def plot(r, summary, output, receipt):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    figures = output/"figures"; figures.mkdir()
    controls = [json.loads(s) for s in (r["path"]/"raw/control.jsonl").read_text().splitlines()]
    sources = [("Stage "+summary["stage"], r)]
    if summary["stage"] == "B":
        sources.insert(0, ("Stage A no human", load(Path(receipt["authorized_runs"]["A"]))))
    inventory_path = ROOT/"outputs/hospital-lights-audit-20261010-01/collision_inventory.json"
    if not inventory_path.exists():
        rules = json.loads((ROOT/"configs/research/reveal_window_search.json").read_text())
        inventory_path = Path(rules["collision_inventory"])
    walls = wall_inventory(json.loads(inventory_path.read_text()))
    human = r["cfg"].get("stationary_human")
    # Readable fixed time-spaced chunks, never severity-ranked. Export every
    # prediction separately, even when only this fixed subset is drawn.
    selected = [i for i, e in enumerate(r["events"]) if e["observation_id"] % 10 == 1]
    sidecar = {"runs": {label: {"time": s["t"].tolist(), "xy": s["xy"].tolist(),
                "goal_distance_m": np.linalg.norm(s["xy"]-s["cfg"]["scene"]["goal"][:2], axis=1).tolist()}
                for label, s in sources}, "controls": controls,
        "observations": [{"request_id": e["request_id"], "sim_time": e["observation"]["sim_time"],
            "application_sim_time": e["application"]["sim_time"], "pose": e["agent_pose_at_observation"],
            "world_points": w.tolist()} for e, w in zip(r["events"], r["worlds"])],
        "plotted_chunk_ids": [r["events"][i]["request_id"] for i in selected], "walls": walls,
        "human": human, "goal": r["cfg"]["scene"]["goal"], "summary": summary}
    write_json(output/"plot_coordinates.json", sidecar)
    for zoom in [False, True]:
        fig, ax = plt.subplots(figsize=(12, 7))
        for wall in walls:
            lo, hi = wall["world_aabb_min"], wall["world_aabb_max"]
            ax.add_patch(Rectangle(lo[:2], hi[0]-lo[0], hi[1]-lo[1], color=".7", alpha=.25))
        for label, s in sources:
            ax.plot(*s["xy"].T, label=label, lw=2)
        ax.scatter(*r["xy"][0], marker="o", label="Start")
        ax.scatter(*r["xy"][-1], marker="x", s=75, label="Final")
        goal = r["cfg"]["scene"]["goal"][:2]
        ax.scatter(*goal, marker="*", s=120, label="Goal")
        ax.add_patch(Circle(goal, 1.5, fill=False, color="green", ls=":"))
        if human:
            h = human["position"][:2]
            ax.add_patch(Circle(h, .25, color="crimson", label="Stationary human proxy"))
            ax.add_patch(Circle(h, .25+r["cfg"]["paperlike"]["robot_radius_m"], fill=False, color="crimson", ls="--", label="Exclusion radius"))
        if zoom:
            for i in selected:
                ax.plot(*r["worlds"][i].T, color="orange", alpha=.5, lw=.8)
            ax.plot([], [], color="orange", label="Native predictions, own observation anchors")
            ax.set_xlim(-2, 9); ax.set_ylim(7, 14)
        else:
            xy = np.concatenate([s["xy"] for _, s in sources]+[np.array([goal])])
            ax.set_xlim(xy[:, 0].min()-2, xy[:, 0].max()+2)
            ax.set_ylim(xy[:, 1].min()-2, xy[:, 1].max()+2)
        ax.set_aspect("equal"); ax.set_xlabel("World X [m]"); ax.set_ylabel("World Y [m]")
        ax.set_title(f"Paper-like positive control — Stage {summary['stage']} — {summary['termination']}")
        ax.legend(fontsize=8, loc="upper right"); fig.tight_layout()
        fig.savefig(figures/("interaction_zoom.png" if zoom else "world_trajectory.png"), dpi=170); plt.close(fig)
    ct = np.array([c["sim_time"] for c in controls])
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for i, ax in enumerate(axes[:2]):
        ax.plot(ct, [c["target_vw"][i] for c in controls], label="Held target")
        ax.plot(ct, [c["applied_vw"][i] for c in controls], label="Applied after slew/priority", ls="--")
        ax.set_ylabel(["v [m/s]", "w [rad/s]"][i]); ax.legend()
    ts = [e["observation"]["sim_time"] for e in r["events"]]
    axes[2].scatter(ts[1:], np.diff(ts), s=6, label="Action-observation gap at later application")
    axes[2].axhline(.1, color="black", ls=":", label="0.1 s target; backup skips action calls")
    axes[2].set_ylabel("Gap [sim s]"); axes[2].set_yscale("log"); axes[2].legend(fontsize=8)
    for field, label in [("action_call_wall_s", "Action call"),
                         ("observation_to_application_wall_s", "Observation to application (C1 includes cold startup)")]:
        if field == "action_call_wall_s":
            values = [(e["ready"]["monotonic_ns"]-e["inference_start"]["monotonic_ns"])/1e9 for e in r["events"]]
        else:
            values = [(e["application"]["monotonic_ns"]-e["observation"]["monotonic_ns"])/1e9 for e in r["events"]]
        axes[3].scatter(ts, values, s=6, label=label, alpha=.7)
    axes[3].axhline(.1, color="black", ls=":"); axes[3].legend(fontsize=8)
    axes[3].set_yscale("log"); axes[3].set_ylabel("Latency [wall s]")
    axes[3].set_xlabel("Action application simulation time [s]")
    fig.tight_layout(); fig.savefig(figures/"commands_cadence.png", dpi=170); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for label, s in sources:
        axes[0].plot(s["t"], np.linalg.norm(s["xy"]-goal, axis=1), label=label)
    axes[0].axhline(1.5, color="green", ls=":", label="Goal 1.5 m"); axes[0].set_ylabel("Goal distance [m]"); axes[0].legend()
    if human:
        for label, s in sources:
            clear = np.linalg.norm(s["xy"]-human["position"][:2], axis=1)-.25-r["cfg"]["paperlike"]["robot_radius_m"]
            axes[1].plot(s["t"], clear, label=label+(" (counterfactual geometry only)" if "no human" in label else ""))
        axes[1].axhline(0, color="red", ls=":"); axes[1].legend()
    else:
        axes[1].text(.5, .5, "Stage A: no human; human clearance is undefined", ha="center", transform=axes[1].transAxes)
    axes[1].set_xlabel("Simulation time [s]"); axes[1].set_ylabel("Human proxy clearance [m]")
    fig.tight_layout(); fig.savefig(figures/"clearance_goal.png", dpi=170); plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--setup-dir", required=True)
    p.add_argument("--stage", choices=["A", "B"], required=True)
    p.add_argument("--output-dir", required=True)
    a = p.parse_args()
    analyze(Path(a.setup_dir).resolve(), a.stage, Path(a.output_dir).resolve())
