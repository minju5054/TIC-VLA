"""Static scientific figures for saved route/seam evidence; never edit raw RGB."""
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from research.route_geometry import scene_obstacles, rectangle


def plot_all(runs, output, radius):
    figures = Path(output)/"figures"
    figures.mkdir()

    def save(fig, name):
        fig.savefig(figures/(name+".png"), dpi=170, bbox_inches="tight")
        fig.savefig(figures/(name+".pdf"), bbox_inches="tight")
        plt.close(fig)

    def scene(ax, run):
        for obstacle in scene_obstacles(run["cfg"]):
            r = rectangle(obstacle)
            ax.add_patch(Rectangle(r[:2], r[2]-r[0], r[3]-r[1], color="0.65", alpha=.7))
            if obstacle["name"] == "central_blocker":
                expanded = rectangle(obstacle, radius)
                ax.add_patch(Rectangle(expanded[:2], expanded[2]-expanded[0], expanded[3]-expanded[1],
                                       fill=False, edgecolor="black", linestyle=":", label="Inflated blocker"))
                ax.axvline(run["cfg"]["route_switch"]["decision_gate_x"], color="purple", ls="--", alpha=.5, label="Decision gate")
        ax.scatter(*run["cfg"]["scene"]["goal"][:2], marker="*", color="green", s=100, label="Goal")
        human_path = run["path"]/"raw/pedestrian_state.csv"
        if human_path.is_file():
            with human_path.open() as f:
                rows = list(csv.DictReader(f))
            ax.plot([float(r["physx_x"]) for r in rows], [float(r["physx_y"]) for r in rows],
                    color="magenta", ls="--", label="Measured human")
        ax.set(xlabel="World X [m]", ylabel="World Y / left [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.2)

    for run in runs:
        robot = np.array([[float(r["x"]), float(r["y"])] for r in run["robot"]])
        times = np.array([float(r["sim_time"]) for r in run["robot"]])
        fig, ax = plt.subplots(figsize=(11, 6), layout="constrained")
        scene(ax, run)
        ax.plot(*robot.T, color="black", lw=2.5, label="Measured robot")
        for event, world in zip(run["events"], run["worlds"]):
            ax.plot(*world.T, lw=1, alpha=.75, label=f"C{event['request_id']}")
            ax.scatter(*event["agent_pose_at_observation"][:2], s=10, color="black")
        ax.set_title(f"{run['variant']}: all native chunks projected at their own observation poses")
        ax.set_ylim(-3, 3)
        ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
        save(fig, f"{run['variant']}_world")
        eligible = [r for r in run["pairs"] if r["eligible_after_warmup"] and r["raw_boundary_position_gap_m"] is not None]
        candidates = [r for r in eligible if r["reconciliation_relevant_candidate"]]
        selections = candidates or ([max(eligible, key=lambda r: r["raw_boundary_position_gap_m"])] if eligible else [])
        # Always show the predeclared static gate C3/C4, even if it failed.
        if run["variant"] == "static":
            selections = [r for r in run["pairs"] if r["fresh_request_id"] == 4] + selections
        for row in selections:
            i = row["fresh_request_id"]-1
            old, fresh = run["events"][i-1:i+1]
            detail = run["details"][i-1]["seam"]
            op, fp = run["worlds"][i-1:i+1]
            obs, app = fresh["observation"], fresh["application"]
            segment = robot[(times >= obs["sim_time"]-1e-8) & (times <= app["sim_time"]+1e-8)]
            boundary, expected = np.array(app["position"][:2]), np.array(detail["fresh_expected_xy"])
            fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")
            scene(ax, run)
            ax.plot(*op.T, "o-", ms=2, color="tab:blue", label=f"OLD C{old['request_id']} {row['old_route_side']}")
            ax.plot(*fp.T, "o-", ms=2, color="tab:orange", label=f"FRESH C{fresh['request_id']} {row['fresh_route_side']}")
            ax.plot(*segment.T, color="black", lw=3, label="Executed OLD while pending")
            ax.scatter(*boundary, color="red", marker="x", s=70, label="Actual switch B")
            if "route_switch" in run["cfg"]:
                gate = run["cfg"]["route_switch"]["decision_gate_x"]
                for value, color in [(row["old_gate_y"], "tab:blue"), (row["fresh_gate_y"], "tab:orange")]:
                    if value is not None:
                        ax.scatter(gate, value, marker="D", color=color, s=45)
            for event, color in [(old, "tab:blue"), (fresh, "tab:orange")]:
                h = event["human_at_observation"]
                if h:
                    ax.scatter(*h["position"][:2], color=color, marker="s", s=55)
            ax.set_title(f"{run['variant']} C{old['request_id']} → C{fresh['request_id']}\n{row['classification']}", fontsize=10)
            ax.text(0, -.2, f"RGB: rgb_{old['request_id']:06d}.png / rgb_{fresh['request_id']:06d}.png", transform=ax.transAxes, fontsize=9)
            ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
            save(fig, f"{run['variant']}_pair_{old['request_id']:02d}_{fresh['request_id']:02d}")
            fig, ax = plt.subplots(figsize=(8, 6), layout="constrained")
            ax.plot(*segment.T, "k.-", label="Measured OLD during pending")
            ax.plot(*np.vstack([obs["position"][:2], fp[:5]]).T, ".-", color="tab:orange",
                    label="FRESH + derived tau=0 anchor")
            ax.plot([boundary[0], expected[0]], [boundary[1], expected[1]], "r--", label="RAW position seam")
            ax.scatter(*boundary, marker="x", s=100, color="red", label="Actual switch B")
            ax.scatter(*expected, marker="D", color="tab:orange", label="FRESH p(tau_switch)")
            for origin, key, color in [(boundary, "executed_tangent_rad", "black"), (expected, "fresh_tangent_rad", "tab:orange")]:
                phi = detail[key]
                if phi is not None:
                    ax.quiver(*origin, np.cos(phi)*.12, np.sin(phi)*.12, angles="xy", scale_units="xy", scale=1, color=color)
            gap = row["raw_boundary_position_gap_m"]
            tangent = row["raw_executed_to_fresh_tangent_gap_deg"]
            tangent_label = f"{tangent:.3f}" if tangent is not None else "undefined"
            ax.set_title(f"{run['variant']} C{old['request_id']} → C{fresh['request_id']}: gap {gap:.4f} m\n"
                         f"Executed/FRESH tangent: {tangent_label} deg", fontsize=10)
            ax.text(0, -.13, "Derived geometric tangents; not native TIC-VLA yaw. No correction applied.", transform=ax.transAxes, fontsize=8)
            ax.set(xlabel="World X [m]", ylabel="World Y [m]")
            ax.set_aspect("equal", adjustable="datalim"); ax.grid(alpha=.25)
            ax.legend(fontsize=8)
            save(fig, f"{run['variant']}_raw_seam_{fresh['request_id']:02d}")
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.8), layout="constrained")
    for ax, key, label in zip(axes, ["raw_boundary_position_gap_m", "raw_executed_to_fresh_tangent_gap_deg", "delta_w"],
                              ["RAW boundary gap [m]", "Executed/FRESH tangent gap [deg]", "Absolute command jump [rad/s]"]):
        for j, run in enumerate(runs):
            values = [abs(r[key]) for r in run["pairs"] if r["eligible_after_warmup"] and r[key] is not None]
            ax.scatter(np.full(len(values), j), values, alpha=.65)
            if values:
                ax.plot([j-.2, j+.2], [np.median(values)]*2, color="black", lw=2)
        ax.set_xticks(range(len(runs)), [r["variant"] for r in runs], rotation=20)
        ax.set_ylabel(label); ax.grid(axis="y", alpha=.25)
    fig.suptitle("Post-warm-up raw diagnostics (FRESH ID > 4); black bar = median\n"
                 "Invalid paths remain descriptive VLA failures, not hard-case candidates", fontsize=11)
    save(fig, "easy_control_comparison")
