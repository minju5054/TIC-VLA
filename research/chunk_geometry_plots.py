"""Standalone scientific figures from saved, observation-anchored geometry."""
import os
from pathlib import Path

import numpy as np
from research.analyze_chunk_geometry import project_world, human_at


def render_figures(run, rows, details, directory, static_rows=None):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/ticvla-geometry-matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    directory = Path(directory); directory.mkdir()
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": .18, "figure.facecolor": "white",
                         "savefig.facecolor": "white", "pdf.fonttype": 42})
    requests, robot = run["requests"], run["robot"]
    curves = [project_world(r["native"], r["pose"]) for r in requests]
    colors = plt.get_cmap("viridis")(np.linspace(.08, .90, len(requests)))
    all_points = np.concatenate(curves)
    zoom_min, zoom_max = all_points[:, 1].min()-.10, all_points[:, 1].max()+.10
    x_min = min(all_points[:, 0].min(), robot["x"].min())-.3
    x_max = max(all_points[:, 0].max(), robot["x"].max())+.3
    goal = run["metadata"]["config"]["scene"].get("goal")

    def save(fig, name):
        fig.savefig(directory/(name+".png"), dpi=170, bbox_inches="tight")
        fig.savefig(directory/(name+".pdf"), bbox_inches="tight")
        plt.close(fig)

    def world_axes(ax, zoom=False):
        ax.set_xlabel("World X [m]"); ax.set_ylabel("World Y [m]")
        ax.set_xlim(x_min, max(x_max, goal[0]+.3 if goal else x_max))
        if zoom:
            ax.set_ylim(zoom_min, zoom_max)
            ax.set_title("Lateral detail (unequal axis scales)", loc="left", fontsize=10)
        else:
            ax.set_aspect("equal", adjustable="datalim")

    def context(ax):
        ax.plot(robot["x"], robot["y"], color=".15", ls=":", lw=2, label="Executed robot XY")
        if run["human"] is not None:
            human = run["human"]
            ax.plot(human["physx_x"], human["physx_y"], color="#d55e00", ls="--", lw=1.7,
                    label="Measured human XY")
        if goal:
            ax.scatter(goal[0], goal[1], marker="*", s=120, c="#218739", edgecolor="black", label="Goal")

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), gridspec_kw={"height_ratios": [1.5, 1]}, constrained_layout=True)
    for ax in axes:
        context(ax)
        for request, curve, color in zip(requests, curves, colors):
            ax.plot(curve[:, 0], curve[:, 1], color=color, lw=1.6, label=f"C{request['id']}")
            # The observation origin is a separate marker, never prepended to native data.
            pose = request["pose"]
            ax.scatter(*pose[:2], marker=">", s=35, color=color, edgecolor="black", zorder=5)
            ax.annotate(f"{request['id']}", pose[:2], xytext=(0, -15), textcoords="offset points", ha="center", fontsize=8)
    for request in requests:
        human = human_at(run, request["time"])
        if human["position"] is not None:
            p = human["position"]
            axes[0].scatter(p[0], p[1], marker="x", s=30, c="#d55e00")
            axes[0].annotate(f"H@C{request['id']}", p[:2], xytext=(7, 0), textcoords="offset points", fontsize=8)
    world_axes(axes[0]); world_axes(axes[1], zoom=True)
    axes[0].legend(ncol=4, fontsize=8, loc="upper right")
    fig.suptitle("Eight native chunks projected with their own observation poses\n"
                 "Triangles = observation origins; human at t=0 is unmeasured, not extrapolated", fontsize=13)
    save(fig, "all_chunks_world")

    for index, (row, detail) in enumerate(zip(rows, details)):
        old, fresh = requests[index:index+2]
        ow, fw = curves[index:index+2]
        m, overlap = row["nominal_index_shift"], row["overlap_point_count"]
        fig, axes = plt.subplots(2, 1, figsize=(11, 7.4), gridspec_kw={"height_ratios": [1.3, 1]}, constrained_layout=True)
        for ax in axes:
            ax.plot(ow[:, 0], ow[:, 1], "--", color="#0072b2", alpha=.45, label="OLD full")
            ax.plot(fw[:, 0], fw[:, 1], "--", color="#d55e00", alpha=.45, label="FRESH full")
            ax.plot(ow[m:, 0], ow[m:, 1], color="#0072b2", lw=2, label="OLD aligned overlap")
            ax.plot(fw[:overlap, 0], fw[:overlap, 1], color="#d55e00", lw=2, label="FRESH aligned overlap")
            for j in np.unique(np.linspace(0, overlap-1, 6).astype(int)):
                ax.plot([ow[m+j, 0], fw[j, 0]], [ow[m+j, 1], fw[j, 1]], color=".4", lw=.7, alpha=.6)
            segment = (robot["sim_time"] >= old["time"]-1e-8) & (robot["sim_time"] <= fresh["time"]+1e-8)
            ax.plot(robot["x"][segment], robot["y"][segment], color="black", ls=":", lw=2.5, label="Executed between observations")
            for request, color, label in [(old, "#0072b2", "OLD origin"), (fresh, "#d55e00", "FRESH origin")]:
                ax.scatter(*request["pose"][:2], marker=">", s=55, color=color, edgecolor="black", label=label, zorder=5)
        missing = []
        for key, color, label in [("human_at_old_observation", "#0072b2", "Human @ OLD"),
                                  ("human_at_fresh_observation", "#d55e00", "Human @ FRESH")]:
            h = detail[key]
            if h["position"] is not None:
                axes[0].scatter(*h["position"][:2], marker="x", s=65, color=color, label=label)
            else:
                missing.append(label+" unavailable")
        axes[0].set_aspect("equal", adjustable="datalim")
        axes[0].set_xlabel("World X [m]"); axes[0].set_ylabel("World Y [m]")
        axes[0].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1))
        axes[1].set_ylim(zoom_min, zoom_max)
        axes[1].set_xlabel("World X [m]"); axes[1].set_ylabel("World Y [m]")
        axes[1].set_title("Lateral detail (unequal axis scales); connectors use nominal time only", fontsize=10)
        fig.suptitle(f"C{old['id']} → C{fresh['id']}  |  aligned RMSE {row['aligned_overlap_rmse_m']:.3f} m  |  "
                     f"max {row['aligned_overlap_max_m']:.3f} m\n"
                     f"mean/max |lateral| {row['mean_abs_lateral_revision_m']:.3f}/{row['max_abs_lateral_revision_m']:.3f} m"
                     + ("  |  "+", ".join(missing) if missing else ""), fontsize=12)
        save(fig, f"pair_{old['id']:02d}_{fresh['id']:02d}")

    x = np.arange(1, len(rows)+1)
    labels = [f"{r['old_request_id']}→{r['fresh_request_id']}\ntF={r['fresh_observation_sim_time']:.1f}s" for r in rows]

    def pair_axis(ax):
        ax.set_xticks(x, labels); ax.set_xlabel("Consecutive request IDs / FRESH observation simulation time")
        ax.set_xlim(.7, len(rows)+.3)

    specifications = [
        ("rmse_by_pair", [("aligned_overlap_rmse_m", "Dynamic aligned RMSE", "#0072b2")],
         "Aligned overlap RMSE [m]", "Same absolute nominal future time; observation-pose world projection"),
        ("lateral_revision_by_pair", [("mean_abs_lateral_revision_m", "Mean |lateral|", "#0072b2"),
                                       ("max_abs_lateral_revision_m", "Max |lateral|", "#d55e00")],
         "Absolute lateral revision [m]", "Signed differences resolved in each FRESH observation frame"),
        ("tangent_revision_by_pair", [("mean_abs_tangent_change_deg", "Mean |tangent change|", "#0072b2"),
                                       ("max_abs_tangent_change_deg", "Max |tangent change|", "#d55e00")],
         "Absolute tangent change [deg]", "Derived geometric tangent; not native TIC-VLA yaw"),
        ("human_relative_by_pair", [("human_forward_m", "Human forward", "#0072b2"),
                                    ("human_left_m", "Human left", "#d55e00")],
         "Measured human position in FRESH frame [m]", "Temporal context only; no pedestrian causal-effect claim")]
    for name, fields, ylabel, title in specifications:
        fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
        for field, label, color in fields:
            ax.plot(x, [r[field] for r in rows], "o-", color=color, label=label, lw=1.7)
        if name == "rmse_by_pair" and static_rows:
            ax.plot(x, [r["aligned_overlap_rmse_m"] for r in static_rows], "s--", color=".4", label="Static within-run RMSE")
        if name == "human_relative_by_pair":
            ax.axhline(0, color=".5", lw=.8)
        else:
            ax.set_ylim(bottom=0)
        pair_axis(ax); ax.set_ylabel(ylabel); ax.set_title(title); ax.legend(fontsize=9)
        save(fig, name)

    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    for row, detail, color in zip(rows, details, colors):
        ax.plot(detail["fresh_nominal_horizon_s"], detail["pointwise_displacement_m"], color=color,
                marker=".", ms=4, label=f"{row['old_request_id']}→{row['fresh_request_id']}")
    ax.set_xlabel("Nominal future horizon after FRESH observation [s]")
    ax.set_ylabel("Aligned pointwise displacement [m]"); ax.set_ylim(bottom=0)
    ax.set_title("Where does each revision occur along the common future window?")
    ax.legend(title="Request pair", loc="upper left", bbox_to_anchor=(1.01, 1))
    save(fig, "overlap_error_vs_horizon")
    return matplotlib.__version__
