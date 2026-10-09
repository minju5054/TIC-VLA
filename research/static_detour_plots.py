"""Standalone scientific figures for saved asymmetric static calibration."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from research.route_geometry import rectangle, scene_obstacles


def scene(ax, cfg):
    radius = cfg["static_detour"]["robot_footprint_radius_m"]
    for obj in scene_obstacles(cfg):
        rect = rectangle(obj)
        goal, blocker = obj["name"] == "GoalWall", obj["name"] == "offset_blocker"
        color = "forestgreen" if goal else "saddlebrown" if blocker else "gray"
        ax.add_patch(Rectangle(rect[:2], *(rect[2:]-rect[:2]), color=color, alpha=.4,
                              label="goal wall" if goal else "offset blocker" if blocker else None))
        if blocker:
            inflated = rectangle(obj, radius)
            ax.add_patch(Rectangle(inflated[:2], *(inflated[2:]-inflated[:2]), fill=False,
                                  edgecolor=color, linestyle="--", label="blocker + 0.4 m proxy"))
    ax.axvline(cfg["static_detour"]["decision_gate_x"], color="gray", ls=":", label="decision gate")
    ax.scatter(*cfg["scene"]["goal"][:2], color="green", marker="*", s=90, zorder=6)
    ax.set(xlabel="World X [m]", ylabel="World Y / scene left [m]", xlim=(-.4, 8.5), ylim=(-2.7, 2.7))
    ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2)


def save(fig, directory, name):
    fig.savefig(directory/(name+".png"), dpi=170, bbox_inches="tight")
    fig.savefig(directory/(name+".pdf"), bbox_inches="tight")
    plt.close(fig)


def plot_all(directory, cfg, rows, worlds, times, xy, clearance):
    directory.mkdir(exist_ok=False)
    primary = [(r, w) for r, w in zip(rows, worlds) if r["eligible_after_startup"]]
    colors = plt.colormaps["viridis"](np.linspace(.05, .95, len(primary)))
    fig, ax = plt.subplots(figsize=(11, 7)); scene(ax, cfg)
    for (r, world), color in zip(primary, colors):
        ax.plot(world[:, 0], world[:, 1], color=color, label=f"C{r['request_id']} {r['route_side']}")
        ax.scatter(r["robot_observation_x"], r["robot_observation_y"], color=color, marker="x", zorder=5)
    ax.plot(xy[:, 0], xy[:, 1], "k-", lw=2, label="actual measured robot")
    ax.set_title("Static detour: C4 onward, own-observation world projection\nCrosses mark observation origins; native arrays unchanged")
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.14), ncol=3, fontsize=8)
    save(fig, directory, "world_overview")
    for (r, world), color in zip(primary, colors):
        fig, ax = plt.subplots(figsize=(10, 6)); scene(ax, cfg)
        ax.plot(xy[:, 0], xy[:, 1], "k-", alpha=.3, label="actual robot")
        ax.plot(world[:, 0], world[:, 1], "o-", color=color, ms=3, label="native projected chunk")
        x, y, yaw = [r[key] for key in ["robot_observation_x", "robot_observation_y", "robot_observation_yaw"]]
        ax.scatter(x, y, color="black", marker="x", s=60, label="observation origin")
        ax.arrow(x, y, .35*np.cos(yaw), .35*np.sin(yaw), head_width=.07, color="black")
        if r["gate_y"] is not None:
            ax.scatter(cfg["static_detour"]["decision_gate_x"], r["gate_y"], marker="D", color="red", zorder=6, label="gate crossing")
        ax.set_title(f"C{r['request_id']}: {r['route_side']} | blocker clearance {r['minimum_offset_blocker_clearance_m']:.3f} m\n"
                     f"v={r['controller_v']:.3f} m/s, w={r['controller_w']:.3f} rad/s")
        ax.legend(loc="upper center", bbox_to_anchor=(.5, -.14), ncol=3, fontsize=8)
        save(fig, directory, f"request_{r['request_id']:02d}")
    fig, axes = plt.subplots(2, 1, figsize=(11, 9), gridspec_kw={"height_ratios": [2, 1]})
    scene(axes[0], cfg); axes[0].plot(xy[:, 0], xy[:, 1], "k-", lw=2, label="actual robot")
    axes[0].scatter(*xy[-1], marker="x", color="red", label="final robot")
    axes[0].legend(fontsize=8); axes[0].set_title("Actual measured execution; conservative footprint proxy")
    axes[1].plot(times, clearance); axes[1].axhline(0, color="red", ls="--")
    axes[1].set(xlabel="Simulation time [s]", ylabel="Signed blocker clearance [m]",
                title="Per-tick center clearance; reported minimum also checks between-tick segments")
    axes[1].grid(alpha=.3); fig.tight_layout(); save(fig, directory, "actual_execution")
    for name, field, ylabel in [("controller_w", "controller_w", "Command w [rad/s]"), ("gate_y", "gate_y", "World gate Y [m]")]:
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot([r["request_id"] for r in rows], [np.nan if r[field] is None else r[field] for r in rows], "o-")
        ax.axvspan(.5, 3.5, color="gray", alpha=.15, label="excluded bootstrap/startup")
        if field == "gate_y":
            block = next(o for o in cfg["scene"]["obstacles"] if o["name"] == "offset_blocker")
            low = rectangle(block, cfg["static_detour"]["robot_footprint_radius_m"])[3]
            high = cfg["scene"]["corridor_half_width"]-.05-cfg["static_detour"]["robot_footprint_radius_m"]
            ax.axhspan(low, high, color="green", alpha=.1, label="LEFT gate interval; full path must also be feasible")
        ax.set(xlabel="Request ID", ylabel=ylabel, xticks=[r["request_id"] for r in rows])
        ax.grid(alpha=.3); ax.legend(fontsize=8); fig.tight_layout(); save(fig, directory, name)
