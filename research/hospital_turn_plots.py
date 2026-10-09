"""Static scientific figures of saved Hospital evidence; no scene reconstruction."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot_all(output, run, rows, worlds, summary, motion):
    target = output/"figures"
    target.mkdir()
    t, xy, local, yaw = motion
    ep = run["cfg"]["official_episode"]["episode"]
    ids = [r["request_id"] for r in rows]
    obs = np.array([[r["robot_world_x"], r["robot_world_y"]] for r in rows])
    first = summary["phases"]["first_stable_left_request"]

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(target/name, dpi=170)
        plt.close(fig)

    def series(ax, key, label):
        ax.plot(ids, [r[key] for r in rows], ".-", label=label)
        ax.axvspan(.5, 3.5, alpha=.12, color="gray", label="excluded startup C1-C3")
        if first:
            ax.axvline(first, color="purple", ls="--", label=f"first stable C{first}")
        for r in rows:
            if r["degraded_near_black"]:
                ax.axvspan(r["request_id"]-.45, r["request_id"]+.45, color="red", alpha=.1)
        ax.grid(alpha=.3); ax.legend(fontsize=8); ax.set_xlabel("Request ID")

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(*xy.T, color="black", label="measured robot")
    ax.scatter(*obs.T, c=ids, cmap="viridis", s=18, label="observations")
    for rid, point in zip(ids, obs):
        if rid == 1 or rid % 4 == 0 or rid == first:
            ax.annotate(f"C{rid}", point, fontsize=8)
    ax.scatter(*ep["start"][:2], marker="s", label="official start")
    ax.scatter(*ep["goal"][:2], marker="*", s=160, label="goal diagnostic")
    ax.set(xlabel="World X [m]", ylabel="World Y [m]", title="Official Hospital: measured trajectory (scene mesh not plotted)")
    ax.set_aspect("equal", adjustable="datalim"); ax.grid(alpha=.3); ax.legend()
    save(fig, "actual_world_trajectory.png")

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(*local.T, color="black")
    ax.scatter(0, 0, marker="s", label="official start frame")
    ax.set(xlabel="Episode-local forward [m]", ylabel="Episode-local left [m]", title="Measured motion in official start frame (+Y world forward)")
    ax.set_aspect("equal", adjustable="datalim"); ax.grid(alpha=.3); ax.legend()
    save(fig, "actual_episode_local_trajectory.png")

    fig, axes = plt.subplots(2, 1, figsize=(11, 7))
    for ax, key, label in zip(axes, ["controller_v", "controller_w"], ["v [m/s]", "w [rad/s]"]):
        series(ax, key, label); ax.set_ylabel(label)
    save(fig, "controller_by_request.png")

    fig, axes = plt.subplots(4, 1, figsize=(11, 12))
    for ax, key, label in zip(axes,
        ["native_endpoint_left_m", "lookahead_left_m", "lookahead_bearing_deg", "derived_total_heading_change_deg"],
        ["Endpoint left [m]", "Lookahead left [m]", "Lookahead bearing [deg]", "Derived total heading change [deg]"]):
        series(ax, key, label); ax.set_ylabel(label)
    fig.suptitle("Derived geometric tangent; not native TIC-VLA yaw")
    save(fig, "chunk_turn_geometry.png")

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, np.degrees(yaw-yaw[0]), label="measured yaw relative to settled start")
    if first:
        ax.axvline(rows[first-1]["observation_sim_time"], color="purple", ls="--", label=f"C{first} stable intent observed")
    ax.set(xlabel="Simulation time [s]", ylabel="Unwrapped yaw change [deg]")
    ax.grid(alpha=.3); ax.legend(); save(fig, "actual_yaw.png")

    selected = sorted({i for i in [summary["phases"]["last_forward_dominant_request_before_turn"],
                                  summary["phases"]["transition_previous_request"], first] if i})
    if not selected:
        selected = [run["cfg"]["hospital_validation"]["first_primary_request_id"], len(rows)]
    fig, axes = plt.subplots(1, 2, figsize=(11, 7))
    ax, native_ax = axes
    ax.plot(*xy.T, color="gray", alpha=.6, label="measured robot")
    for rid in selected:
        p, r = worlds[rid-1], rows[rid-1]
        line, = ax.plot(*p.T, ".-", label=f"C{rid}, w={r['controller_w']:.3f} rad/s")
        ax.scatter(r["robot_world_x"], r["robot_world_y"], marker="x", s=85, color=line.get_color())
        native_ax.plot(*run["chunks"][rid-1].T, ".-", color=line.get_color(), label=f"C{rid}")
    bounds = np.vstack([worlds[i-1] for i in selected]+[obs[np.array(selected)-1]])
    ax.set_xlim(bounds[:, 0].min()-.3, bounds[:, 0].max()+.3)
    ax.set_ylim(bounds[:, 1].min()-.3, bounds[:, 1].max()+.3)
    ax.set(xlabel="World X [m]", ylabel="World Y [m]", title="Own observation world projection\nx = observation origin; actual line clipped")
    native_ax.set(xlabel="Native forward [m]", ylabel="Native left [m]", title="Separate observation body frames\nNative geometry context only")
    for a in axes:
        a.set_aspect("equal", adjustable="box"); a.grid(alpha=.3); a.legend(fontsize=8)
    fig.suptitle("Automatic selection: previous C3 is excluded startup; first eligible stable C4"
                 if selected == [3, 4] else "Automatically selected phase context")
    save(fig, "selected_chunks_world.png")
