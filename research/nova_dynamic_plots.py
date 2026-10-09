"""Scientific saved-only figures for the fixed dynamic-human primary pair."""
import numpy as np


def plot_all(output,dynamic,baseline,results):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    figs=output/"figures";figs.mkdir()
    p=results["dynamic"]["primary_pair"];details=results["dynamic"]["details"]
    if len(dynamic["events"])<16:return
    old,fresh=dynamic["events"][14:16];ow,fw=dynamic["worlds"][14:16]
    human=np.array(fresh["human_at_observation"]["position"][:2])
    boundary=np.array(fresh["application"]["position"][:2]);obs=np.array(fresh["observation"]["position"][:2])
    trigger=np.array(dynamic["trigger"]["application"]["position"][:2])
    def save(fig,name):
        fig.tight_layout();fig.savefig(figs/(name+".png"),dpi=170);plt.close(fig)
    def world(ax):
        ax.plot(*dynamic["xy"].T,color="c",label="Measured robot")
        ax.plot(*dynamic["human_xy"].T,color="m",label="Measured human")
        ax.plot(*ow.T,color="darkorange",marker=".",label="OLD C15: own observation world")
        ax.plot(*fw.T,color="green",marker=".",label="FRESH C16: own observation world")
        ax.scatter(*trigger,c="blue",marker="*",s=100,label="C15 application / trigger")
        ax.scatter(*obs,c="gold",edgecolors="k",marker="s",label="C16 observation")
        ax.scatter(*boundary,facecolors="white",edgecolors="k",marker="D",label="RAW switch boundary B",zorder=6)
        ax.add_patch(Circle(human,p["conflict_radius_m"],color="red",alpha=.15,label="Human + Nova proxy at C16 obs"))
        ax.scatter(*human,c="m",marker="x",s=70);ax.set_aspect("equal")
        ax.set(xlabel="Hospital world X [m]",ylabel="Hospital world Y [m]")
        ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig,ax=plt.subplots(figsize=(12,7));world(ax);ax.set_title("Recorded dynamic run; Hospital coordinates, scene mesh omitted");save(fig,"world_overview")
    fig,ax=plt.subplots(figsize=(9,8));world(ax)
    points=np.vstack([ow,fw,dynamic["human_xy"],boundary]);lo=points.min(0)-.5;hi=points.max(0)+.5
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),title=f"Prespecified C15→C16 | overlap RMSE {p['overlap_rmse_m']:.3f} m")
    save(fig,"primary_pair_closeup")
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    aligned=details["alignment"];times=np.array(aligned["absolute_times"])
    delta=np.array(details["fresh_frame_delta"])
    for ax,values,label in zip(axes,[delta[:,0],delta[:,1],np.linalg.norm(delta,axis=1)],["Signed FRESH forward revision [m]","Signed FRESH lateral revision [m]","Euclidean revision [m]"]):
        ax.plot(times,values);ax.set_ylabel(label);ax.grid(alpha=.3)
    axes[-1].set_xlabel("Common absolute nominal target time [sim s]")
    axes[0].set_title("Nominal temporal interpolation; no spatial matching")
    save(fig,"temporal_overlap_revision")
    seam=p["raw_seam"];expected=np.array(seam["fresh_expected_xy"]) if seam["fresh_expected_xy"] is not None else None
    fig,ax=plt.subplots(figsize=(9,7));mask=(dynamic["times"]>=fresh["observation"]["sim_time"])&(dynamic["times"]<=fresh["application"]["sim_time"])
    ax.plot(*dynamic["xy"][mask].T,color="c",marker=".",label="Measured OLD while C16 pending")
    ax.plot(*np.vstack([obs,fw]).T,color="green",marker=".",label="FRESH nominal diagnostic curve")
    ax.scatter(*obs,c="gold",edgecolors="k",label="Observation (derived tau=0 anchor)")
    ax.scatter(*boundary,facecolors="white",edgecolors="k",marker="D",label="Switch B",zorder=6)
    if expected is not None:
        ax.scatter(*expected,c="red",marker="x",label="FRESH expected at tau")
        ax.plot(*np.vstack([boundary,expected]).T,"r--",label=f"RAW position seam {seam['raw_boundary_position_gap_m']:.4f} m")
        points=np.vstack([dynamic["xy"][mask],expected]);lo=points.min(0)-.12;hi=points.max(0)+.12
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
    ax.set_aspect("equal");ax.set(xlabel="World X [m]",ylabel="World Y [m]",title="Uncorrected observation-anchored FRESH at application")
    ax.grid(alpha=.3);ax.legend(fontsize=8);save(fig,"raw_seam")
    fig,axes=plt.subplots(2,1,figsize=(11,8))
    for name,color in [("old","darkorange"),("fresh","green")]:
        fixed=details.get(name+"_fixed_human_clearance");moving=details.get(name+"_dynamic_clearance")
        if fixed:axes[0].plot(fixed["absolute_times"],fixed["clearance_m"],color=color,label=name.upper())
        if moving:axes[1].plot(moving["absolute_times"],moving["sample_clearance_m"],color=color,label=name.upper())
    for ax,label in zip(axes,["Human fixed at C16 observation","Time-varying measured human, nominal robot targets"]):
        ax.axhline(0,color="red",ls="--");ax.set(ylabel="Inflated clearance [m]",xlabel="Nominal absolute time [s]",title=label);ax.grid(alpha=.3);ax.legend()
    save(fig,"human_clearance")
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    t=dynamic["times"];robot=dynamic["robot"]
    for ax,key in zip(axes[:2],["target_v","target_w"]):
        ax.step(t,[float(r[key]) for r in robot],where="pre");ax.set_ylabel(key+(" [m/s]" if key=="target_v" else " [rad/s]"))
    axes[2].plot(dynamic["human_times"],np.linalg.norm(dynamic["human_xy"]-dynamic["human_xy"][0],axis=1),color="m")
    axes[2].set(ylabel="Human displacement [m]",xlabel="Simulation time [s]")
    for ax in axes:
        ax.axvline(dynamic["trigger"]["trigger_sim_time"],color="m",label="C15 trigger")
        for e in dynamic["events"]:
            ax.axvline(e["observation"]["sim_time"],color="gray",alpha=.12)
            ax.axvline(e["application"]["sim_time"],color="blue",alpha=.12,ls="--")
        ax.axvspan(fresh["observation"]["sim_time"],fresh["application"]["sim_time"],color="gold",alpha=.3,label="C16 pending / C15 active")
        ax.grid(alpha=.3)
    axes[0].legend();save(fig,"command_timing")
    labels=["No-human C15→C16","Dynamic-human C15→C16"]
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,key,title in zip(axes,["overlap_rmse_m","mean_abs_lateral_revision_m","raw_boundary_position_gap_m"],["Overlap RMSE [m]","Mean absolute lateral [m]","RAW position seam [m]"]):
        values=[]
        for name in ["baseline","dynamic"]:
            pair=results[name]["primary_pair"];values.append(pair["raw_seam"][key] if key.startswith("raw_") else pair[key])
        ax.bar(labels,values,color=["gray","teal"]);ax.set_title(title);ax.tick_params(axis="x",labelrotation=15)
    save(fig,"baseline_comparison")
