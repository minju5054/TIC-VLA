"""Saved-only primary C15→C16 human conflict, nominal revision and RAW seam."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.nova_evidence import load_nova
from research.nova_response import read_csv
from research.analyze_chunk_geometry import project_world
from research.analyze_handoffs import metrics as latency_metrics
from research.analyze_nova_response import contacts
from research.route_geometry import temporal_overlap,boundary_seam,rotation,wrap,interpolate
from research.nova_dynamic_geometry import polyline_clearance,remaining_curve,temporal_human_clearance,validate_frozen_config
from research.hospital_episode import digest
from research.records import write_json,provenance


def load_saved(path,dynamic=False):
    path=Path(path).resolve()
    complete=(path/"summary.json").is_file()
    if not complete and not (path/"failure.json").is_file():raise ValueError("No completed or recorded failed run")
    run=load_nova(path,require_complete=complete)
    run["complete"]=complete
    run["worlds"]=[project_world(c,e["agent_pose_at_observation"]) for c,e in zip(run["chunks"],run["events"])]
    run["times"]=np.array([float(r["sim_time"]) for r in run["robot"]])
    run["xy"]=np.array([[float(r["x"]),float(r["y"])] for r in run["robot"]])
    run["contacts"]=contacts(path)
    if dynamic:
        human=read_csv(path/"raw/human_state.csv")
        ht=np.array([float(r["sim_time"]) for r in human]);hp=np.array([[float(r[k]) for k in ["x","y"]] for r in human])
        if len(human)!=len(run["robot"]) or np.any(np.diff(ht)<=0) or not np.allclose(ht,run["times"],atol=1e-9,rtol=0):
            raise ValueError("Human and robot require exact physics-tick alignment")
        trigger=json.loads((path/"raw/human_trigger.json").read_text())
        if trigger["trigger_request_id"]!=15 or trigger["depends_on_prediction_content"]:raise ValueError("Invalid trigger")
        if len(run["events"])>=15 and trigger["trigger_sim_time"]!=run["events"][14]["application"]["sim_time"]:
            raise ValueError("Trigger differs from C15 application")
        for e in run["events"]:
            for event,key in [("observation","human_at_observation"),("response_detected","human_at_response_detected"),("application","human_at_application")]:
                h=e[key];i=e[event]["tick"]
                if h is None or abs(h["sim_time"]-ht[i])>1e-9 or not np.allclose(h["position"][:2],hp[i],atol=1e-7):
                    raise ValueError("Human request snapshot not exact matching measured tick")
        run.update(human_rows=human,human_times=ht,human_xy=hp,trigger=trigger)
    return run


def request_metrics(run,settings):
    result=[];radius=settings["robot_radius_m"]+settings["human_radius_m"]
    for i,(event,chunk,world) in enumerate(zip(run["events"],run["chunks"],run["worlds"])):
        obs,app=event["observation"],event["application"];rid=event["request_id"]
        diag=json.loads((run["path"]/"raw/scene_observations"/(Path(event["rgb_observation_reference"]).name+".json")).read_text())
        spans=[s for s in run["contacts"] if s["start_sim_time"]<=app["sim_time"] and (s["end_sim_time"] is None or s["end_sim_time"]>=obs["sim_time"])]
        # contacts retain prim names; human and Hospital are separate contexts.
        human_prefix=run["cfg"].get("dynamic_human",{}).get("prim","/NO_HUMAN")
        human_hits=[s for s in spans if human_prefix in json.dumps(s)]
        static_hits=[s for s in spans if s not in human_hits]
        length=float(np.linalg.norm(np.diff(chunk,axis=0),axis=1).sum())
        usable=bool(length>settings["minimum_tangent_displacement_m"] and np.max(np.linalg.norm(chunk,axis=1))>=run["cfg"]["controller"]["lookahead"])
        clean=not diag["degraded_near_black"] and not diag["camera_near_scene_collision_paths"]
        look=chunk[event["lookahead_index"]];segments=np.diff(world,axis=0)
        valid_segments=np.linalg.norm(segments,axis=1)>settings["minimum_tangent_displacement_m"]
        headings=np.unwrap(np.arctan2(segments[valid_segments,1],segments[valid_segments,0]))
        seam=boundary_seam(world,obs,app,run["times"],run["xy"],dt=settings["native_waypoint_dt"],
            window=settings["executed_tangent_window_s"],tangent_epsilon=settings["minimum_tangent_displacement_m"])
        h=event["human_at_observation"]
        clearance=polyline_clearance(world,h["position"],radius) if h else None
        latency=latency_metrics(event)
        row={"request_id":rid,"observation_sim_time":obs["sim_time"],"application_sim_time":app["sim_time"],
            "robot_x":obs["position"][0],"robot_y":obs["position"][1],"robot_yaw_rad":obs["pose"][2],
            "human_x":h["position"][0] if h else None,"human_y":h["position"][1] if h else None,
            "endpoint_forward_m":float(chunk[-1,0]),"endpoint_left_m":float(chunk[-1,1]),
            "mean_left_m":float(chunk[:,1].mean()),"lookahead_forward_m":float(look[0]),"lookahead_left_m":float(look[1]),
            "lookahead_bearing_deg":float(np.degrees(np.arctan2(look[1],look[0]))),
            "derived_total_tangent_change_deg":float(np.degrees(headings[-1]-headings[0])) if len(headings)>1 else None,
            "target_v":event["controller_command"][0],"target_w":event["controller_command"][1],
            "native_path_length_m":length,"usable_path":usable,"clean_visibility":clean,"rgb_p95":diag["rgb_p95_0_255"],
            "static_contact_span_count":len(static_hits),"human_contact_span_count":len(human_hits),
            "valid_geometry_context":bool(usable and clean and not static_hits and not human_hits),
            "human_clearance_m":clearance,"human_intersection":clearance<=0 if clearance is not None else None,
            "observation_to_switch_sim_s":app["sim_time"]-obs["sim_time"],
            "physics_ticks_inside_action":run["inside_action_ticks"][i],
            **latency,**{k:seam[k] for k in ["raw_boundary_position_gap_m","raw_executed_to_fresh_tangent_gap_deg",
                "raw_robot_heading_to_fresh_tangent_gap_deg","executed_window_displacement_m"]}}
        if "human_times" in run:
            dynamic=temporal_human_clearance(obs["sim_time"]+.1*np.arange(1,31),world,run["human_times"],run["human_xy"],radius)
            row["time_varying_human_min_clearance_m"]=dynamic["minimum_clearance_m"] if dynamic else None
        result.append(row)
    return result


def pair_metrics(run,rows,settings,old_id=15,fresh_id=16):
    if len(run["events"])<fresh_id:
        return {"old_request_id":old_id,"fresh_request_id":fresh_id,"qualification_decision":"INSUFFICIENT EVIDENCE","reason":"Missing primary request evidence"},{}
    old,fresh=run["events"][old_id-1],run["events"][fresh_id-1]
    ow,fw=run["worlds"][old_id-1],run["worlds"][fresh_id-1];oc,fc=rows[old_id-1],rows[fresh_id-1]
    ot,ft=old["observation"]["sim_time"],fresh["observation"]["sim_time"];dt=settings["native_waypoint_dt"]
    aligned=temporal_overlap(ow,fw,ot,ft,dt,settings["minimum_tangent_displacement_m"])
    seam=boundary_seam(fw,fresh["observation"],fresh["application"],run["times"],run["xy"],dt=dt,
        window=settings["executed_tangent_window_s"],tangent_epsilon=settings["minimum_tangent_displacement_m"])
    local=aligned["delta"]@rotation(fresh["agent_pose_at_observation"][2]) if aligned else None
    norm=np.linalg.norm(aligned["delta"],axis=1) if aligned else None
    human=fresh["human_at_observation"];radius=settings["robot_radius_m"]+settings["human_radius_m"]
    rt,rp=remaining_curve(ow,ot,ft,dt)
    old_clear=polyline_clearance(rp,human["position"],radius) if human and rp is not None else None
    fresh_clear=polyline_clearance(fw,human["position"],radius) if human else None
    old_initial=oc["human_clearance_m"]
    old_valid=bool(oc["valid_geometry_context"] and (old_initial is None or old_initial>0))
    fresh_valid=fc["valid_geometry_context"]
    change=aligned is not None and aligned["rmse"]>settings["numerical_position_tolerance_m"]
    conflict=old_clear is not None and old_clear<=0 and old_valid
    improved=conflict and fresh_clear>0
    transport=fc["physics_ticks_inside_action"]>0 and fc["robot_translation_m"]>settings["numerical_position_tolerance_m"]
    seam_present=seam["raw_boundary_position_gap_m"] is not None and seam["raw_boundary_position_gap_m"]>settings["numerical_position_tolerance_m"]
    if human is None:decision="SAVED NO-HUMAN DESCRIPTIVE CONTROL"
    elif not oc["valid_geometry_context"] or not fresh_valid:decision="INSUFFICIENT EVIDENCE"
    elif old_valid and conflict and change and improved and transport and seam_present:decision="RECONCILIATION-RELEVANT DYNAMIC HARD CASE ESTABLISHED"
    elif old_valid and conflict and change:decision="VALID INTENT REVISION, HARD-CASE QUALIFICATION INCOMPLETE"
    else:decision="NO RECONCILIATION-RELEVANT INTENT REVISION"
    result={"old_request_id":old_id,"fresh_request_id":fresh_id,"primary":(old_id,fresh_id)==(15,16),
        "trigger_event":run.get("trigger"),"old_valid":old_valid,"fresh_valid":bool(fresh_valid),
        "old_context":oc,"fresh_context":fc,"old_initial_human_clearance_m":old_initial,
        "human_at_old_observation":old["human_at_observation"],"human_at_old_application":old["human_at_application"],
        "human_at_fresh_observation":human,"human_at_fresh_application":fresh["human_at_application"],
        "old_human_min_clearance_m":old_clear,"fresh_human_min_clearance_m":fresh_clear,
        "old_human_intersection":old_clear<=0 if old_clear is not None else None,
        "fresh_human_intersection":fresh_clear<=0 if fresh_clear is not None else None,
        "clearance_improvement_m":fresh_clear-old_clear if human and old_clear is not None else None,
        "conflict_radius_m":radius,"valid_old_conflict_fresh_avoidance_pair":bool(old_valid and fresh_valid and improved),
        "temporal_overlap_absolute_domain":aligned["absolute_times"][[0,-1]].tolist() if aligned else None,
        "overlap_point_count":len(norm) if aligned else 0,"overlap_rmse_m":aligned["rmse"] if aligned else None,
        "overlap_mean_m":float(norm.mean()) if aligned else None,"overlap_max_m":float(norm.max()) if aligned else None,
        "mean_signed_forward_revision_m":float(local[:,0].mean()) if aligned else None,
        "mean_abs_forward_revision_m":float(np.abs(local[:,0]).mean()) if aligned else None,
        "max_abs_forward_revision_m":float(np.abs(local[:,0]).max()) if aligned else None,
        "mean_signed_lateral_revision_m":float(local[:,1].mean()) if aligned else None,
        "mean_abs_lateral_revision_m":float(np.abs(local[:,1]).mean()) if aligned else None,
        "max_abs_lateral_revision_m":float(np.abs(local[:,1]).max()) if aligned else None,
        "common_horizon_endpoint_displacement_m":float(norm[-1]) if aligned else None,
        "full_endpoint_displacement_m":float(np.linalg.norm(fw[-1]-ow[-1])),
        "endpoint_note":"Primary common-horizon endpoint uses same nominal absolute time; full endpoints have different times",
        "lookahead_displacement_m":float(np.linalg.norm(fw[fresh["lookahead_index"]]-ow[old["lookahead_index"]])),
        "lookahead_note":"Controller-selected world targets; not same-time correspondence",
        "mean_abs_tangent_revision_deg":aligned["tangent_mean_deg"] if aligned else None,
        "max_abs_tangent_revision_deg":aligned["tangent_max_deg"] if aligned else None,
        "delta_v":fc["target_v"]-oc["target_v"],"delta_w":fc["target_w"]-oc["target_w"],
        "latency":latency_metrics(fresh),"observation_to_switch_sim_s":fresh["application"]["sim_time"]-ft,
        "raw_seam":seam,"qualification_decision":decision,
        "classification_note":"Numerical tolerances only; conflict resolution supplies task meaning, no arbitrary seam severity threshold. Descriptive comparison, no identical-state causal claim."}
    details={"old_remaining_times":rt.tolist() if rt is not None else None,"old_remaining_points":rp.tolist() if rp is not None else None,
        "alignment":{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in aligned.items()} if aligned else None,
        "fresh_frame_delta":local.tolist() if local is not None else None}
    if human and rp is not None:
        for name,times,world in [("old",rt,rp),("fresh",ft+dt*np.arange(1,len(fw)+1),fw)]:
            dynamic=temporal_human_clearance(times,world,run["human_times"],run["human_xy"],radius)
            result[name+"_time_varying_human_clearance_m"]=dynamic["minimum_clearance_m"] if dynamic else None
            details[name+"_dynamic_clearance"]=dynamic
            details[name+"_fixed_human_clearance"]={"absolute_times":times.tolist(),"clearance_m":(np.linalg.norm(world-np.array(human["position"][:2]),axis=1)-radius).tolist()}
    return result,details


def pretrigger_comparison(dynamic,baseline):
    rows=[]
    for i in range(min(15,len(dynamic["events"]))):
        de,be=dynamic["events"][i],baseline["events"][i]
        dpose,bpose=np.array(de["agent_pose_at_observation"]),np.array(be["agent_pose_at_observation"])
        dc,bc=dynamic["chunks"][i],baseline["chunks"][i]
        # Separate pose divergence and forward/left output shape in each own
        # frame. The latter is explicitly not a frame-invariant revision metric.
        world_difference=dynamic["worlds"][i]-baseline["worlds"][i]
        v=dynamic["path"]/"raw/scene_observations"/("human_"+Path(de["rgb_observation_reference"]).name+".json")
        visibility=json.loads(v.read_text()) if v.exists() else {}
        rows.append({"request_id":i+1,"observation_time_difference_s":de["observation"]["sim_time"]-be["observation"]["sim_time"],
            "observation_xy_difference_m":float(np.linalg.norm(dpose[:2]-bpose[:2])),
            "observation_yaw_difference_deg":float(np.degrees(wrap(dpose[2]-bpose[2]))),
            "world_same_relative_horizon_rmse_m":float(np.sqrt(np.mean(np.sum(world_difference**2,axis=1)))),
            "own_local_shape_rmse_m_diagnostic":float(np.sqrt(np.mean(np.sum((dc-bc)**2,axis=1)))),
            "delta_target_w":de["controller_command"][1]-be["controller_command"][1],
            "human_frustum_proxy":visibility.get("in_pinhole_frustum_proxy"),
            "human_center_los_clear":visibility.get("line_of_sight_clear_to_center"),
            "note":"Separate closed loops, same relative nominal horizon; local shape diagnostic INVALID AS FRAME-INVARIANT REVISION METRIC; no identical-state causal inference"})
    return rows


def analyze(run_dir,baseline_dir,receipt_path,output_dir):
    source,base,out=map(lambda p:Path(p).resolve(),[run_dir,baseline_dir,output_dir])
    if out.exists():raise FileExistsError(out)
    if any(p in out.parents or out in p.parents for p in [source,base]):raise ValueError("Separate output required")
    receipt=json.loads(Path(receipt_path).read_text())
    hashes={str(p):digest(p) for folder in [source,base] for p in folder.rglob("*") if p.is_file()}
    hashes[str(Path(receipt_path).resolve())]=digest(receipt_path)
    dynamic,baseline=load_saved(source,True),load_saved(base)
    validate_frozen_config(dynamic["cfg"],baseline["cfg"])
    if dynamic["metadata"]["freeze_receipt_sha256"]!=digest(receipt_path):raise ValueError("Wrong freeze receipt")
    settings=dynamic["cfg"]["dynamic_analysis"]
    results={}
    for name,run in [("dynamic",dynamic),("baseline",baseline)]:
        rows=request_metrics(run,settings);pair,detail=pair_metrics(run,rows,settings)
        results[name]={"request_metrics":rows,"primary_pair":pair,"details":detail}
    consistency=pretrigger_comparison(dynamic,baseline)
    primary=results["dynamic"]["primary_pair"]
    summary={"primary_pair":[15,16],"qualification_decision":primary["qualification_decision"],
        "dynamic_run":str(source),"baseline_run":str(base),"dynamic_complete":dynamic["complete"],
        "prediction_count":len(dynamic["events"]),"trigger":dynamic["trigger"],"dynamic":primary,
        "baseline":results["baseline"]["primary_pair"],"pretrigger_comparison":consistency,
        "limitations":["One scripted fixed-speed human, frozen rest pose, no crowd policy","Circumscribed proxy, not exact yaw-dependent collision oracle",
            "Nominal waypoint timing, not native timestamp channel","No spatial correspondence, correction or compensation",
            "Different closed-loop states and stochastic reasoning; descriptive baseline comparison",
            "Static validity uses measured contact/RGB context; no complete predicted-path Hospital collision oracle",
            "Camera brightness/proximity/frustum diagnostics are not semantic visibility guarantees"]}
    out.mkdir(parents=True,exist_ok=False)
    write_json(out/"metadata.json",{"source_files_sha256":hashes,**provenance(ROOT),
        "all_research_code_sha256":{str(p.relative_to(ROOT)):digest(p) for p in (ROOT/"research").rglob("*.py")}})
    write_json(out/"primary_pair.json",primary);write_json(out/"baseline_primary_pair.json",results["baseline"]["primary_pair"])
    write_json(out/"summary.json",summary);write_json(out/"details.json",results)
    secondary=[]
    for old,fresh in [(14,15),(16,17)]:
        m,_=pair_metrics(dynamic,results["dynamic"]["request_metrics"],settings,old,fresh);secondary.append(m)
    write_json(out/"secondary_pairs.json",secondary)
    for name,rows in [("request_metrics",results["dynamic"]["request_metrics"]),("baseline_request_metrics",results["baseline"]["request_metrics"]),("pretrigger_comparison",consistency)]:
        with (out/(name+".csv")).open("x") as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        write_json(out/(name+".json"),rows)
    from research.nova_dynamic_plots import plot_all
    plot_all(out,dynamic,baseline,results)
    if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError("Source changed during analysis")
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ["run-dir","baseline-run-dir","freeze-receipt","output-dir"]:p.add_argument("--"+name,required=True)
    a=p.parse_args();s=analyze(a.run_dir,a.baseline_run_dir,a.freeze_receipt,a.output_dir)
    print(s["qualification_decision"]);print(json.dumps({k:v for k,v in s["dynamic"].items() if k not in ["old_context","fresh_context","trigger_event"]},indent=2))


if __name__=="__main__":main()
