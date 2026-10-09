"""Model-free Hospital geometry audit and baseline-derived human placement."""
import argparse
import json
import math
from pathlib import Path
import sys
import traceback
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.records import RunRecords,write_json,provenance
from research.hospital_episode import digest
from research.nova_dynamic_geometry import baseline_design


def footprint(stage,cfg):
    from pxr import Usd,UsdGeom,UsdPhysics
    body=UsdGeom.Xformable(stage.GetPrimAtPath(cfg["robot"]["body"])).ComputeLocalToWorldTransform(0)
    shapes=[]
    for p in Usd.PrimRange(stage.GetPrimAtPath(cfg["robot"]["prim"]),Usd.TraverseInstanceProxies()):
        kind=p.GetTypeName();pts=None
        if kind not in ["Mesh","Cylinder","Capsule","Sphere","Cube"]:continue
        if kind=="Mesh":
            pts=np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(),float)
        elif kind in ["Cylinder","Capsule","Sphere","Cube"]:
            if kind=="Cube":half=np.repeat(float(UsdGeom.Cube(p).GetSizeAttr().Get())/2,3)
            elif kind=="Sphere":half=np.repeat(float(UsdGeom.Sphere(p).GetRadiusAttr().Get()),3)
            else:
                obj=UsdGeom.Cylinder(p) if kind=="Cylinder" else UsdGeom.Capsule(p)
                radius=float(obj.GetRadiusAttr().Get());half=np.repeat(radius,3)
                half["XYZ".index(str(obj.GetAxisAttr().Get()))]=float(obj.GetHeightAttr().Get())/2+(radius if kind=="Capsule" else 0)
            pts=np.array([[x,y,z] for x in [-half[0],half[0]] for y in [-half[1],half[1]] for z in [-half[2],half[2]]])
        if pts is None or not pts.size:raise ValueError("Unaudited collision shape: "+str(p.GetPath()))
        m=np.asarray(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)*body.GetInverse())
        points=pts@m[:3,:3]+m[3,:3]
        shapes.append({"path":str(p.GetPath()),"type":kind,"collision_api":p.HasAPI(UsdPhysics.CollisionAPI),"xy_min":points[:,:2].min(0).tolist(),
            "xy_max":points[:,:2].max(0).tolist(),"radius_m":float(np.linalg.norm(points[:,:2],axis=1).max())})
    if not shapes:raise ValueError("No Nova collision geometry")
    return {"conservative_radius_m":max(s["radius_m"] for s in shapes),"shapes":shapes,
        "note":"Body-origin circumradius of all loaded mesh vertices / analytic primitive bounding corners (visual and collision geometry). Includes descendants of CollisionAPI Xform containers; conservative yaw-independent 2D proxy, not exact collision oracle"}


def audit_path(sim,human):
    import carb
    from omni.physx import get_physx_scene_query_interface
    query=get_physx_scene_query_interface();start=np.array(human["start_position"]);end=np.array(human["end_position"])
    count=int(np.ceil(np.linalg.norm(end-start)/.05))+1;checks=[]
    for point in np.linspace(start,end,count):
        floor=query.raycast_closest(carb.Float3(float(point[0]),float(point[1]),.3),carb.Float3(0,0,-1),.6)
        hits=[]
        for z in [.30,.65,1.,1.35,1.60]:
            def callback(hit):
                path=str(hit.collision)
                if not path.startswith(sim.cfg["robot"]["prim"]):hits.append(path)
                return True
            query.overlap_sphere(.25,carb.Float3(float(point[0]),float(point[1]),z),callback,False)
        floor_ok=bool(floor["hit"] and abs(float(floor["position"][2]))<.02 and abs(float(floor["normal"][2]))>.9)
        checks.append({"xy":point[:2].tolist(),"floor_hit":floor_ok,"floor_path":str(floor.get("collision","")),
                       "nonfloor_overlap_paths":sorted(set(hits))})
    return {"valid":all(c["floor_hit"] and not c["nonfloor_overlap_paths"] for c in checks),"samples":checks,
        "sampling_note":"<=5cm along path; radius .25m spheres at 5 heights plus downward floor rays. Conservative clearance audit, not navmesh/crowd-policy validation."}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--baseline-run",required=True);p.add_argument("--output-dir",required=True)
    p.add_argument("--config-output",required=True)
    a=p.parse_args();base=Path(a.baseline_run).resolve();out=Path(a.output_dir).resolve();config_out=Path(a.config_output)
    if config_out.exists():raise FileExistsError(config_out)
    meta=json.loads((base/"metadata.json").read_text());cfg=meta["config"]
    records=RunRecords(out,{"mode":"model_free_human_placement","baseline_run":str(base),"config":cfg,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":True,"renderer":"RayTracedLighting","width":1920,"height":1080})
    sim=None;code=1
    try:
        from research.nova_simulation import NovaSimulation
        from research.human_actor import TriggeredHuman
        sim=NovaSimulation(cfg,records)
        geometry=footprint(sim.stage,cfg);write_json(out/"nova_footprint.json",geometry)
        design=baseline_design(base,geometry["conservative_radius_m"])
        for candidate in design["candidates"]:candidate["hospital_audit"]=audit_path(sim,candidate["human"])
        write_json(out/"intervention_design.json",design)
        valid=[c for c in design["candidates"] if c["hospital_audit"]["valid"] and c["staged_old_clearance_m"]>0 and c["expected_fresh_observation_old_clearance_m"]<0]
        if not valid:raise RuntimeError("No valid baseline-derived staging/crossing candidate; inspect pre-model geometry")
        chosen=valid[0]
        human={**chosen["human"],"prim":"/World/DynamicHuman","asset":"https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/male_adult_construction_03/male_adult_construction_03.usd",
            "radius_m":.25,"capsule_cylinder_height_m":1.2,
            "source_note":"Official Isaac People character family; source IRA default /Isaac/People/Characters/, ~1m/s walking comment; scripted fixed speed, no crowd-policy reproduction"}
        newcfg={**cfg,"dynamic_human":human,"dynamic_analysis":{"primary_pair":[15,16],"native_waypoint_dt":.1,
            "robot_radius_m":geometry["conservative_radius_m"],"human_radius_m":.25,"executed_tangent_window_s":.1,
            "minimum_tangent_displacement_m":1e-6,"numerical_position_tolerance_m":1e-6,
            "baseline_run":str(base),"preflight_dir":str(out),
            "validity_rule":"Finite usable nondegenerate native path; p95>20; no camera/static-contact artifact; OLD initially outside human proxy. Conservative circumscribed proxy, no exact static path oracle.",
            "qualification_rule":"Strong only if valid OLD becomes conflicting, valid FRESH avoids proxy, measured OLD transport and RAW seam exceed numerical tolerance. Otherwise descriptive revision with incomplete qualification, no severity threshold.",
            "primary_fixed_before_model":True}}
        sim.pedestrian=TriggeredHuman(sim,human);sim.apply(0,0);sim.step()
        initial=sim.pedestrian.snapshot()
        # Diagnostic camera at the saved baseline observation, not a new
        # navigation run. This preflight robot pose is explicitly reconstructed.
        e=json.loads((base/"raw/requests/request_000015.json").read_text())
        original=sim.state()
        sim.robot.set_world_pose(np.array(e["observation"]["position"]),np.array(e["observation"]["quaternion_wxyz"]))
        sim.capture("baseline_C15_pose_staged_human.png")
        sim.robot.set_world_pose(np.array(original["position"]),np.array(original["quaternion_wxyz"]))
        sim.pedestrian.on_request_accepted(15,sim.state())
        for _ in range(round(3/sim.dt)):sim.step()
        final=sim.pedestrian.snapshot()
        sim.pedestrian.file.flush();sim.state_file.flush()
        error=float(np.linalg.norm(np.array(final["position"])-human["end_position"]))
        if error>1e-4:raise RuntimeError("Measured human endpoint does not follow scripted path")
        with config_out.open("x") as f:yaml.safe_dump(newcfg,f,sort_keys=False)
        write_json(out/"summary.json",{"status":"PASS","model_calls":0,"selected_candidate":chosen["side_sign"],
            "human":human,"measured_initial":initial,"measured_final":final,"endpoint_error_m":error,
            "robot_radius_m":geometry["conservative_radius_m"],"config_path":str(config_out),"config_sha256":digest(config_out),
            "diagnostic_note":"Only preflight snapshot robot is placed at saved C15 pose; no policy/navigation rerun. Human moves continuously in physics at1m/s."})
        print("NOVA_DYNAMIC_PREFLIGHT_PASS",json.dumps(human),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(out/"failure.json",{"error":repr(exc),"model_calls":0})
    finally:
        if sim:sim.close()
        app.close(exit_code=code)


if __name__=="__main__":main()
