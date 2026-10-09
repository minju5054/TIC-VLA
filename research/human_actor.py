"""Official character visual plus a configured kinematic capsule, no crowd policy."""
import csv
import math
import numpy as np
from research.nova_dynamic_geometry import human_position, AcceptanceTrigger
from research.records import write_json, clocks


def author_human(stage, config, asset_info=None, physical=True):
    """Shared exact source visual transform for experiment and stopped GUI replay."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    root_path=config["prim"]
    if asset_info is None:
        source=Usd.Stage.Open(config["asset"])
        if not source: raise RuntimeError("Official human asset failed to load")
        unit=UsdGeom.GetStageMetersPerUnit(source);up=str(UsdGeom.GetStageUpAxis(source))
        if up not in ["Y","Z"]: raise ValueError("Unsupported human axis")
        box=UsdGeom.BBoxCache(0,["default","render","proxy"]).ComputeWorldBound(source.GetDefaultPrim()).ComputeAlignedRange()
        low,high=np.array(box.GetMin()),np.array(box.GetMax())
        rotation=np.eye(3) if up=="Z" else np.array([[1,0,0],[0,0,-1],[0,1,0]])
        corners=np.array([[x,y,z] for x in [low[0],high[0]] for y in [low[1],high[1]] for z in [low[2],high[2]]])
        pts=(corners@rotation.T)*unit;lo,hi=pts.min(0),pts.max(0);height=float(hi[2]-lo[2])
        if not 1.3<height<2.5: raise ValueError("Unexpected human asset height")
        asset_info={"asset":config["asset"],"source_units":unit,"source_up_axis":up,"height_m":height,
            "visual_translation_m":[float(-(lo[0]+hi[0])/2),float(-(lo[1]+hi[1])/2),float(-lo[2])],
            "source_bounds":[low.tolist(),high.tolist()],"source_to_z_up_rotation":rotation.tolist(),
            "motion_note":"Scripted fixed-speed translated character; frozen authored rest pose, no gait/crowd-policy reproduction"}
    root=UsdGeom.Xform.Define(stage,root_path);root.ClearXformOpOrder()
    translate=root.AddTranslateOp();translate.Set(Gf.Vec3d(*config["start_position"]))
    root.AddRotateZOp().Set(math.degrees(config["yaw"]))
    if physical: UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateKinematicEnabledAttr(True)
    visual=UsdGeom.Xform.Define(stage,root_path+"/VisualTransform")
    visual.AddTranslateOp().Set(Gf.Vec3d(*asset_info["visual_translation_m"]))
    if asset_info["source_up_axis"]=="Y":visual.AddRotateXOp().Set(90)
    unit=asset_info["source_units"];visual.AddScaleOp().Set(Gf.Vec3f(unit,unit,unit))
    stage.DefinePrim(root_path+"/VisualTransform/Model","Xform").GetReferences().AddReference(config["asset"])
    if physical:
        capsule=UsdGeom.Capsule.Define(stage,root_path+"/Collider")
        capsule.CreateAxisAttr("Z");capsule.CreateRadiusAttr(config["radius_m"])
        capsule.CreateHeightAttr(config["capsule_cylinder_height_m"])
        capsule.AddTranslateOp().Set(Gf.Vec3d(0,0,asset_info["height_m"]/2))
        capsule.MakeInvisible();UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    return translate,asset_info


class TriggeredHuman:
    def __init__(self, sim, config):
        self.sim,self.config=sim,config
        self.trigger=AcceptanceTrigger(config["trigger_request_id"])
        self.translate,self.asset=author_human(sim.stage,config)
        write_json(sim.records.path/"human_asset.json",self.asset)
        self.file=(sim.records.path/"raw/human_state.csv").open("x")
        self.writer=csv.DictWriter(self.file,fieldnames=["tick","sim_time","x","y","z","yaw","target_x","target_y","target_z","motion_active","trigger_request_id","trigger_sim_time"])
        self.writer.writeheader();self.current=np.array(config["start_position"],float)

    def on_request_accepted(self, request_id, application):
        if self.trigger.accept(request_id,application["sim_time"]):
            write_json(self.sim.records.path/"raw/human_trigger.json",{
                "trigger_request_id":request_id,"trigger_sim_time":self.trigger.time,"application":application,
                "event_source":"main_thread_after_C15_acceptance_before_next_physics_step",
                "human_at_trigger":self.snapshot(),"depends_on_prediction_content":False,**clocks()})

    def update(self,sim_time):
        from pxr import Gf
        self.current=human_position(self.config,sim_time,self.trigger.time)
        self.translate.Set(Gf.Vec3d(*self.current))

    def snapshot(self):
        import omni.physx
        pose=omni.physx.get_physx_interface().get_rigidbody_transformation(self.config["prim"])
        if not pose["ret_val"]:raise RuntimeError("Measured PhysX human pose unavailable")
        t=self.sim.state()["sim_time"]
        active=self.trigger.time is not None and self.trigger.time<=t<self.trigger.time+np.linalg.norm(np.array(self.config["end_position"])-self.config["start_position"])/self.config["speed_m_s"]
        return {"position":list(pose["position"]),"yaw":self.config["yaw"],"sim_time":t,
            "motion_active":bool(active),"trigger_request_id":self.config["trigger_request_id"],
            "trigger_sim_time":self.trigger.time,"event_source":"main_thread_measured_PhysX",**clocks()}

    def measure(self,sim_time):
        s=self.snapshot()
        self.writer.writerow(dict(zip(self.writer.fieldnames,[self.sim.tick,sim_time,*s["position"],s["yaw"],*self.current,
            s["motion_active"],s["trigger_request_id"],self.trigger.time])))

    def close(self):self.file.close()
