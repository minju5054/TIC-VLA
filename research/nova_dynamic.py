"""One gated Nova/Hospital dynamic-human experiment; fixed C15 acceptance trigger."""
import argparse
import inspect
import json
import os
from pathlib import Path
import sys
import traceback
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance,clocks
from research.hospital_episode import digest,validate_config
from research.nova_dynamic_geometry import validate_frozen_config
from research.nova_evidence import NovaRecords,validate_continuous


def verify_receipt(cfg,config_path,receipt,run_id):
    base=Path(cfg["dynamic_analysis"]["baseline_run"])
    validate_frozen_config(cfg,json.loads((base/"metadata.json").read_text())["config"])
    if (receipt["authorized_run_id"]!=run_id or receipt["config_sha256"]!=digest(config_path)
            or not receipt["preflight_pass"] or not receipt["visual_review_pass"]):
        raise ValueError("Pre-model freeze/authorization mismatch")
    for key in ["code_sha256","source_sha256"]:
        for p,h in receipt[key].items():
            if digest(ROOT/p)!=h:raise ValueError("Frozen file changed: "+p)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config",default="configs/research/nova_e16_dynamic_human.yaml")
    p.add_argument("--freeze-receipt",required=True);p.add_argument("--run-id",required=True)
    a=p.parse_args()
    if Path(a.run_id).name!=a.run_id:p.error("One fresh run ID required")
    cfg=yaml.safe_load(Path(a.config).read_text());validate_config(cfg)
    receipt=json.loads(Path(a.freeze_receipt).read_text());verify_receipt(cfg,a.config,receipt,a.run_id)
    records=NovaRecords(ROOT/"outputs"/a.run_id,{"mode":"dynamic","config":cfg,"command_variant":"direct",
        "freeze_receipt":receipt,"freeze_receipt_sha256":digest(a.freeze_receipt),"config_sha256":digest(a.config),
        "command_argv":sys.argv,"all_research_code_sha256":{str(p.relative_to(ROOT)):digest(p) for p in (ROOT/"research").rglob("*.py")},
        **provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":True,"renderer":"RayTracedLighting","width":1920,"height":1080})
    sim=None;code=1
    try:
        from research.nova_simulation import NovaSimulation
        from research.human_actor import TriggeredHuman
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        from pxr import Gf,UsdGeom
        import carb
        from omni.physx import get_physx_scene_query_interface

        class HumanSimulation(NovaSimulation):
            def capture(self,name):
                data,obs,path=super().capture(name)
                cam=UsdGeom.Camera(self.stage.GetPrimAtPath(cfg["camera"]["prim"]))
                matrix=UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(0)
                human=self.pedestrian.snapshot();target=np.array(human["position"])+[0,0,.9]
                eye=np.array(matrix.ExtractTranslation());delta=target-eye;distance=np.linalg.norm(delta)
                hit=get_physx_scene_query_interface().raycast_closest(carb.Float3(*map(float,eye)),carb.Float3(*map(float,delta/distance)),float(distance))
                camera_local=np.array(matrix.GetInverse().Transform(Gf.Vec3d(*target)))
                depth=-camera_local[2];f=float(cam.GetFocalLengthAttr().Get())
                aperture=np.array([cam.GetHorizontalApertureAttr().Get(),cam.GetVerticalApertureAttr().Get()])
                in_proxy=bool(depth>0 and np.all(np.abs(camera_local[:2]/depth)<=aperture/(2*f)))
                write_json(records.path/"raw/scene_observations"/("human_"+name+".json"),{
                    "human":human,"camera_world_xyz":eye.tolist(),"human_camera_local":camera_local.tolist(),
                    "in_pinhole_frustum_proxy":in_proxy,"first_ray_hit":str(hit.get("collision","")),
                    "line_of_sight_clear_to_center":bool(not hit["hit"] or str(hit.get("collision","")).startswith(cfg["dynamic_human"]["prim"])),
                    "note":"Geometric visibility context only; original fisheye camera unchanged; not a pixel visibility oracle"})
                return data,obs,path

        sim=HumanSimulation(cfg,records,actor_factory=lambda s:TriggeredHuman(s,cfg["dynamic_human"]))
        control_path=Path(inspect.getfile(DifferentialController))
        write_json(records.path/"simulation-runtime.json",{"python":sys.version,"numpy":np.__version__,
            "isaac_version":(Path(os.environ["ISAAC_PATH"])/"VERSION").read_text().strip(),
            "differential_controller_source":str(control_path),"controller_sha256":digest(control_path)})
        sim.phase="dynamic_closed_loop"
        result=run_model_loop(sim,cfg,records,"dynamic",evidence_analyzer=validate_continuous)
        write_json(records.path/"summary.json",{"status":"PASS","mode":"dynamic",**result})
        print("NOVA_DYNAMIC_RUN_COMPLETE",json.dumps(result),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(records.path/"failure.json",{"status":"FAIL","error":repr(exc),**clocks()})
    finally:
        if sim:sim.close()
        app.close(exit_code=code)


if __name__=="__main__":main()
