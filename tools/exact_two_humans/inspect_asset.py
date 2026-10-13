"""Model-free inspection of actual character mesh/skinning and authored bounds."""
import argparse, json, sys, traceback
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True);args=p.parse_args()
out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RealTimePathTracing','width':640,'height':480,'disable_viewport_updates':True});code=1
try:
 from pxr import Usd,UsdGeom,UsdSkel,Vt,Gf
 from research.human_actor import author_human
 import omni.usd
 cfg=json.loads((ROOT/'configs/research/ticvla_2hz_stationary_human_selected.json').read_text())['human']
 cfg.update(prim='/World/ProbeHuman',start_position=[0.,0.,0.])
 stage=omni.usd.get_context().get_stage();_,info=author_human(stage,cfg,physical=False)
 for _ in range(5):app.update()
 cache=UsdSkel.Cache()
 for prim in stage.Traverse():
  if prim.IsA(UsdSkel.Root):cache.Populate(UsdSkel.Root(prim),Usd.PrimDefaultPredicate)
 rows=[]
 for prim in stage.Traverse():
  if not prim.IsA(UsdGeom.Mesh):continue
  mesh=UsdGeom.Mesh(prim);points=mesh.GetPointsAttr().Get(Usd.TimeCode.Default());mat=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
  arr=np.asarray(points);world=np.column_stack([arr,np.ones(len(arr))])@np.asarray(mat)
  query=cache.GetSkinningQuery(prim);skin=None
  if query:
   binding=UsdSkel.BindingAPI(prim);skeleton=binding.GetInheritedSkeleton();skq=cache.GetSkelQuery(skeleton)
   transforms=skq.ComputeSkinningTransforms(Usd.TimeCode.Default())
   skinned=Vt.Vec3fArray(points)
   try:
    ok=query.ComputeSkinnedPoints(transforms,skinned,Usd.TimeCode.Default())
    smat=UsdGeom.Xformable(skeleton).ComputeLocalToWorldTransform(Usd.TimeCode.Default());sw=np.column_stack([np.asarray(skinned),np.ones(len(skinned))])@np.asarray(smat)
    skin={'success':ok,'skeleton':str(skeleton.GetPath()),'point_min_skelworld':sw[:,:3].min(0).tolist(),'point_max_skelworld':sw[:,:3].max(0).tolist(),'geom_bind_transform':np.asarray(query.GetGeomBindTransform()).tolist(),'skinned_min':np.asarray(skinned).min(0).tolist(),'skel_world':np.asarray(smat).tolist()}
   except Exception as e:skin={'error':repr(e),'skinning_methods':dir(query)}
  rows.append({'path':str(prim.GetPath()),'visibility':str(mesh.ComputeVisibility()),'points':len(points),'point_min_world':world[:,:3].min(0).tolist(),'point_max_world':world[:,:3].max(0).tolist(),'authored_extent':str(mesh.GetExtentAttr().Get()),'skinning':skin})
 result={'asset_info':info,'meshes':rows,'model_calls':0,'navigation_steps':0}
 (out/'asset-inspection.json').write_text(json.dumps(result,indent=2,allow_nan=False))
 print('ASSET_INSPECTION',json.dumps(result),flush=True);code=0
except Exception:
 traceback.print_exc()
finally:app.close(exit_code=code)
