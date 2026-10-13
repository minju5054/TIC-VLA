"""Model-free discovery of the original Hospital wheelchair asset."""
import sys,json,hashlib,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RealTimePathTracing','width':640,'height':480});code=1
try:
 import omni.usd,omni.client
 from pxr import Usd,UsdGeom,UsdPhysics
 import numpy as np
 root=Path(__file__).resolve().parents[2]
 cfg=json.loads((root/'outputs/nova-e16-hospital-lights-baseline-20261010-01/metadata.json').read_text())['config']
 stage=omni.usd.get_context().get_stage();stage.GetRootLayer().subLayerPaths.append(cfg['scene']['usd'])
 cache=UsdGeom.BBoxCache(0,['default','render','proxy']);rows=[]
 for prim in stage.GetPrimAtPath('/Root').GetChildren():
  if 'wheel' not in prim.GetName().lower():continue
  b=cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
  refs=[str(x) for x in prim.GetPrimStack()]
  rows.append({'prim':str(prim.GetPath()),'local_bounds':[list(b.GetMin()),list(b.GetMax())], 'matrix':np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)).tolist(),'references':str(prim.GetMetadata('references')),'layers':[x.layer.identifier for x in prim.GetPrimStack()], 'colliders':[str(x.GetPath()) for x in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()) if x.HasAPI(UsdPhysics.CollisionAPI)],'rigid_bodies':[str(x.GetPath()) for x in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()) if x.HasAPI(UsdPhysics.RigidBodyAPI)]})
 (a.output/'asset.json').write_text(json.dumps({'scene':cfg['scene']['usd'],'candidates':rows,'model_calls':0},indent=2));print('WHEELCHAIR_ASSETS',json.dumps(rows),flush=True);code=0
finally:app.close(exit_code=code)
