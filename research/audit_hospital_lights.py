"""Load actual Hospital USD and audit authored lights/fixtures without inference."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.hospital_episode import digest


def serial(value):
    if value is None or isinstance(value,(str,bool,int,float)):return value
    try:return [serial(x) for x in value]
    except TypeError:return str(value)


def inspect_stage(stage):
    from pxr import Usd,UsdGeom,UsdLux,UsdShade,UsdPhysics
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],useExtentsHint=False)
    lights=[];fixtures=[];materials={};keywords=[];collisions=[]
    words=['light','lamp','ceiling','fixture','fluorescent','emissive','hospital light']
    prims=list(Usd.PrimRange(stage.GetPrimAtPath('/Root'),Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)))
    def location(p):
        x=UsdGeom.Xformable(p)
        matrix=np.array(x.ComputeLocalToWorldTransform(0)) if x else np.eye(4)
        box=cache.ComputeWorldBound(p).ComputeAlignedRange() if p.IsA(UsdGeom.Imageable) else None
        return {'world_matrix_row_vector':matrix.tolist(),'world_xyz':matrix[3,:3].tolist(),
            'world_aabb_min':serial(box.GetMin()) if box and not box.IsEmpty() else None,
            'world_aabb_max':serial(box.GetMax()) if box and not box.IsEmpty() else None}
    for p in prims:
        path=str(p.GetPath());matched=[w for w in words if w in path.lower()]
        if matched:keywords.append({'path':path,'type':p.GetTypeName(),'keywords':matched})
        if p.HasAPI(UsdLux.LightAPI) or p.GetTypeName().endswith('Light'):
            img=UsdGeom.Imageable(p)
            lights.append({'path':path,'type':p.GetTypeName(),'has_light_api':p.HasAPI(UsdLux.LightAPI),
                'active':p.IsActive(),'loaded':p.IsLoaded(),'instance_proxy':p.IsInstanceProxy(),
                'visibility':str(img.ComputeVisibility()) if img else None,
                'parent_fixture_path':str(p.GetParent().GetPath()),
                'attributes':{a.GetName():serial(a.Get()) for a in p.GetAttributes()},**location(p)})
        if p.IsA(UsdGeom.Mesh):
            material=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            material_path=str(material.GetPath()) if material else None
            emission=[]
            if material:
                for shader in Usd.PrimRange(material.GetPrim()):
                    for a in shader.GetAttributes():
                        if any(k in a.GetName().lower() for k in ['emiss','emission','glow']):
                            emission.append({'attribute':str(a.GetPath()),'value':serial(a.Get()),'connections':[str(x) for x in a.GetConnections()]})
                if emission:materials[material_path]=emission
            if matched or emission:
                fixtures.append({'mesh':path,'matched_keywords':matched,'material':material_path,'emission':emission,**location(p)})
        if p.HasAPI(UsdPhysics.CollisionAPI):
            collisions.append({'path':path,'type':p.GetTypeName(),'enabled':UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get(),**location(p)})
    return lights,fixtures,materials,keywords,collisions


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-run',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
    source=Path(a.source_run).resolve();out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((source/'metadata.json').read_text())['config']
    if 'bright_profile' in cfg:raise ValueError('Original validated exposure/source run required')
    events=[json.loads(p.read_text()) for p in sorted((source/'raw/requests').glob('*.json'))]
    hashes={str(p):digest(p) for p in [source/'metadata.json',*sorted((source/'raw/requests').glob('*.json'))]}
    write_json(out/'metadata.json',{'model_calls':0,'physics_reexecuted':False,'source_hashes':hashes,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        import carb,omni.client
        from PIL import Image
        from pxr import UsdGeom
        from research.nova_render_saved import SavedRenderer
        from research.nova_bright import image_stats
        render=SavedRenderer(app,cfg);lights,fixtures,materials,keywords,collisions=inspect_stage(render.stage)
        write_json(out/'lights.json',lights);write_json(out/'fixtures.json',fixtures)
        write_json(out/'emissive_materials.json',materials);write_json(out/'keyword_paths.json',keywords);write_json(out/'collision_geometry.json',collisions)
        settings=carb.settings.get_settings();keys=['/rtx/post/tonemap/'+k for k in ['op','filmIso','exposureTime','fNumber','responsivity','enableSrgbToGamma','whitepoint','colorMode','dither']]+['/rtx/post/histogram/enabled','/rtx/post/motionblur/enabled']
        defaults={k:serial(settings.get(k)) for k in keys};write_json(out/'renderer_defaults.json',defaults)
        result,version,content=omni.client.read_file(cfg['scene']['usd'])
        if result!=omni.client.Result.OK:raise RuntimeError('Cannot hash official source bytes: '+str(result))
        scene={'url':cfg['scene']['usd'],'source_byte_sha256':hashlib.sha256(bytes(content)).hexdigest(),'source_byte_count':len(bytes(content)),
            'meters_per_unit':UsdGeom.GetStageMetersPerUnit(render.stage),'up_axis':str(UsdGeom.GetStageUpAxis(render.stage)),
            'used_layers':[{'identifier':layer.identifier,'canonical_text_sha256':hashlib.sha256(layer.ExportToString().encode()).hexdigest()} for layer in render.stage.GetUsedLayers() if not layer.anonymous],
            'collision_prim_count':len(collisions),'collision_inventory_sha256':digest(out/'collision_geometry.json'),'official_source_modified':False}
        write_json(out/'hospital_scene.json',scene)
        folder=out/'selected_route_pose_renders';folder.mkdir();stats=[]
        for rid in [1,8,12,15,16,17,18,20,24,32,48]:
            camera=render.pose(events[rid-1]);rgb=render.capture();Image.fromarray(rgb).save(folder/f'rgb_{rid:02d}.png')
            if rid==1:Image.fromarray(rgb).save(out/'original_start_rgb.png')
            stats.append({'request_id':rid,'camera_world_matrix_row_vector':camera,**image_stats(rgb)})
        write_json(out/'original_brightness.json',stats)
        if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Saved source changed')
        summary={'status':'AUDIT_COMPLETE','model_calls':0,'physics_reexecuted':False,'authored_light_count':len(lights),
            'light_types':{k:sum(l['type']==k for l in lights) for k in sorted(set(l['type'] for l in lights))},
            'fixture_mesh_record_count':len(fixtures),'emission_material_count':len(materials),'keyword_path_count':len(keywords)}
        write_json(out/'summary.json',summary);print('HOSPITAL_LIGHT_AUDIT',json.dumps(summary),flush=True);code=0
    except Exception as exc:traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc),'model_calls':0})
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
