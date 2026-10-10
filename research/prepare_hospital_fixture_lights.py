"""Preregister ceiling illumination from actual scene fixture bounds/transforms."""
import argparse,hashlib,json,sys,traceback
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.hospital_lights import native_profile,fixture_sources
from research.audit_hospital_lights import serial


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['audit-dir','source-run','native-calibration','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args();audit=Path(a.audit_dir);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=False)
    result=json.loads((Path(a.native_calibration)/'summary.json').read_text())
    if result['selected_candidate'] is not None:raise ValueError('Native passing profile takes priority')
    fixtures=fixture_sources(json.loads((audit/'fixtures.json').read_text()));lights=json.loads((audit/'lights.json').read_text());renderer=json.loads((audit/'renderer_defaults.json').read_text())
    cfg=json.loads((Path(a.source_run)/'metadata.json').read_text())['config']
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':640,'height':480});code=1
    try:
        from research.nova_render_saved import SavedRenderer
        from pxr import Usd,UsdShade
        import omni.client
        render=SavedRenderer(app,cfg);shader_records=[];source_records={};spec=[]
        for i,f in enumerate(fixtures):
            p=render.stage.GetPrimAtPath(f['mesh']);material=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            shader_records.append({'fixture':f['mesh'],'material':str(material.GetPath()),'shader_attributes':[
                {'path':str(a.GetPath()),'value':serial(a.Get())} for s in Usd.PrimRange(material.GetPrim()) for a in s.GetAttributes()]})
            for s in Usd.PrimRange(material.GetPrim()):
                for attr in s.GetAttributes():
                    value=attr.Get()
                    if attr.GetName().endswith('sourceAsset') and value:
                        url=getattr(value,'resolvedPath','')
                        if url and url not in source_records:
                            status,_,content=omni.client.read_file(url)
                            if status==omni.client.Result.OK:
                                raw=bytes(content);text=raw.decode(errors='replace');lines=text.splitlines()
                                source_records[url]={'sha256':hashlib.sha256(raw).hexdigest(),'emission_source_lines':[
                                    {'line':j+1,'text':line} for j,line in enumerate(lines) if any(k in line.lower() for k in ['emiss','intensity','diffuse_color','texture_2d'])]}
            matrix=np.array(f['world_matrix_row_vector']);orientation=matrix[:3,:3]/np.linalg.norm(matrix[:3,:3],axis=1)[:,None]
            if abs(orientation[2,2])<.99:raise ValueError('Nonhorizontal fixture needs audit')
            # USD area lights emit along local -Z. Keep the fixture tangent axes,
            # orient the emission normal downward and use its lower emitting face.
            if orientation[2,2]<0:orientation[[1,2]]*=-1
            lo,hi=np.array(f['world_aabb_min']),np.array(f['world_aabb_max']);center=(lo+hi)/2;center[2]=lo[2]-.002
            world=np.eye(4);world[:3,:3]=orientation;world[3,:3]=center
            circular='/Geo_M_Light2_low/' in f['mesh'];size=hi-lo
            spec.append({'path':f'/World/HospitalFixtureLights/Light{i:03d}','source_mesh':f['mesh'],
                'fixture_world_matrix_row_vector':matrix.tolist(),'light_world_matrix_row_vector':world.tolist(),
                'type':'DiskLight' if circular else 'RectLight','radius_m':float(min(size[:2])/2),
                'width_m':float(size[0]),'height_m':float(size[1]),
                'emitter_rule':'Center of audited fixture footprint, 2 mm below lower face to avoid self-shadow; orientation uses fixture axes with local -Z down'} )
        profiles=[]
        for intensity in [5000.,15000.,45000.]:
            profile=native_profile(lights,renderer,1);profile['lighting_mode']='hospital_fixture_lights'
            profile['fallback_fixture_lights']=[{**s,'intensity':intensity} for s in spec]
            profile['policy']='All audited round and panel ceiling fixtures, no trajectory filtering; same radiance family, original native lights/exposure unchanged'
            profiles.append(profile)
        write_json(out/'profiles.json',profiles);write_json(out/'fixture_materials.json',shader_records);write_json(out/'mdl_sources.json',source_records)
        write_json(out/'summary.json',{'model_calls':0,'physics_reexecuted':False,'fixture_count':len(spec),'DiskLight_count':sum(s['type']=='DiskLight' for s in spec),
            'RectLight_count':sum(s['type']=='RectLight' for s in spec),'intensities':[5000,15000,45000],'selection':'Lowest intensity passing all 48 pose criteria and manual review','source':'Actual fixture meshes/transforms only; no route/camera position used',**provenance(ROOT)})
        print('FIXTURE_PROFILES_READY',len(spec),flush=True);code=0
    except Exception as exc:traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc)})
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
