"""Pre-model fixed-exposure calibration on all saved no-human observations."""
import argparse,json,sys,traceback
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.hospital_episode import digest
from research.nova_bright import BRIGHT_RULES,EXPOSURE_STOPS,image_stats,bright_pass,apply_profile


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-run',required=True);p.add_argument('--output-dir',required=True)
    p.add_argument('--fixed-lights-fallback',action='store_true')
    a=p.parse_args();source=Path(a.source_run).resolve();out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=False)
    meta=json.loads((source/'metadata.json').read_text());cfg=meta['config']
    paths=sorted((source/'raw/requests').glob('*.json'));events=[json.loads(x.read_text()) for x in paths]
    hashes={str(x):digest(x) for x in [source/'metadata.json',*paths]}
    candidates=[1500,5000,15000] if a.fixed_lights_fallback else EXPOSURE_STOPS
    light_positions=[];last=None;distance=0.
    for e in events:
        pos=np.array(e['observation']['position'][:2])
        if last is not None:distance+=float(np.linalg.norm(pos-last))
        if not light_positions or distance>=2.:
            light_positions.append([*pos.tolist(),2.1]);distance=0.
        last=pos
    write_json(out/'preregistered.json',{'model_calls':0,'source_hashes':hashes,'candidates':candidates,
        'fixed_light_positions':light_positions if a.fixed_lights_fallback else [],
        'candidate_type':'world-fixed sphere intensity, exposure +2 stops' if a.fixed_lights_fallback else 'exposure stops',
        'criteria':BRIGHT_RULES,'selection':'Lowest exposure passing every saved observation plus manual visual review. If none, fixed world lights fallback requires separate preregistered calibration; no model call.',**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        import carb
        from PIL import Image
        from research.nova_render_saved import SavedRenderer
        render=SavedRenderer(app,cfg);settings=carb.settings.get_settings()
        keys=['/rtx/post/tonemap/'+k for k in ['op','filmIso','exposureTime','fNumber','responsivity','enableSrgbToGamma','whitepoint','colorMode']]
        defaults={k:settings.get(k) for k in keys};write_json(out/'renderer_before.json',defaults)
        base_time=float(defaults['/rtx/post/tonemap/exposureTime']);results=[]
        for candidate in candidates:
            stop=2 if a.fixed_lights_fallback else candidate
            profile={'method':'fixed_renderer_exposure','relative_exposure_stops':stop,'lights':[],
                'renderer_settings':{**{k:v for k,v in defaults.items() if v is not None},
                    '/rtx/post/tonemap/op':2,'/rtx/post/tonemap/exposureTime':base_time*2**stop,
                    '/rtx/post/histogram/enabled':False,'/rtx/post/motionblur/enabled':False,'/rtx/post/tonemap/dither':0.}}
            if a.fixed_lights_fallback:
                profile['method']='fixed_scene_stationary_interior_lights_and_fixed_exposure'
                profile['lights']=[{'world_xyz':xyz,'intensity':float(candidate),'radius_m':.3} for xyz in light_positions]
            folder=out/(f'fixed_lights_{candidate}' if a.fixed_lights_fallback else f'exposure_{stop:02d}');folder.mkdir()
            if a.fixed_lights_fallback:render.stage.RemovePrim('/World/BrightHospital')
            actual=apply_profile(profile,render.stage)
            write_json(folder/'profile.json',profile);write_json(folder/'renderer_readback.json',actual)
            rows=[]
            for e in events:
                camera=render.pose(e);rgb=render.capture();rid=e['request_id']
                Image.fromarray(rgb).save(folder/f'rgb_{rid:02d}.png')
                stats=image_stats(rgb);rows.append({'request_id':rid,'camera_world_matrix_row_vector':camera,**stats,'pass':bright_pass(stats)})
            write_json(folder/'metrics.json',rows);result={'candidate':candidate,'profile_directory':folder.name,'exposure_stops':stop,'all_pass':all(x['pass'] for x in rows),
                'failed_request_ids':[x['request_id'] for x in rows if not x['pass']],
                'minimum_mean':min(x['mean'] for x in rows),'minimum_p95':min(x['p95'] for x in rows),
                'max_saturation':max(x['saturated_pixel_fraction'] for x in rows)};results.append(result)
            print('EXPOSURE_CANDIDATE',json.dumps(result),flush=True)
        if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Source changed')
        selected=next((r['profile_directory'] for r in results if r['all_pass']),None)
        write_json(out/'summary.json',{'status':'NUMERIC_PASS_REQUIRES_VISUAL_REVIEW' if selected else 'NO_EXPOSURE_CANDIDATE_PASS',
            'model_calls':0,'physics_reexecuted':False,'selected_profile_directory':selected,'candidates':results})
        print('BRIGHT_CALIBRATION_COMPLETE',selected,flush=True);code=0
    except Exception as exc:traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc),'model_calls':0})
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
