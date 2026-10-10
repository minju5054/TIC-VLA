"""Model-free native-light candidates on saved camera poses; original exposure."""
import argparse,json,sys,traceback
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.hospital_episode import digest
from research.nova_bright import BRIGHT_RULES,image_stats,bright_pass
from research.hospital_lights import native_profile,apply_hospital_lights


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['source-run','audit-dir','output-dir']:p.add_argument('--'+key,required=True)
    p.add_argument('--profiles-json',help='Preregistered fixture fallback profiles, after native failure')
    a=p.parse_args();source=Path(a.source_run).resolve();audit=Path(a.audit_dir).resolve();out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((source/'metadata.json').read_text())['config'];events=[json.loads(p.read_text()) for p in sorted((source/'raw/requests').glob('*.json'))]
    lights=json.loads((audit/'lights.json').read_text());renderer=json.loads((audit/'renderer_defaults.json').read_text())
    profiles=json.loads(Path(a.profiles_json).read_text()) if a.profiles_json else [native_profile(lights,renderer,m) for m in [1,2,4]]
    hashes={str(p):digest(p) for p in [source/'metadata.json',*sorted((source/'raw/requests').glob('*.json')),*sorted(audit.glob('*.json'))]}
    write_json(out/'preregistered.json',{'profiles':profiles,'criteria':BRIGHT_RULES,'selection':'First passing least-invasive profile in listed order, all 48 poses plus manual scene visibility review',
        'model_calls':0,'source_evidence_sha256':hashes,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        from research.nova_render_saved import SavedRenderer
        from PIL import Image
        render=SavedRenderer(app,cfg);results=[]
        for index,profile in enumerate(profiles,1):
            folder=out/f'candidate_{index:02d}';folder.mkdir()
            readback=apply_hospital_lights(render.stage,profile);write_json(folder/'profile.json',profile);write_json(folder/'applied.json',readback)
            rows=[]
            for e in events:
                matrix=render.pose(e);rgb=render.capture();rid=e['request_id'];Image.fromarray(rgb).save(folder/f'rgb_{rid:02d}.png')
                stats=image_stats(rgb);rows.append({'request_id':rid,'camera_world_matrix_row_vector':matrix,**stats,'pass':bright_pass(stats)})
            write_json(folder/'brightness.json',rows)
            result={'candidate':index,'native_multiplier':profile['native_multiplier'],'fixture_light_count':len(profile['fallback_fixture_lights']),
                'all_pass':all(r['pass'] for r in rows),'failed_ids':[r['request_id'] for r in rows if not r['pass']],
                'min_mean':min(r['mean'] for r in rows),'min_p95':min(r['p95'] for r in rows),'max_saturation':max(r['saturated_pixel_fraction'] for r in rows)}
            results.append(result);print('LIGHT_CANDIDATE',json.dumps(result),flush=True)
        if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Saved source changed')
        write_json(out/'summary.json',{'model_calls':0,'physics_reexecuted':False,'candidates':results,'selected_candidate':next((r['candidate'] for r in results if r['all_pass']),None)})
        print('LIGHT_CALIBRATION_COMPLETE',flush=True);code=0
    except Exception as exc:traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc),'model_calls':0})
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
