"""Stopped-time RGB/semantic qualification of geometry-audited candidates."""
import argparse
import json
import sys
import traceback
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json, provenance
from research.hospital_episode import digest
from research.analyze_nova_dynamic_handoff import load_saved
from research.nova_dynamic_geometry import remaining_curve, polyline_clearance
from research.nova_occlusion import VISIBILITY_RULES, segmentation_stats, visibility_state, first_reveal, leakage_pass
from research.nova_bright import image_stats, bright_pass


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['baseline-run','geometry-dir','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args(); run=load_saved(a.baseline_run); geometry=Path(a.geometry_dir); out=Path(a.output_dir)
    source=json.loads((geometry/'metadata.json').read_text())
    if source['baseline_run']!=str(run['path']):raise ValueError('Wrong geometry source')
    info=json.loads((geometry/'summary.json').read_text()); rule=json.loads((geometry/'preregistered.json').read_text())
    if not info['baseline_spatial_gate_pass'] or not info['geometry_preflight_complete']:raise ValueError('Geometry prerequisite missing')
    hashes={str(p):digest(p) for d in [run['path'],geometry] for p in d.rglob('*') if p.is_file()}
    out.mkdir(parents=True,exist_ok=False)
    write_json(out/'metadata.json',{'baseline_run':str(run['path']),'geometry_dir':str(geometry),'source_evidence_sha256':hashes,
               'model_calls':0,'physics_reexecuted':False,'rule':rule,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080}); code=1
    try:
        import omni.replicator.core as rep
        from isaacsim.core.utils.semantics import add_labels
        from research.nova_render_saved import SavedRenderer
        from research.hospital_lights import apply_hospital_lights
        from research.human_actor import author_human
        from PIL import Image
        render=SavedRenderer(app,run['cfg']);write_json(out/'applied_lighting.json',apply_hospital_lights(render.stage,run['cfg']['hospital_lighting']))
        events=run['events'][:info['render_window_count']]
        expected=json.loads((geometry/'camera_matrices.json').read_text())
        def pose(e):
            matrix=render.pose(e)
            np.testing.assert_allclose(matrix,expected[e['request_id']-1],atol=1e-6,rtol=0)
            return matrix
        absent_dir=out/'absent';absent_dir.mkdir();absent=[]
        for e in events:
            pose(e);rgb=render.capture();absent.append(rgb);Image.fromarray(rgb).save(absent_dir/f"C{e['request_id']:02d}.png")
        segmentation=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
        segmentation.attach([render.product]);human=info['human_template'];asset=None;chosen=None;results=[]
        candidates=json.loads((geometry/'geometry_candidates.json').read_text())
        for c in [c for c in candidates if c.get('render_candidate')]:
            folder=out/f"candidate_{c['id']:03d}";folder.mkdir()
            _,asset=author_human(render.stage,{**human,'start_position':c['position']},asset,physical=False)
            add_labels(render.stage.GetPrimAtPath(human['prim']),['occluded_human'])
            # Independent scene variants, not visibility tricks in an experiment.
            # Actual run uses physical=True and authors a fixed actor before reset.
            rows=[]
            for e,reference in zip(events,absent):
                matrix=pose(e);rgb=render.capture();packed=segmentation.get_data()
                stats,mask=segmentation_stats(packed['data'],packed['info']['idToLabels'])
                row={'request_id':e['request_id'],**stats,'state':visibility_state(stats),
                     'leakage':leakage_pass(reference,rgb),'brightness':image_stats(rgb),'camera_world_matrix_row_vector':matrix}
                rows.append(row);Image.fromarray(rgb).save(folder/f"C{e['request_id']:02d}.png")
                Image.fromarray((mask*255).astype('uint8')).save(folder/f"C{e['request_id']:02d}.mask.png")
                if row['state']=='VISIBLE':break
            reveal=first_reveal(rows);fresh=reveal['fresh_request_id']
            _,remaining=remaining_curve(run['worlds'][fresh-2],events[fresh-2]['observation']['sim_time'],events[fresh-1]['observation']['sim_time']) if fresh and fresh>1 else (None,None)
            clearance=polyline_clearance(remaining,c['position'],rule['robot_radius_m']+.25) if remaining is not None else None
            prior=rows[:-1] if fresh else rows
            passed=bool(reveal['valid_transition'] and len(prior)>=VISIBILITY_RULES['minimum_hidden_preflight_observations']
                        and all(r['state']=='HIDDEN' and r['leakage']['pass'] for r in prior)
                        and all(bright_pass(r['brightness'],run['cfg']['bright_validation']) for r in rows)
                        and clearance is not None and clearance<0)
            result={**c,'rows':rows,'reveal':reveal,'actual_first_reveal_old_clearance_m':clearance,'pass':passed}
            write_json(folder/'result.json',result);results.append({'id':c['id'],'pass':passed,'reveal':reveal})
            print('OCCLUSION_RENDER_CANDIDATE',json.dumps(results[-1]),flush=True)
            if passed:chosen=result;break
            render.stage.RemovePrim(human['prim'])
        write_json(out/'human_asset.json',asset)
        write_json(out/'summary.json',{'occlusion_preflight_pass':bool(chosen),'baseline_spatial_gate_pass':True,
                   'model_calls':0,'physics_reexecuted':False,'render_results':results,'selected':chosen,
                   'human':{**human,'position':chosen['position']} if chosen else None,
                   'note':'Actual silhouette and paired-RGB review required before model authorization.'})
        if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Source changed')
        print('OCCLUSION_RENDER_COMPLETE',bool(chosen),flush=True);code=0
    except Exception as exc:traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc),'model_calls':0})
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
