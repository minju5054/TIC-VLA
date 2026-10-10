"""Preserve a paused search; cheaply filter its fixed candidates before rendering."""
import argparse
import json
import sys
import traceback
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.reveal_prefilter import human_samples, temporal_pairs, prefilter_result
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination
from research.analyze_nova_dynamic_handoff import load_saved
from research.records import write_json, provenance, clocks
from research.hospital_episode import digest
from research.control import rotation_wxyz


def validate_reuse(source, candidates, base_sha, source_hashes):
    """Validate complete evidence only; never touch incomplete candidate folders."""
    from PIL import Image
    frozen=json.loads((source/'preregistered.json').read_text())
    if frozen['config_sha256']!=base_sha:raise ValueError('Reuse base-config SHA mismatch')
    verify_hashes(source_hashes)
    geo={c['id']:c for c in candidates};reused={}
    lighting=json.loads((source/'applied_lighting.json').read_text())
    for path in sorted(source.glob('**/candidate_*/observations.json')):
        cid=int(path.parent.name.split('_')[-1]);c=geo[cid]
        if not c['render_candidate'] or cid in reused:raise ValueError('Unexpected/duplicate completed candidate')
        batch=path.parent.parent
        applied=batch/'lighting.json' if batch.name.startswith('render_batch_') else source/'render_lighting.json'
        if json.loads(applied.read_text())!=lighting:raise ValueError('Reuse lighting mismatch')
        rows=json.loads(path.read_text())
        if [r['request_id'] for r in rows]!=list(range(1,49)):raise ValueError('Reuse requires 48 complete observations')
        for row in rows:
            with Image.open(row['rgb']) as im:
                if im.size!=(1920,1080) or im.mode!='RGB':raise ValueError('Reuse RGB shape/mode mismatch')
                im.verify()
            with Image.open(row['mask']) as im:mask=np.asarray(im)>0
            if mask.shape!=(1080,1920) or int(mask.sum())!=row['human_visible_pixel_count']:
                raise ValueError('Reuse mask mismatch')
            if abs(float(mask.mean())-row['human_visible_fraction'])>1e-15:raise ValueError('Reuse pixel fraction mismatch')
        reused[cid]={'candidate_id':cid,'position':c['position'],'yaw':c['yaw'],'observations':str(path),
                     'base_config_sha256':base_sha,'geometry_candidates_sha256':digest(source/'geometry_candidates.json'),
                     'source_evidence_manifest_sha256':digest(source/'source_evidence_sha256.json'),
                     'observations_sha256':digest(path),'lighting_identical':True,'all_48_rgb_and_masks_valid':True}
    return reused


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='configs/research/reveal_window_prefilter.json')
    p.add_argument('--output-dir',required=True);a=p.parse_args()
    rules=json.loads(Path(a.config).read_text());source=Path(rules['source_search_dir']).resolve()
    parent=json.loads((source/'preregistered.json').read_text());cfg=parent['config']
    if digest('configs/research/reveal_window_search.json')!=parent['config_sha256']:
        raise ValueError('Original base configuration changed')
    out=validate_destination(a.output_dir,[source,cfg['baseline_run'],cfg['baseline_analysis']]);out.mkdir(parents=True)
    original_hashes=json.loads((source/'source_evidence_sha256.json').read_text());verify_hashes(original_hashes)
    paused_hashes=evidence_hashes([source]);write_json(out/'paused_source_sha256.json',paused_hashes)
    write_json(out/'source_evidence_sha256.json',{**original_hashes,**paused_hashes})
    write_json(out/'preregistered.json',{'config':cfg,'config_sha256':parent['config_sha256'],
               'prefilter':rules,'prefilter_config_sha256':digest(a.config),'registration':clocks(),
               'parent_search_dir':str(source),'parent_preregistration_sha256':digest(source/'preregistered.json'),
               'protocol_note':'User-authorized prefilter replaces exhaustive render scheduling only; visibility/ranking unchanged.',
               'code':provenance(ROOT)})
    run=load_saved(cfg['baseline_run']);candidates=json.loads((source/'geometry_candidates.json').read_text())
    reused=validate_reuse(source,candidates,parent['config_sha256'],original_hashes)
    write_json(out/'reused_candidates.json',reused)
    write_json(out/'metadata.json',{'baseline_run':str(run['path']),'config':cfg,'parent_search_dir':str(source),
               'prefilter_config_sha256':digest(a.config),'base_config_sha256':parent['config_sha256'],
               'model_calls':0,'navigation_physics_reexecuted':False,'prefilter_rgb_renders':0,**provenance(ROOT)})
    for name in ['camera_matrices.json','human_asset.json']:
        write_json(out/name,json.loads((source/name).read_text()))
    matrices=np.asarray(json.loads((source/'camera_matrices.json').read_text()))
    asset=json.loads((Path(cfg['previous_geometry'])/'robot_asset.json').read_text())
    extrinsic=np.asarray(asset['camera_body_transform'])
    for e,m in zip(run['events'],matrices):
        body=np.eye(4);body[:3,:3]=rotation_wxyz(e['observation']['quaternion_wxyz']).T
        body[3,:3]=e['observation']['position'];np.testing.assert_allclose(extrinsic@body,m,rtol=0,atol=1e-6)
    camera=next(c for c in asset['cameras'] if c['path']==run['cfg']['camera']['prim'])
    hs=camera['horizontal_aperture']/(2*camera['focal_length']);vs=camera['vertical_aperture']/(2*camera['focal_length'])
    inverses=np.linalg.inv(matrices)
    pair_metrics={c['id']:temporal_pairs(run,c,cfg) for c in candidates}
    # Register before querying final ray outcomes. No renderer, render product,
    # annotator, human authoring, World reset/step or timeline play is used.
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'disable_viewport_updates':True});code=1
    try:
        import carb,omni.usd,omni.timeline
        from pxr import UsdGeom,UsdPhysics
        from omni.physx import get_physx_interface,get_physx_scene_query_interface
        from research.audit_hospital_lights import inspect_stage
        timeline=omni.timeline.get_timeline_interface();timeline.stop()
        stage=omni.usd.get_context().get_stage();stage.GetRootLayer().subLayerPaths.append(run['cfg']['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        collisions=inspect_stage(stage)[-1]
        if collisions!=json.loads(Path(cfg['collision_inventory']).read_text()):raise ValueError('Frozen collider inventory mismatch')
        if not any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse()):UsdPhysics.Scene.Define(stage,'/World/StaticQueryScene')
        get_physx_interface().force_load_physics_from_usd();query=get_physx_scene_query_interface()
        floor=query.raycast_closest(carb.Float3(*map(float,run['xy'][0]),.3),carb.Float3(0,0,-1),.6)
        if not floor['hit'] or abs(floor['position'][2])>.02:raise ValueError('Static ray query floor probe failed')
        detailed={};results=[];ray_count=0
        for c in candidates:
            pairs=pair_metrics[c['id']];g=c['geometry'];rays=None;groups=[]
            if g['floor_ok'] and not g['overlap_paths'] and c['bypass']['pass'] and any(p['temporal_pass'] for p in pairs):
                samples=human_samples(c['position'],c['yaw'],rules);groups=[s['group'] for s in samples]
                xyz=np.array([s['world'] for s in samples]);homogeneous=np.column_stack([xyz,np.ones(len(xyz))]);rays=[]
                for rid,(matrix,inverse) in enumerate(zip(matrices,inverses),1):
                    local=homogeneous@inverse;depth=-local[:,2];factor=rules['camera_envelope_factor']
                    envelope=(depth>0)&(np.abs(local[:,0])<=factor*hs*depth)&(np.abs(local[:,1])<=factor*vs*depth)
                    origin=matrix[3,:3];hits=[]
                    for point in xyz:
                        delta=point-origin;length=float(np.linalg.norm(delta));found=[]
                        def callback(hit):
                            if str(hit.collision).startswith('/Root/') and hit.distance<length-1e-6:
                                found.append({'prim':str(hit.collision),'distance_m':float(hit.distance)})
                            return True
                        query.raycast_all(carb.Float3(*map(float,origin)),carb.Float3(*map(float,delta/length)),length,callback,True)
                        hits.append(min(found,key=lambda h:h['distance_m']) if found else None);ray_count+=1
                    exposed=[bool(inside and hit is None) for inside,hit in zip(envelope,hits)]
                    rays.append({'request_id':rid,'exposed':exposed,'in_broad_camera_envelope':envelope.tolist(),'first_static_hits':hits})
                detailed[c['id']]={'samples':samples,'observations':rays}
            result=prefilter_result(c,pairs,rays,groups,rules)
            # Detailed rays live separately to keep the main candidate table small.
            result.pop('ray_observations');result.pop('pair_metrics')
            copy={**c,'original_render_candidate':c['render_candidate'],'prefilter':result,
                  'prefilter_pass':result['pass'],'reused_observations':reused.get(c['id'],{}).get('observations'),
                  'render_candidate':bool(result['pass'] or c['id'] in reused),
                  'new_render_required':bool(result['pass'] and c['id'] not in reused)}
            results.append(copy)
        counts=Counter(reason for c in results for reason in c['prefilter']['rejection_reasons'])
        physical=[c for c in results if c['prefilter']['physical_pass']]
        bypass=[c for c in physical if c['prefilter']['bypass_pass']]
        conflict=[c for c in bypass if c['prefilter']['conflicting_pair_count']]
        lead=[c for c in conflict if c['prefilter']['positive_lead_pair_count']]
        current=[c for c in lead if c['prefilter']['current_clear_pair_count']]
        temporal=[c for c in current if c['prefilter']['temporal_pass_pair_count']]
        survivors=[c for c in results if c['prefilter_pass']]
        summary={'candidate_count_before':len(results),'previous_render_cohort_count':sum(c['original_render_candidate'] for c in results),
                 'funnel':{'floor_static_valid':len(physical),'bypass_valid':len(bypass),'remaining_conflict':len(conflict),
                           'positive_lead':len(lead),'current_clearance_positive':len(current),'application_before_conflict':len(temporal),
                           'multi_ray_possible_transition':len(survivors)},
                 'candidate_count_after':len(survivors),'rejection_counts':dict(counts),
                 'completed_reused_count':len(reused),'reused_survivor_count':sum(c['id'] in reused for c in survivors),
                 'new_render_required_count':sum(c['new_render_required'] for c in results),
                 'surviving_candidate_ids':[c['id'] for c in survivors],
                 'ray_count':ray_count,'prefilter_rgb_renders':0,'semantic_visibility_assigned_from_rays':False,
                 'model_calls':0,'navigation_physics_reexecuted':False,'collider_inventory_identical':True,
                 'camera_matrices_verified':True,'timeline_playing':timeline.is_playing(),
                 'base_config_and_visibility_ranking_unchanged':True}
        write_json(out/'geometry_candidates.json',results)
        write_json(out/'temporal_pair_prefilter.json',pair_metrics);write_json(out/'multi_ray_prefilter.json',detailed)
        write_json(out/'prefilter_summary.json',summary)
        oldspace=json.loads((source/'search_space.json').read_text());write_json(out/'search_space.json',{**oldspace,'prefilter_summary':summary})
        verify_hashes(paused_hashes);verify_hashes(original_hashes)
        write_json(out/'prefilter_immutable.json',{'paused_search_unchanged':True,'baseline_unchanged':True})
        print('PREFILTER_COMPLETE',json.dumps(summary),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(out/'prefilter_failure.json',{'error':repr(exc)})
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
