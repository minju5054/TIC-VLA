"""One immutable user-selected C19 placement. No search and no model interface."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance,clocks
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination,classify,state
from research.analyze_nova_dynamic_handoff import load_saved

BASELINE=ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01'
RULES=ROOT/'configs/research/reveal_window_search.json'


def placement(event):
    if event['request_id']!=19:raise ValueError('Only C19 is authorized')
    x,y,yaw=event['agent_pose_at_observation']
    return {'position':[x,y+.40,0.], 'yaw':float((yaw+2*np.pi)%(2*np.pi)-np.pi)}


def check_frozen(out):
    frozen=json.loads((out/'freeze.json').read_text())
    event=json.loads((BASELINE/'raw/requests/request_000019.json').read_text())
    if frozen['human']!=placement(event):raise ValueError('Frozen placement changed')
    verify_hashes(frozen['source_evidence_sha256'])
    return frozen,load_saved(BASELINE)


def freeze(out):
    out=validate_destination(out,[BASELINE]);run=load_saved(BASELINE)
    if not run['complete'] or len(run['events'])!=48:raise ValueError('Complete baseline required')
    cfg=json.loads(RULES.read_text());event=run['events'][18];human=placement(event)
    cfg.update(human_yaw_rad=human['yaw'],human_orientation_rule='wrap(saved C19 yaw + pi)',
               ranking=['Single user-fixed candidate only; no ranking or replacement'],
               candidate_positions=[human['position']],render_policy='Exactly one frozen human; all 48 saved observations, semantics authoritative')
    sources=[BASELINE,RULES,Path(cfg['collision_inventory']),Path(cfg['previous_render'])/'human_asset.json',
             ROOT/'outputs/e16-c-observation-map-20261010-01/e16_c_observation_poses.csv']
    hashes=evidence_hashes(sources)
    out.mkdir(parents=True,exist_ok=False)
    write_json(out/'freeze.json',{'human':human,'C19_observation':event['observation'],'source_evidence_sha256':hashes,
        'formula':'x=C19.x; y=C19.y+0.40 world metres; z=0; yaw=wrap(C19.yaw+pi)',
        'coordinate_convention':'Hospital world metres, +Z up; yaw CCW from +X, radians; no local-frame offset',
        'candidate_count':1,'retuning_permitted':False,'before_any_human_render':True,
        'visibility':cfg['visibility'],'code_sha256':evidence_hashes([Path(__file__)]),**clocks(),**provenance(ROOT)})
    write_json(out/'preregistered.json',{'config':cfg,'task':'SINGLE MANUAL C19 PLACEMENT; NOT A SEARCH'})
    write_json(out/'source_evidence_sha256.json',hashes)
    write_json(out/'human_asset.json',json.loads((Path(cfg['previous_render'])/'human_asset.json').read_text()))
    print(json.dumps({'C19':event['observation'],'human':human},indent=2))


def physical(out):
    frozen,run=check_frozen(out)
    if (out/'physical.json').exists():raise FileExistsError('Physical result already exists')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'disable_viewport_updates':True});code=1
    try:
        import omni.usd,omni.timeline
        from pxr import UsdGeom
        from research.blind_corner_queries import Queries
        from research.audit_hospital_lights import inspect_stage
        cfg=json.loads((out/'preregistered.json').read_text())['config']
        omni.timeline.get_timeline_interface().stop();stage=omni.usd.get_context().get_stage()
        stage.GetRootLayer().subLayerPaths.append(run['cfg']['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        assert inspect_stage(stage)[-1]==json.loads(Path(cfg['collision_inventory']).read_text())
        query=Queries(stage);human=frozen['human'];p=human['position']
        geometry=query.human(p,cfg['human_radius_m'],human['yaw'])
        bypass=query.bypass(p,cfg['robot_radius_m'],heading=run['events'][18]['agent_pose_at_observation'][2])
        clearance=float(np.linalg.norm(np.array(p[:2])-run['xy'][0])-cfg['robot_radius_m']-cfg['human_radius_m'])
        checks={'floor_support':geometry['floor_ok'],'no_static_overlap':not geometry['overlap_paths'],
                'no_initial_robot_conflict':clearance>0,'local_bypass':bypass['pass']}
        result={'id':1,'position':p,'yaw':human['yaw'],'geometry':geometry,'bypass':bypass,
                'initial_robot_human_clearance_m':clearance,'checks':checks,'pass':all(checks.values()),
                'model_calls':0,'render_calls':0,'navigation_physics_reexecuted':False,
                'note':'Existing capsule plus yaw-oriented visual bounds; sampled local bypass, not full navigation feasibility.'}
        write_json(out/'physical.json',result);verify_hashes(frozen['source_evidence_sha256'])
        print('C19_PHYSICAL',json.dumps(result),flush=True);code=0
    finally:app.close(exit_code=code)


def semantic(out):
    frozen,run=check_frozen(out);candidate=json.loads((out/'physical.json').read_text())
    if not candidate['pass']:raise ValueError('C19 HUMAN PLACEMENT PHYSICALLY INVALID')
    folder=out/'candidate_001';folder.mkdir(exist_ok=False)
    cfg=json.loads((out/'preregistered.json').read_text())['config']
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        import omni.replicator.core as rep
        from isaacsim.core.utils.semantics import add_labels
        from research.nova_render_saved import SavedRenderer
        from research.hospital_lights import apply_hospital_lights
        from research.human_actor import author_human
        from research.hospital_scene import rotation_wxyz
        from research.nova_occlusion import segmentation_stats
        from PIL import Image
        render=SavedRenderer(app,run['cfg'])
        write_json(out/'applied_lighting.json',apply_hospital_lights(render.stage,run['cfg']['hospital_lighting']))
        asset=json.loads((out/'human_asset.json').read_text());human=frozen['human']
        human_cfg={'prim':'/World/SearchHuman','asset':asset['asset'],'radius_m':cfg['human_radius_m'],
                   'capsule_cylinder_height_m':1.2,'yaw':human['yaw'],'start_position':human['position']}
        author_human(render.stage,human_cfg,asset,physical=False)
        add_labels(render.stage.GetPrimAtPath(human_cfg['prim']),['occluded_human'])
        segmentation=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
        segmentation.attach([render.product]);rows=[]
        extrinsic=np.asarray(json.loads((Path(cfg['previous_geometry'])/'robot_asset.json').read_text())['camera_body_transform'])
        for event in run['events']:
            obs=event['observation'];matrix=render.pose(event)
            body=np.eye(4);body[:3,:3]=rotation_wxyz(obs['quaternion_wxyz']).T;body[3,:3]=obs['position']
            np.testing.assert_allclose(matrix,extrinsic@body,atol=1e-6,rtol=0)
            rgb=render.capture();packed=segmentation.get_data()
            stats,mask=segmentation_stats(packed['data'],packed['info']['idToLabels'])
            rid=event['request_id'];row={'request_id':rid,'sim_time':obs['sim_time'],**stats,
                'state':state(stats,cfg['visibility']),'camera_world_matrix_row_vector':matrix}
            rows.append(row)
            Image.fromarray(rgb).save(folder/f'C{rid:02d}.png');Image.fromarray((mask*255).astype('uint8')).save(folder/f'C{rid:02d}.mask.png')
            overlay=rgb.copy();overlay[mask]=(rgb[mask]*.45+np.array([255,20,150])*.55).astype('uint8')
            Image.fromarray(overlay).save(folder/f'C{rid:02d}.overlay.png')
            print('C19_SEMANTIC',rid,row['state'],stats['human_visible_pixel_count'],flush=True)
        result=classify(candidate,rows,run,cfg)
        result.update(render_folder=str(folder.resolve()),occluding_prim_path=None)
        write_json(folder/'result.json',result);write_json(out/'candidates.json',[result])
        summary={'physical_pass':True,'semantic_preflight_pass':result['strict_qualified'],
                 'decision':'C19 STRICT PREFLIGHT PASS' if result['strict_qualified'] else 'C19 STRICT REVEAL FAILED - STOP BEFORE INFERENCE',
                 'selected_candidate_id':1 if result['strict_qualified'] else None,'diagnostic_candidate_id':1,
                 'first_clear_request_id':result.get('fresh_request_id'),'failures':result['failures'],
                 'model_calls':0,'human_model_run_executed':False,'navigation_physics_reexecuted':False}
        write_json(out/'summary.json',summary);verify_hashes(frozen['source_evidence_sha256'])
        print('C19_SEMANTIC_COMPLETE',json.dumps(summary),flush=True);code=0
    finally:app.close(exit_code=code)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['freeze','physical','semantic'],required=True)
    p.add_argument('--output-dir',required=True);a=p.parse_args();globals()[a.phase](Path(a.output_dir).resolve())
