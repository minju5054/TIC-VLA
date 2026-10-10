"""Single exact-C19 cart physical freeze and saved-camera semantic preflight."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.analyze_nova_dynamic_handoff import load_saved
from research.cart_geometry import freeze_pose,rotation,outline,path_clearance
from research.hospital_cart import SOURCE,COPY,signature,author,mask_stats
BASELINE=ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01'


def physical(out):
    run=load_saved(BASELINE);cfg=run['cfg'];out=validate_destination(out,[BASELINE]);out.mkdir(parents=True)
    hashes=evidence_hashes([BASELINE,ROOT/'outputs/e16-cart-audit-20261010-01/asset.json',
        ROOT/'outputs/e16-c19-manual-20261010-01',ROOT/'outputs/e16-c19-south-20261010-01'])
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'disable_viewport_updates':True});code=1
    try:
        import omni.usd,omni.client
        from pxr import UsdGeom
        from research.blind_corner_queries import Queries
        stage=omni.usd.get_context().get_stage();stage.GetRootLayer().subLayerPaths.append(cfg['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        query=Queries(stage);source=stage.GetPrimAtPath(SOURCE);bounds=UsdGeom.BBoxCache(0,['default','render','proxy']).ComputeUntransformedBound(source).ComputeAlignedRange()
        xy=run['events'][18]['agent_pose_at_observation'][:2]
        support=query.q.raycast_closest(query.carb.Float3(*xy,.3),query.carb.Float3(0,0,-1),.6)
        if not support['hit']:raise ValueError('No actual floor support')
        cart=freeze_pose(run['events'],[list(bounds.GetMin()),list(bounds.GetMax())],float(support['position'][2]))
        radius=cfg['spatial_turn_gate']['robot_conservative_radius_m'];h=cart['footprint_half_m'];z=cart['height_m']/2
        floor_checks=[query.floor(p) for p in outline(cart,0.)[::17]]
        overlaps=query.overlap([*xy,cart['center_world_xyz'][2]+z+.001],extent=[*h,z-.001],yaw=cart['yaw_rad'])
        initial=path_clearance(run['xy'][:2],cart,radius)['minimum_clearance_m'];baseline=path_clearance(run['xy'],cart,radius,run['times'])
        f=np.array([np.cos(cart['tangent_rad']),np.sin(cart['tangent_rad'])]);n=np.array([-f[1],f[0]])
        half_long=max(h);half_short=min(h);traces=[];bypass=None
        for side in [-1,1]:
            for extra in np.arange(.05,.501,.05):
                offset=side*(half_short+radius+extra)
                points=np.array(xy)+offset*n+np.linspace(-half_long-radius-.3,half_long+radius+.3,41)[:,None]*f
                trace=query.robot_path(points,radius+.05);clear=path_clearance(points,cart,radius)['minimum_clearance_m']
                ok=clear>0 and all(r['floor'] and not r['overlap'] for r in trace)
                traces.append({'side':side,'offset_m':float(offset),'pass':ok,'trace':trace,'cart_clearance_m':clear})
                if ok and bypass is None:bypass={'side':side,'offset_m':float(offset),'world_xy':points.tolist(),'clearance_m':clear}
        assets={}
        asset_url=cfg['scene']['usd'].rsplit('/',1)[0]+'/Props/SM_SupplyCart_01e.usd'
        for url in [cfg['scene']['usd'],asset_url]:
            result,_,data=omni.client.read_file(url)
            if result!=omni.client.Result.OK:raise ValueError('Asset bytes unavailable')
            assets[url]=hashlib.sha256(bytes(data)).hexdigest()
        frozen={'source_prim':SOURCE,'asset_url':asset_url,'scene_usd':cfg['scene']['usd'],'copy_prim':COPY,
            'asset_sha256':assets,'source_prim_signature':signature(stage,SOURCE),'source_world_matrix_row_vector':np.asarray(UsdGeom.Xformable(source).ComputeLocalToWorldTransform(0)).tolist(),
            'cart':cart,'C19_observation':run['events'][18]['observation'],'robot_radius_m':radius,
            'orientation_rule':'Longest local horizontal bound axis parallel to measured C18->C20 tangent; local +axis points with tangent',
            'center_rule':'Exact saved C19 XY = footprint centre; Z = support floor, not source pivot',
            'coordinate_convention':'World metres, +Z up, yaw CCW from +X; USD matrices act on row vectors',
            'visibility_rules':json.loads((ROOT/'configs/research/reveal_window_search.json').read_text())['visibility'],
            'selection_rule':'Primary actual first CLEAR and immediate OLD; if first CLEAR=C1, first later OLD-conflict to FRESH-clear pair is secondary only; no severity ranking',
            'candidate_count':1,'source_evidence_sha256':hashes,'before_model_inference':True,**provenance(ROOT)}
        write_json(out/'freeze.json',frozen)
        copied=author(stage,frozen);write_json(out/'copied_cart.json',copied)
        checks={'floor_support':all(r[0] for r in floor_checks),'no_static_overlap':not overlaps,'no_initial_collision':initial>0,
            'baseline_intersects_inflated_cart':baseline['minimum_clearance_m']<=0,'local_bypass':bypass is not None}
        result={'checks':checks,'pass':all(checks.values()),'static_overlap_paths':overlaps,'floor_checks':floor_checks,
            'initial_clearance_m':initial,'baseline':baseline,'bypass':bypass,'bypass_checks':traces,
            'model_calls':0,'navigation_physics_reexecuted':False,'proxy':'Original oriented full cart bounds + unchanged enclosing Nova disk; local bypass uses an additional .05m static margin'}
        write_json(out/'physical.json',result);verify_hashes(hashes)
        print('CART_PHYSICAL',json.dumps({k:v for k,v in result.items() if k!='bypass_checks'}),flush=True);code=0
    finally:app.close(exit_code=code)


def semantic(out):
    frozen=json.loads((out/'freeze.json').read_text());verify_hashes(frozen['source_evidence_sha256']);run=load_saved(BASELINE)
    if not json.loads((out/'physical.json').read_text())['pass']:raise ValueError('Physical gate failed; no model run')
    folder=out/'visibility';folder.mkdir(exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        import omni.replicator.core as rep
        from research.nova_render_saved import SavedRenderer
        from research.hospital_lights import apply_hospital_lights
        from research.hospital_scene import rotation_wxyz
        from PIL import Image
        render=SavedRenderer(app,run['cfg']);apply_hospital_lights(render.stage,run['cfg']['hospital_lighting']);author(render.stage,frozen)
        annotator=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False});annotator.attach([render.product])
        extrinsic=np.asarray(json.loads((BASELINE/'robot_asset.json').read_text())['camera_body_transform']);rows=[]
        for event in run['events']:
            obs=event['observation'];matrix=render.pose(event);body=np.eye(4);body[:3,:3]=rotation_wxyz(obs['quaternion_wxyz']).T;body[3,:3]=obs['position']
            np.testing.assert_allclose(matrix,extrinsic@body,atol=1e-6,rtol=0)
            rgb=render.capture();stats,mask=mask_stats(annotator.get_data(),frozen['visibility_rules']);rid=event['request_id']
            row={'request_id':rid,'sim_time':obs['sim_time'],**stats,'camera_world_matrix_row_vector':matrix};rows.append(row)
            Image.fromarray(rgb).save(folder/f'C{rid:02d}.png');Image.fromarray((mask*255).astype('uint8')).save(folder/f'C{rid:02d}.mask.png')
            overlay=rgb.copy();overlay[mask]=(overlay[mask]*.45+np.array([255,20,150])*.55).astype('uint8');Image.fromarray(overlay).save(folder/f'C{rid:02d}.overlay.png')
            print('CART_VISIBILITY',rid,stats['state'],stats['cart_visible_pixel_count'],flush=True)
        write_json(out/'visibility.json',rows);write_json(out/'summary.json',{'physical_pass':True,'first_visible_request_id':next((r['request_id'] for r in rows if r['visible']),None),
            'first_clear_request_id':next((r['request_id'] for r in rows if r['state']=='CLEAR'),None),'cart_visible_at_C1':rows[0]['visible'],
            'model_run_permitted':True,'visibility_diagnostic_only':True,'model_calls':0,'navigation_physics_reexecuted':False})
        verify_hashes(frozen['source_evidence_sha256']);code=0
    finally:app.close(exit_code=code)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['physical','semantic'],required=True);p.add_argument('--output-dir',required=True)
    a=p.parse_args();globals()[a.phase](Path(a.output_dir).resolve())
