"""Isaac 6 stopped-stage inspection/query/render only; never starts a policy."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from research.two_hz_preflight import (read, digest, canonical, PRIMARY, POLICY, RADIUS,
    HUMAN_RADIUS, MARGIN, SEPARATION, load_reference, references, point_path,
    rect_path, sample_path, bypass_vertices, early_clear)
from research.records import write_json


def source_bytes(url):
    import omni.client
    status, version, content=omni.client.read_file(url)
    if status!=omni.client.Result.OK: raise RuntimeError('Asset unavailable: '+url)
    return {'url':url,'sha256':hashlib.sha256(bytes(content)).hexdigest(),'size_bytes':len(bytes(content))}


def colliders(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    from research.audit_hospital_lights import serial
    cache=UsdGeom.BBoxCache(0,['default','render','proxy'],useExtentsHint=False); rows=[]
    for p in Usd.PrimRange(stage.GetPrimAtPath('/Root'),Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not p.HasAPI(UsdPhysics.CollisionAPI): continue
        box=cache.ComputeWorldBound(p).ComputeAlignedRange()
        matrix=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0))
        rows.append({'path':str(p.GetPath()),'type':p.GetTypeName(),'enabled':UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get(),
            'world_matrix_row_vector':matrix.tolist(),'world_xyz':matrix[3,:3].tolist(),
            'world_aabb_min':serial(box.GetMin()) if not box.IsEmpty() else None,
            'world_aabb_max':serial(box.GetMax()) if not box.IsEmpty() else None})
    return rows


def static_stage(app,cfg):
    import omni.usd, omni.timeline
    from pxr import UsdGeom
    omni.timeline.get_timeline_interface().stop()
    stage=omni.usd.get_context().get_stage()
    stage.GetRootLayer().subLayerPaths.append(cfg['scene']['usd'])
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    for _ in range(20): app.update()
    return stage


def ray(q, origin, direction, length):
    hits=[]
    def cb(h):
        path=str(h.collision)
        if path.startswith('/Root/'):
            hits.append({'prim':path,'distance_m':float(h.distance),'position':list(h.position),'normal':list(h.normal)})
        return True
    q.q.raycast_all(q.carb.Float3(*map(float,origin)),q.carb.Float3(*map(float,direction)),float(length),cb,True)
    return min(hits,key=lambda h:h['distance_m']) if hits else None


def inspect(app,cfg,out):
    from pxr import Usd, UsdGeom, Sdf
    from research.blind_corner_queries import Queries
    folder=out/'inspections'/str(time.time_ns());folder.mkdir(parents=True,exist_ok=False)
    stage=static_stage(app,cfg); inventory=colliders(stage)
    previous=read('outputs/hospital-native-light-audit-20261010-01/collision_geometry.json')
    write_json(folder/'collision_geometry.json',inventory)
    if inventory!=previous:
        old={r['path']:r for r in previous};new={r['path']:r for r in inventory}
        write_json(folder/'inventory-difference.json',{'old_count':len(old),'new_count':len(new),
            'removed':sorted(set(old)-set(new)),'added':sorted(set(new)-set(old)),
            'changed':[{'path':p,'old':old[p],'new':new[p]} for p in old.keys()&new.keys() if old[p]!=new[p]]})
        raise ValueError('Original Hospital collider inventory changed; inspect saved difference')
    scene=source_bytes(cfg['scene']['usd'])
    if scene['sha256']!=cfg['hospital_lighting_source_sha256']: raise ValueError('Hospital asset changed')
    write_json(folder/'scene-source.json',scene)
    cache=UsdGeom.BBoxCache(0,['default','render','proxy'],useExtentsHint=False); carts=[]; identities=[]; assets={}
    for p in stage.GetPrimAtPath('/Root').GetChildren():
        refs=p.GetMetadata('references')
        if not refs: continue
        for ref in refs.GetAppliedItems():
            for spec in p.GetPrimStack():
                if not spec.HasInfo('references'): continue
                authored=spec.GetInfo('references').GetAppliedItems()
                if ref not in authored: continue
                url=Sdf.ComputeAssetPathRelativeToLayer(spec.layer,ref.assetPath)
                identities.append({'prim':str(p.GetPath()),'reference_asset':url})
                # Verified asset reference identity, not prim-name keyword alone.
                name=Path(ref.assetPath).stem
                if not (name.startswith('SM_SupplyCart_') or name.startswith('SM_GasCart_') or name.startswith('SM_Cart_')): continue
                if url not in assets: assets[url]=source_bytes(url)
                box=cache.ComputeWorldBound(p).ComputeAlignedRange()
                carts.append({'prim':str(p.GetPath()),'asset':assets[url],
                    'world_matrix_row_vector':np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)).tolist(),
                    'world_aabb_min':list(box.GetMin()),'world_aabb_max':list(box.GetMax()),
                    'colliders':[r for r in inventory if r['path'].startswith(str(p.GetPath())+'/')],
                    'footprint_type':'Conservative world XY AABB of actual referenced visual geometry; collider bounds retained separately',
                    'original_root_instance':True})
                break
    write_json(folder/'native_cart_inventory.json',carts);write_json(folder/'root_asset_identities.json',identities)
    gate=cfg['spatial_turn_gate']; south=gate['source_south_corner'];north=gate['source_north_wall']
    for row in [south,north]:
        if row not in inventory: raise ValueError('Structural gate source mismatch')
    sy=south['world_aabb_max'][1]; ny=north['world_aabb_min'][1];center=(sy+ny)/2
    x0=gate['exit_center_x_max_m'];xmin=cfg['scene']['goal'][0]+2.
    q=Queries(stage)
    xs=sorted(set(np.linspace(xmin,x0,int(np.ceil((x0-xmin)/.1))+1).tolist()+[x0-s for s in POLICY['grid_downstream_m']]),reverse=True)
    sections=[]
    for x in xs:
        hits=[{'z_m':z,'direction_world_y':sign,'hit':ray(q,[x,center,z],[0,sign,0],6.)} for z in [.35,.70] for sign in [-1,1]]
        valid=all(h['hit'] and any(w in h['hit']['prim'] for w in ['Wall','DoorFrame','Trim']) and
            abs(h['hit']['position'][1]-(sy if h['direction_world_y']==-1 else ny))<.03 for h in hits)
        sections.append({'x_m':x,'center_y_m':center,'valid':valid,'queries':hits,
                         'invalid_reason':None if valid else 'Missing/obstructed/nonparallel lateral structural face'})
    geometry={'axes':'World metres +Z up; yaw CCW; west forward=(-1,0), left=(0,-1)',
        'origin':'World Hospital origin; travel s=entrance_x-x, left=center_y-y',
        'source_south':south,'source_north':north,'south_y_m':sy,'north_y_m':ny,'center_y_m':center,
        'width_m':ny-sy,'entrance_x_m':x0,'goal_region_east_extent_x_m':xmin,
        'centerline_derivation':'Midpoint of independently audited parallel structural inner AABB faces; ray-confirmed per-section, not fitted to robot poses',
        'cross_section_validity':'Nearest original collider at z=.35 and .70 on both +/-Y rays must be structural and within .03m of source face; invalid sections retained',
        'cross_sections':sections,'floor_prim':q.floor_prim,'original_inventory_equal':True}
    write_json(folder/'corridor_geometry.json',geometry)
    write_json(folder/'inspection-complete.json',{'status':'PASS','cart_instances':len(carts),'valid_cross_sections':sum(s['valid'] for s in sections),'total_cross_sections':len(sections),'model_calls':0,'physics_steps':0})
    (out/'stage-a').mkdir(exist_ok=True)
    for f in folder.glob('*.json'): write_json(out/'stage-a'/f.name,read(f))


def check_frozen(out):
    plan=read(out/'stage-b/frozen-plan.json'); receipt=read(out/'stage-b/freeze-receipt.json')
    if digest(out/'stage-b/frozen-plan.json')!=receipt['plan_sha256']: raise ValueError('Frozen plan changed')
    for p,h in receipt['executed_source_sha256'].items():
        if digest(p)!=h: raise ValueError('Frozen source changed: '+p)
    return plan


def physical(app,cfg,out):
    from research.blind_corner_queries import Queries
    from research.human_actor import author_human
    from pxr import UsdGeom
    plan=check_frozen(out);folder=out/'stage-b/physical';folder.mkdir(exist_ok=False)
    stage=static_stage(app,cfg);before=colliders(stage)
    if before!=read(out/'stage-a/collision_geometry.json'): raise ValueError('Hospital changed')
    q=Queries(stage); refs=[load_reference(p) for p in plan['references']]; results=[]
    human_asset=source_bytes(plan['human']['asset']);write_json(folder/'human-source.json',human_asset)
    relevant=[r for r in before if r['enabled'] and r['path']!=q.floor_prim and r['world_aabb_min'] is not None
              and r['world_aabb_min'][2]<.70+RADIUS+MARGIN and r['world_aabb_max'][2]>.70-RADIUS-MARGIN]
    for c in plan['candidates']:
        h={**plan['human'],'start_position':c['position_world_xyz'],'yaw':c['yaw_rad']}
        translate,info=author_human(stage,h,physical=False)
        for key in ['asset','source_units','source_up_axis','height_m','source_bounds','visual_translation_m']:
            if info[key]!=plan['human_asset_audit'][key]: raise ValueError('Audited human asset changed: '+key)
        loaded=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(h['prim'])).ComputeLocalToWorldTransform(0))
        if not np.allclose(loaded[3,:3],c['position_world_xyz'],atol=1e-9,rtol=0): raise ValueError('Human transform mismatch')
        place=q.human(c['position_world_xyz'],HUMAN_RADIUS,c['yaw_rad'])
        floor=ray(q,[*c['position_world_xyz'][:2],.3],[0,0,-1],.6)
        initial_distance=float(np.linalg.norm(refs[0]['xy'][0]-c['position_world_xyz'][:2]))
        placement=bool(c['station_valid'] and place['floor_ok'] and not place['overlap_paths'] and initial_distance>RADIUS+HUMAN_RADIUS)
        relevance={r['path']:point_path(r['xy'],c['position_world_xyz'][:2],RADIUS+HUMAN_RADIUS,r['arc']) for r in refs}
        sides={}
        for side in ['LEFT','RIGHT']:
            try: vertices=bypass_vertices(refs[0],c,side)
            except ValueError as exc:
                sides[side]={'pass':False,'reason':str(exc),'vertices_xy':[]};continue
            samples=sample_path(vertices);trace=q.robot_path(samples,RADIUS+MARGIN)
            for row in trace:
                xy=np.array(row['xy']);probes=[xy,xy+[RADIUS,0],xy+[-RADIUS,0],xy+[0,RADIUS],xy+[0,-RADIUS]]
                row['floor_probes']=[{'xy':v.tolist(),'valid':q.floor(v)[0],'prim':q.floor(v)[1]} for v in probes]
                row['floor']=bool(row['floor'] and all(v['valid'] for v in row['floor_probes']))
            human=point_path(vertices,c['position_world_xyz'][:2])
            boxes=[]
            # Broad phase excludes boxes farther than the current corridor neighborhood.
            for r in relevant:
                lo=np.array(r['world_aabb_min']);hi=np.array(r['world_aabb_max'])
                if np.any(hi[:2]<vertices.min(0)-2.) or np.any(lo[:2]>vertices.max(0)+2.): continue
                d=rect_path(vertices,lo,hi,RADIUS)
                boxes.append({**d,'prim':r['path']})
            minimum=min(boxes,key=lambda d:d['clearance_m']) if boxes else None
            failures=[{'index':i,**row} for i,row in enumerate(trace) if not row['floor'] or row['overlap']]
            passed=bool(human['distance_m']>=SEPARATION and minimum and minimum['clearance_m']>=MARGIN and not failures)
            sides[side]={'pass':passed,'vertices_xy':vertices.tolist(),'sample_count':len(samples),
                'max_sample_spacing_m':float(np.linalg.norm(np.diff(samples,axis=0),axis=1).max()),
                'minimum_human_surface_clearance_m':human['distance_m']-RADIUS-HUMAN_RADIUS,
                'minimum_center_separation_m':human['distance_m'],'minimum_structural_clearance':minimum,
                'continuous_aabb_checks':boxes,'sample_queries':trace,'failure_samples':failures,
                'downstream_reconnect_progress_m':2.5,'downstream_after_exclusion_m':2.5-SEPARATION,
                'connected':True,'note':'Constructed geometric candidate only; never model-predicted or executed'}
        record={**c,'physical_pass':placement,'placement':place,'floor_query':floor,'human_asset':info,
            'loaded_world_matrix_row_vector':loaded.tolist(),'initial_robot_distance_m':initial_distance,
            'relevance':relevance,'baseline_conflict':relevance[PRIMARY]['conflict'],
            'obstructed_primary_plus_six':sum(relevance[r]['conflict'] for r in [PRIMARY,*plan['references'][2:]]),
            'sides':sides,'eligible_for_rgb':bool(placement and relevance[PRIMARY]['conflict'] and any(v['pass'] for v in sides.values()))}
        write_json(folder/(c['id']+'.json'),record);results.append(record)
        stage.RemovePrim(h['prim'])
        print('PHYSICAL',c['id'],placement,record['baseline_conflict'],[s for s in sides if sides[s]['pass']],flush=True)
    if colliders(stage)!=before: raise ValueError('Original collider mutation')
    write_json(folder/'summary.json',{'status':'PASS','records':[r['id'] for r in results],
        'physical_pass':sum(r['physical_pass'] for r in results),'eligible_for_rgb':sum(r['eligible_for_rgb'] for r in results),
        'original_colliders_unchanged':True,'model_calls':0,'physics_steps':0})


def render_candidates(app,cfg,out):
    import omni.timeline
    from PIL import Image
    from pxr import UsdGeom
    from isaacsim.core.utils.semantics import add_update_semantics
    from research.nova_render_saved import SavedRenderer
    from research.human_actor import author_human
    from research.hospital_lights import apply_hospital_lights
    from research.nova_occlusion import segmentation_stats
    from research.nova_bright import image_stats,bright_pass
    from research.control import rotation_wxyz
    from research.blind_corner_queries import Queries
    plan=check_frozen(out);folder=out/'stage-b/renders';folder.mkdir(exist_ok=False)
    render=SavedRenderer(app,cfg);lighting=apply_hospital_lights(render.stage,cfg['hospital_lighting'])
    write_json(folder/'lighting-readback.json',lighting)
    if len(cfg['hospital_lighting']['fallback_fixture_lights'])!=153: raise ValueError('Wrong lighting')
    before=colliders(render.stage)
    if before!=read(out/'stage-a/collision_geometry.json'): raise ValueError('Geometry changed')
    semantic=render.rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
    semantic.attach([render.product]);ref=load_reference(PRIMARY)
    extrinsic=np.array(read(Path(PRIMARY)/'robot_asset.json')['camera_body_transform'])
    camera=UsdGeom.Camera(render.stage.GetPrimAtPath(cfg['camera']['prim']))
    camera_info={a.GetName():str(a.Get()) for a in camera.GetPrim().GetAttributes()}
    original=next(c for c in read(Path(PRIMARY)/'robot_asset.json')['cameras'] if c['path']==cfg['camera']['prim'])
    actual={'focal_length':camera.GetFocalLengthAttr().Get(),'horizontal_aperture':camera.GetHorizontalApertureAttr().Get(),
            'vertical_aperture':camera.GetVerticalApertureAttr().Get(),'clipping_range':list(camera.GetClippingRangeAttr().Get())}
    if any(not np.allclose(actual[k],original[k],rtol=0,atol=1e-9) for k in actual): raise ValueError('Original camera intrinsics changed')
    q=Queries(render.stage)
    write_json(folder/'camera-attributes.json',camera_info)
    for c in plan['candidates']:
        physical=read(out/'stage-b/physical'/f"{c['id']}.json")
        if not physical['eligible_for_rgb']: continue
        dest=folder/c['id'];dest.mkdir(exist_ok=False)
        h={**plan['human'],'start_position':c['position_world_xyz'],'yaw':c['yaw_rad']}
        _,asset=author_human(render.stage,h,physical=False)
        add_update_semantics(render.stage.GetPrimAtPath(h['prim']),'preflight_human')
        write_json(dest/'human-asset.json',asset)
        conflict=physical['relevance'][PRIMARY]; first=conflict['first_conflict']
        conflict_time=float(ref['times'][first['segment']]+first['fraction']*np.diff(ref['times'])[first['segment']])
        rows=[]
        for e in ref['events']:
            obs=e['observation'];t=obs['sim_time']
            if t>=conflict_time: break
            matrix=np.array(render.pose(e));body=np.eye(4);body[:3,:3]=rotation_wxyz(obs['quaternion_wxyz']).T;body[3,:3]=obs['position']
            expected=extrinsic@body
            camera_error=float(np.max(np.abs(matrix-expected)))
            if camera_error>1e-5: raise ValueError('Exact saved camera transform failed')
            human_matrix=np.array(UsdGeom.Xformable(render.stage.GetPrimAtPath(h['prim'])).ComputeLocalToWorldTransform(0))
            if not np.allclose(human_matrix,physical['loaded_world_matrix_row_vector'],atol=1e-9,rtol=0): raise ValueError('Human moved')
            rgb=render.capture();packed=semantic.get_data();data=np.asarray(packed['data']).copy();labels=packed['info']['idToLabels']
            stats,mask=segmentation_stats(data,labels,'preflight_human');rid=e['request_id'];stem=f'C{rid:02d}'
            Image.fromarray(rgb).save(dest/(stem+'-rgb.png'));Image.fromarray(mask.astype(np.uint8)*255).save(dest/(stem+'-mask.png'))
            with (dest/(stem+'-semantic.npy')).open('xb') as f: np.save(f,data,allow_pickle=False)
            write_json(dest/(stem+'-labels.json'),labels)
            brightness=image_stats(rgb);box=stats['human_bbox_if_visible']
            clear=bool(stats['human_visible_fraction']>=.002 and box and box[2]-box[0]>=20 and box[3]-box[1]>=80)
            delta=np.array(c['position_world_xyz'])-matrix[3,:3]
            local=np.r_[c['position_world_xyz'],1.]@np.linalg.inv(matrix)
            occlusion=[]
            for z in [.5,1.,1.5]:
                target=np.array([*c['position_world_xyz'][:2],z]);direction=target-matrix[3,:3];length=np.linalg.norm(direction)
                occlusion.append({'target_world_xyz':target.tolist(),'original_geometry_hit':ray(q,matrix[3,:3],direction/length,length)})
            row={'request_id':rid,'observation_sim_time':t,'observation_wall_time_ns':obs['wall_time_ns'],
                'saved_observation':obs,'camera_world_matrix_row_vector':matrix.tolist(),'camera_error_max_abs':camera_error,
                'remaining_along_track_m':conflict['closest_arc_m']-float(np.interp(t,ref['times'],ref['arc'])),
                'camera_to_human_base_distance_m':float(np.linalg.norm(delta)),
                'relative_camera_bearing_rad':float(np.arctan2(local[0],-local[2])),
                'in_straight_corridor':obs['position'][0]<=plan['geometry']['entrance_x_m'],
                'before_conflict':t<conflict_time,'state':'CLEAR' if clear else ('MARGINAL' if stats['human_visible_pixel_count'] else 'HIDDEN'),
                **stats,'brightness':brightness,'brightness_pass':bright_pass(brightness,cfg['bright_validation']),
                'render_valid':True,'timeline_playing':render.timeline.is_playing(),'timeline_time':render.timeline.get_current_time(),
                'capture':'Explicit completion, rt_subframes=4, delta_time=0, pause_timeline=True; no overlay',
                'occlusion_rays':occlusion,'source_rgb':str(Path(PRIMARY)/e['rgb_observation_reference']),
                'source_rgb_sha256':digest(Path(PRIMARY)/e['rgb_observation_reference']),
                'occlusion_evidence':'Actual semantic mask determines visibility; original-geometry rays to three torso heights provide explanatory evidence only'}
            row['early_clear_pass']=early_clear(row);rows.append(row)
            write_json(dest/(stem+'-observation.json'),row)
        write_json(dest/'visibility.json',rows)
        render.stage.RemovePrim(h['prim'])
        print('RGB',c['id'],'frames',len(rows),'early',sum(r['early_clear_pass'] for r in rows),flush=True)
    if colliders(render.stage)!=before: raise ValueError('Original colliders changed in render')
    write_json(folder/'complete.json',{'status':'PASS','model_calls':0,'checkpoint_loads':0,'physics_steps':0,'navigation_trials':0,'original_colliders_unchanged':True})


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['inspect','physical','render'],required=True)
    p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir)
    if not (out/'prepare-complete.json').exists(): raise ValueError('Reference gate required')
    startups=list((out/'attempts').glob('startup-*/stdout.log'))
    if not any('ISAAC6_SIMULATION_APP_READY' in f.read_text() and (f.parent/'end.json').exists() and read(f.parent/'end.json')['exit_code']==0 for f in startups):
        raise ValueError('Simulator-only startup gate required')
    cfg=read(Path(PRIMARY)/'runtime-config.json')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        {'inspect':inspect,'physical':physical,'render':render_candidates}[a.phase](app,cfg,out)
        import omni.timeline
        if omni.timeline.get_timeline_interface().is_playing(): raise RuntimeError('Timeline unexpectedly playing')
        print('MODEL_FREE_PREFLIGHT_PASS',a.phase,flush=True);code=0
    except Exception as exc:
        traceback.print_exc();print('MODEL_FREE_PREFLIGHT_FAIL',repr(exc),flush=True)
    finally: app.close(exit_code=code)


if __name__=='__main__': main()
