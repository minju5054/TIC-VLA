"""Model-free Hospital geometry and all-observation silhouette search.

Each candidate is a separate stationary scene variant, not a moving/hidden actor
in a navigation episode. This executable cannot perform a model run.
"""
import argparse
import json
import sys
import traceback
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json, provenance, clocks
from research.hospital_episode import digest
from research.analyze_nova_dynamic_handoff import load_saved
from research.reveal_window import coarse_positions, path_distance, temporal_metrics, evidence_hashes, verify_hashes, state, validate_destination


def prepare(config_path, out):
    cfg = json.loads(Path(config_path).read_text())
    validate_destination(out,[cfg[k] for k in ['baseline_run','baseline_analysis','previous_geometry','previous_render']])
    run = load_saved(cfg['baseline_run'])
    if len(run['events']) != 48 or not run['complete']:
        raise ValueError('Require the complete 48-request source')
    analysis = json.loads((Path(cfg['baseline_analysis'])/'summary.json').read_text())
    if analysis['baseline_spatial_gate_pass'] is not True:
        raise ValueError('Baseline prerequisite failed')
    out.mkdir(parents=True, exist_ok=False)
    hashes = evidence_hashes([cfg['baseline_run'], cfg['baseline_analysis'], cfg['lighting_config'],
                              cfg['collision_inventory'], cfg['previous_geometry'], cfg['previous_render']])
    write_json(out/'source_evidence_sha256.json', hashes)
    write_json(out/'preregistered.json', {'config': cfg, 'config_sha256': digest(config_path),
               'registration': clocks(), 'code': provenance(ROOT), 'task': 'MODEL-FREE REVEAL-WINDOW SEARCH ONLY'})
    write_json(out/'metadata.json', {'baseline_run': str(run['path']), 'config': cfg,
               'model_calls': 0, 'navigation_physics_reexecuted': False,
               'source_evidence_manifest_sha256': digest(out/'source_evidence_sha256.json'), **provenance(ROOT)})
    return cfg, run


def geometry(app, cfg, run, out):
    import carb
    from omni.physx import get_physx_scene_query_interface, get_physx_interface
    from pxr import UsdPhysics, Usd
    from research.nova_render_saved import SavedRenderer
    from research.hospital_lights import apply_hospital_lights
    render = SavedRenderer(app, run['cfg'])
    write_json(out/'applied_lighting.json', apply_hospital_lights(render.stage, run['cfg']['hospital_lighting']))
    if not any(p.IsA(UsdPhysics.Scene) for p in render.stage.Traverse()):
        UsdPhysics.Scene.Define(render.stage, '/World/QueryPhysicsScene')
    for p in Usd.PrimRange(render.stage.GetPrimAtPath(run['cfg']['robot']['prim'])):
        if p.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
    # Load static colliders, without start_simulation, simulate, reset or play.
    get_physx_interface().force_load_physics_from_usd()
    query = get_physx_scene_query_interface()
    asset = json.loads((Path(cfg['previous_render'])/'human_asset.json').read_text())
    low, high = np.array(asset['source_bounds'])*asset['source_units']
    half = (high-low)/2
    if asset['source_up_axis'] != 'Z' or cfg['human_yaw_rad'] != 0:
        raise ValueError('Registered visual-bounds query requires Z up and fixed yaw zero')

    def floor(x, y):
        hit = query.raycast_closest(carb.Float3(float(x), float(y), .30), carb.Float3(0,0,-1), .6)
        ok = bool(hit['hit'] and str(hit.get('collision','')).startswith('/Root/')
                  and abs(hit['position'][2]) < cfg['floor_tolerance_m'] and hit['normal'][2] > .9)
        return ok, str(hit.get('collision',''))

    known_floor_ok, known_floor_prim = floor(*run['xy'][0])
    if not known_floor_ok:
        raise RuntimeError('Static-query initialization failed known baseline floor probe')

    def overlaps(point, radius=None, extent=None):
        hits = []
        def callback(hit):
            if str(hit.collision).startswith('/Root/'):
                # Floor support is tested separately. A sphere enlarged by the
                # horizontal safety margin may graze the ground at z=.65;
                # support contact must not become a lateral bypass obstacle.
                if str(hit.collision) != known_floor_prim:
                    hits.append(str(hit.collision))
            return True
        if extent is not None:
            query.overlap_box(carb.Float3(*map(float,extent)), carb.Float3(*map(float,point)), carb.Float4(0,0,0,1), callback, False)
        else:
            query.overlap_sphere(float(radius), carb.Float3(*map(float,point)), callback, False)
        return sorted(set(hits))

    def space(pos):
        ok, prim = floor(*pos[:2])
        capsule = sorted(set(p for z in [.30,.65,1.,1.35,1.60] for p in overlaps([*pos[:2],z], cfg['human_radius_m']))) if ok else []
        # Enclosing upright visual box catches extended arms. Exclude bottom
        # 2cm floor contact; this is conservative and may reject free mesh space.
        visual = overlaps([*pos[:2], half[2]+.01], extent=[half[0],half[1],half[2]-.01]) if ok else []
        return {'floor_ok':ok, 'floor_prim':prim, 'capsule_overlap_paths':capsule,
                'visual_bounds_overlap_paths':visual, 'overlap_paths':sorted(set(capsule+visual))}

    def bypass(pos):
        i = int(np.linalg.norm(run['xy']-pos[:2],axis=1).argmin())
        a,b = max(0,i-15), min(len(run['xy'])-1,i+15)
        direction = run['xy'][b]-run['xy'][a]
        if np.linalg.norm(direction)<1e-8:
            direction=np.array([0.,1.])
        direction /= np.linalg.norm(direction)
        normal = np.array([-direction[1], direction[0]])
        attempts = []
        r = cfg['robot_radius_m']+cfg['bypass_safety_margin_m']
        for sign in [-1,1]:
            for offset in cfg['bypass_offsets_m']:
                center = np.array(pos[:2])+sign*offset*normal
                xy = center+np.arange(-1.,1.001,cfg['bypass_sample_step_m'])[:,None]*direction
                clear = offset-cfg['robot_radius_m']-cfg['human_radius_m']
                valid = clear >= cfg['bypass_safety_margin_m']
                valid = valid and all(floor(*p)[0] and not overlaps([*p,.65],r) for p in xy)
                attempts.append({'side':sign,'offset_m':offset,'pass':bool(valid)})
                if valid:
                    return {'pass':True, 'clearance_m':float(clear), 'centerline_xy':xy.tolist(),
                            'safety_margin_m':cfg['bypass_safety_margin_m'], 'attempts':attempts,
                            'radius_including_margin_m':r, 'nearest_path_tick_index':i}
        return {'pass':False, 'clearance_m':None, 'centerline_xy':[], 'attempts':attempts}

    inventory = json.loads(Path(cfg['collision_inventory']).read_text())
    edges=[]
    for c in inventory:
        if c['world_aabb_min'] is None or c['world_aabb_max'] is None:
            continue
        lo,hi=np.array(c['world_aabb_min']),np.array(c['world_aabb_max'])
        if c['enabled'] and lo[2]<1.8 and hi[2]>1.5 and min(hi[:2]-lo[:2])<.7:
            edges.extend([[x,y] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]]])
    edges=np.asarray(edges)
    positions=coarse_positions(run['xy'],cfg['coarse_step_m'],cfg['route_band_m'])
    previous=json.loads((Path(cfg['previous_geometry'])/'geometry_candidates.json').read_text())
    previous_set={tuple(c['position']) for c in previous}
    cache={}
    def inspect(pos):
        key=tuple(pos)
        if key not in cache:
            g=space(pos)
            conflicts=[]
            for rid in range(2,49):
                metric=temporal_metrics(run,rid,pos,cfg)
                if metric['conflict'] is not None:
                    conflicts.append(rid)
            cache[key]={'position':pos,'yaw':cfg['human_yaw_rad'],'human_radius_m':cfg['human_radius_m'],
                        'geometry':g,'potential_conflict_fresh_ids':conflicts}
        return cache[key]
    refine=[]
    for pos in positions:
        c=inspect(pos)
        if (c['geometry']['floor_ok'] and not c['geometry']['overlap_paths'] and c['potential_conflict_fresh_ids']
                and np.linalg.norm(edges-np.array(pos[:2]),axis=1).min()<=cfg['refinement_edge_distance_m']):
            for dx in [-.25,.25]:
                for dy in [-.25,.25]:
                    p=[pos[0]+dx,pos[1]+dy,0.]
                    if path_distance(p,run['xy'])<=cfg['route_band_m']:
                        refine.append(p)
    all_positions=sorted(set(map(tuple,positions+refine))|previous_set)
    matrices=[render.pose(e) for e in run['events']]
    write_json(out/'camera_matrices.json',matrices)

    def occluders(pos):
        result=[]
        for m in np.asarray(matrices):
            origin=m[3,:3];d=np.array(pos)+[0,0,.9]-origin;length=np.linalg.norm(d);hits=[]
            def callback(hit):
                if str(hit.collision).startswith('/Root/'):
                    hits.append({'prim':str(hit.collision),'distance_m':float(hit.distance)})
                return True
            query.raycast_all(carb.Float3(*map(float,origin)),carb.Float3(*map(float,d/length)),float(length),callback,True)
            result.append(min(hits,key=lambda h:h['distance_m']) if hits else None)
        return result

    candidates=[]
    for i,key in enumerate(all_positions,1):
        c=inspect(list(key));g=c['geometry'];valid=g['floor_ok'] and not g['overlap_paths']
        bp=bypass(list(key)) if valid else {'pass':False,'clearance_m':None,'centerline_xy':[],'not_evaluated':True}
        eligible=valid and bp['pass'] and (bool(c['potential_conflict_fresh_ids']) or key in previous_set)
        c.update(id=i,previous_region=key in previous_set,bypass=bp,render_candidate=bool(eligible))
        c['center_ray_occluders']=occluders(list(key)) if eligible else []
        candidates.append(c)
    write_json(out/'search_space.json',{'coarse_count':len(positions),'refinement_added_count':len(set(map(tuple,refine))-set(map(tuple,positions))),
               'previous_region_count':len(previous_set),'total_candidate_count':len(candidates),
               'render_eligible_count':sum(c['render_candidate'] for c in candidates),
               'route_xy_bounds':[run['xy'].min(0).tolist(),run['xy'].max(0).tolist()],
               'positions':all_positions,'policy':cfg,'static_query_known_floor_pass':True,
               'physics_steps':0,'timeline_playing':render.timeline.is_playing()})
    write_json(out/'geometry_candidates.json',candidates)
    write_json(out/'human_asset.json',asset)
    print('REVEAL_GEOMETRY_COMPLETE',len(candidates),sum(c['render_candidate'] for c in candidates),flush=True)


def render_candidates(app,cfg,run,out,part='main',shard_index=0,shard_count=1):
    import omni.replicator.core as rep
    from isaacsim.core.utils.semantics import add_labels
    from research.nova_render_saved import SavedRenderer
    from research.hospital_lights import apply_hospital_lights
    from research.human_actor import author_human
    from research.nova_occlusion import segmentation_stats
    from research.nova_bright import image_stats, rgb_difference
    from PIL import Image
    render=SavedRenderer(app,run['cfg'])
    batch=out/('render_batch_'+part);batch.mkdir(exist_ok=False)
    write_json(batch/'lighting.json',apply_hospital_lights(render.stage,run['cfg']['hospital_lighting']))
    write_json(batch/'execution.json',{'part':part,'shard_index':shard_index,'shard_count':shard_count,
               'lossless_png_compression_level':1,'storage_threads':4,**provenance(ROOT)})
    expected=json.loads((out/'camera_matrices.json').read_text())
    def pose(e):
        m=render.pose(e)
        np.testing.assert_allclose(m,expected[e['request_id']-1],atol=1e-6,rtol=0)
    absent=out/'absent'
    parent=json.loads((out/'metadata.json').read_text()).get('parent_search_dir')
    if not absent.exists() and parent:
        absent=Path(parent)/'absent'
    if not absent.exists():
        if shard_count!=1:raise ValueError('Create absent reference with a single renderer before sharding')
        absent.mkdir(exist_ok=False)
        for e in run['events']:
            pose(e);Image.fromarray(render.capture()).save(absent/f"C{e['request_id']:02d}.png",compress_level=1)
    references=[np.asarray(Image.open(absent/f'C{rid:02d}.png')).copy() for rid in range(1,49)]
    segmentation=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
    segmentation.attach([render.product])
    asset=json.loads((out/'human_asset.json').read_text())
    human={'prim':'/World/StationarySearchHuman','asset':asset['asset'],'radius_m':cfg['human_radius_m'],
           'capsule_cylinder_height_m':1.2,'yaw':cfg['human_yaw_rad']}
    candidates=json.loads((out/'geometry_candidates.json').read_text())
    eligible=[c for c in candidates if c['render_candidate']]
    completed={int(p.parent.name.split('_')[-1]):str(p) for p in out.glob('**/candidate_*/observations.json')}
    reuse_path=out/'reused_candidates.json'
    if reuse_path.exists():
        completed.update({int(k):v['observations'] for k,v in json.loads(reuse_path.read_text()).items()})
    reused_hashes=evidence_hashes([Path(p).parent for p in completed.values()]+[absent])
    write_json(batch/'reused_evidence_sha256.json',reused_hashes)
    def store_frame(folder,rid,rgb,stats,mask):
        Image.fromarray(rgb).save(folder/f'C{rid:02d}.png',compress_level=1)
        Image.fromarray(mask.astype(np.uint8)*255).save(folder/f'C{rid:02d}.mask.png',compress_level=1)
        return {'request_id':rid,**stats,'state':state(stats,cfg['visibility']),
                'brightness':image_stats(rgb),'paired_rgb_diagnostic':rgb_difference(references[rid-1],rgb),
                'rgb':str(folder/f'C{rid:02d}.png'),'mask':str(folder/f'C{rid:02d}.mask.png')}
    pool=ThreadPoolExecutor(max_workers=4)
    manifest={}
    for n,c in enumerate(eligible):
        if n%shard_count!=shard_index:continue
        if c['id'] in completed:
            manifest[c['id']]=completed[c['id']];continue
        if c.get('prefilter_pass') is False:
            raise ValueError('A rejected prefilter candidate cannot be newly rendered')
        started=time.monotonic();folder=batch/f"candidate_{c['id']:04d}";folder.mkdir(parents=True,exist_ok=False)
        author_human(render.stage,{**human,'start_position':c['position']},asset,physical=False)
        add_labels(render.stage.GetPrimAtPath(human['prim']),['occluded_human'])
        futures=[]
        for e in run['events']:
            rid=e['request_id'];pose(e);rgb=render.capture();packed=segmentation.get_data()
            stats,mask=segmentation_stats(packed['data'],packed['info']['idToLabels'])
            futures.append(pool.submit(store_frame,folder,rid,rgb,stats,mask))
            if len(futures)>8:futures[-9].result()  # Bounded RGB memory.
        rows=[f.result() for f in futures]
        write_json(folder/'observations.json',rows)
        render.stage.RemovePrim(human['prim'])
        first=next((r['request_id'] for r in rows if r['state']=='CLEAR'),None)
        manifest[c['id']]=str(folder/'observations.json')
        print('REVEAL_RENDER',n+1,'/',len(eligible),'candidate',c['id'],'first_clear',first,'seconds',round(time.monotonic()-started,2),flush=True)
    pool.shutdown();verify_hashes(reused_hashes)
    write_json(batch/'complete.json',{'candidate_observations':manifest,'model_calls':0,'navigation_physics_reexecuted':False})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='configs/research/reveal_window_search.json')
    p.add_argument('--output-dir',required=True)
    p.add_argument('--phase',choices=['geometry','render'],required=True)
    p.add_argument('--render-part',default='main')
    p.add_argument('--shard-index',type=int,default=0);p.add_argument('--shard-count',type=int,default=1)
    a=p.parse_args();out=Path(a.output_dir)
    if a.phase=='geometry':
        cfg,run=prepare(a.config,out)
    else:
        frozen=json.loads((out/'preregistered.json').read_text())
        if 'prefilter_config_sha256' not in frozen:
            raise ValueError('Cheap prefilter required before rendering. Preserve this output and run prefilter_reveal_window.py into a fresh directory.')
        if digest(a.config)!=frozen['config_sha256']:
            raise ValueError('Preregistered config changed')
        cfg=frozen['config'];run=load_saved(cfg['baseline_run'])
        verify_hashes(json.loads((out/'source_evidence_sha256.json').read_text()))
        reuse_path=out/'reused_candidates.json'
        if reuse_path.exists():
            candidates=json.loads((out/'geometry_candidates.json').read_text())
            reused=json.loads(reuse_path.read_text())
            eligible=[c for c in candidates if c['render_candidate']]
            if all(str(c['id']) in reused for c in eligible):
                batch=out/('render_batch_'+a.render_part);batch.mkdir(exist_ok=False)
                write_json(batch/'complete.json',{'candidate_observations':{c['id']:reused[str(c['id'])]['observations'] for c in eligible},
                           'model_calls':0,'navigation_physics_reexecuted':False,'new_rgb_renders':0,
                           'source':'Hash-validated completed paused-search evidence; no SimulationApp started'})
                write_json(out/('render_'+a.render_part+'_immutable.json'),{'source_unchanged':True,'model_calls':0})
                print('RENDER_REUSE_ONLY',len(eligible),'new renders 0',flush=True);return
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080})
    code=1
    try:
        if a.phase=='geometry':geometry(app,cfg,run,out)
        else:render_candidates(app,cfg,run,out,a.render_part,a.shard_index,a.shard_count)
        verify_hashes(json.loads((out/'source_evidence_sha256.json').read_text()))
        receipt=a.phase if a.phase=='geometry' else 'render_'+a.render_part
        write_json(out/(receipt+'_immutable.json'),{'source_unchanged':True,'model_calls':0})
        code=0
    except Exception as exc:
        traceback.print_exc();write_json(out/(a.phase+'_'+a.render_part+'_failure.json'),{'error':repr(exc),'model_calls':0})
    finally:
        app.close(exit_code=code)


if __name__=='__main__':
    main()
