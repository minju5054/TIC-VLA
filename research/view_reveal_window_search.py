"""Interactive stationary alternatives with saved Nova poses; no navigation execution."""
import argparse
import asyncio
import hashlib
import json
import shutil
import subprocess
import sys
import time
import traceback
from types import SimpleNamespace
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.reveal_window import CandidateNavigation, evidence_hashes, verify_hashes, validate_destination, state as visibility_state
from research.analyze_nova_dynamic_handoff import load_saved
from research.records import write_json, provenance


def restart_state(times):
    if float(times[0]) != 0.:raise ValueError('Saved replay must begin at simulation t=0')
    return {'time':0.,'playing':False}


def replay_frame_times(times, fps=10):
    restart_state(times)
    frames=np.arange(0.,float(times[-1]),1./fps).tolist()
    if not frames or times[-1]-frames[-1]>1e-8:frames.append(float(times[-1]))
    else:frames[-1]=float(times[-1])
    return frames


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--search-dir',required=True);p.add_argument('--output-dir',required=True)
    p.add_argument('--self-test',action='store_true')
    p.add_argument('--export-full',action='store_true',help='Export complete saved drive at 10 fps; installed ffmpeg only')
    a=p.parse_args()
    source=Path(a.search_dir).resolve();out=Path(a.output_dir).resolve()
    if out==source or source in out.parents or out.exists():raise ValueError('Fresh output outside source required')
    cfg=json.loads((source/'preregistered.json').read_text())['config']
    validate_destination(out,[source,cfg['baseline_run'],cfg['baseline_analysis']])
    run=load_saved(cfg['baseline_run']);summary=json.loads((source/'summary.json').read_text())
    candidates=json.loads((source/'candidates.json').read_text());nav=CandidateNavigation(candidates)
    is_cart=summary.get('obstacle_type')=='cart'
    asset=None if is_cart else json.loads((source/'human_asset.json').read_text())
    hashes=evidence_hashes([source,run['path']])
    hashes.update(json.loads((source/'source_evidence_sha256.json').read_text()))
    verify_hashes(hashes)
    out.mkdir(parents=True,exist_ok=False)
    write_json(out/'metadata.json',{'mode':'saved_cart_run_GUI' if is_cart else 'saved_baseline_stationary_alternative_GUI','search':str(source),
               'model_calls':0,'navigation_physics_reexecuted':False,'source_evidence_sha256':hashes,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RayTracedLighting','width':1600,'height':1000})
    try:
        import omni.ui as ui
        import omni.timeline
        import omni.usd
        from pxr import Gf,UsdGeom,Usd,UsdPhysics
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        from research.hospital_lights import apply_hospital_lights
        from research.human_actor import author_human
        from PIL import Image,ImageDraw,ImageFont
        # Viewport display only: no Hawk RGB/semantic product or recapture of
        # a completed/rejected candidate. The image panel reuses saved evidence.
        timeline=omni.timeline.get_timeline_interface();timeline.stop();frozen_timeline_time=timeline.get_current_time()
        stage=omni.usd.get_context().get_stage();stage.GetRootLayer().subLayerPaths.append(run['cfg']['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        robot=UsdGeom.Xform.Define(stage,run['cfg']['robot']['prim']);robot.GetPrim().GetReferences().AddReference(run['cfg']['robot']['asset'])
        robot.ClearXformOpOrder()
        render=SimpleNamespace(position=robot.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'recorded'),
                               orientation=robot.AddOrientOp(UsdGeom.XformOp.PrecisionDouble,'recorded'))
        for prim in Usd.PrimRange(robot.GetPrim(),Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.IsInstanceProxy():UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        write_json(out/'lighting.json',apply_hospital_lights(stage,run['cfg']['hospital_lighting']))
        from research.audit_hospital_lights import inspect_stage
        import omni.client
        actual_collisions=inspect_stage(stage)[-1]
        expected_collisions=json.loads(Path(cfg['collision_inventory']).read_text())
        if actual_collisions!=expected_collisions:raise ValueError('Hospital collision inventory changed')
        result,_,content=omni.client.read_file(run['cfg']['scene']['usd'])
        if result!=omni.client.Result.OK:raise RuntimeError('Could not hash Hospital source')
        official_hash=hashlib.sha256(bytes(content)).hexdigest()
        expected_scene=json.loads((Path(cfg['collision_inventory']).parent/'hospital_scene.json').read_text())
        if official_hash!=expected_scene['source_byte_sha256']:raise ValueError('Official Hospital USD changed')
        write_json(out/'scene_identity.json',{'source_byte_sha256':official_hash,'collision_prim_count':len(actual_collisions),
                   'matches_frozen_inventory':True,'renderer_and_fixture_profile':'Unchanged baseline profile, recorded in lighting.json'})
        def guard(event):
            if event.type==int(omni.timeline.TimelineEventType.PLAY):timeline.stop()
        subscription=timeline.get_timeline_event_stream().create_subscription_to_pop(guard)
        human=({'prim':'/World/E16Cart'} if is_cart else
            {'prim':'/World/SearchHuman','asset':asset['asset'],'radius_m':cfg['human_radius_m'],
             'capsule_cylinder_height_m':1.2,'yaw':cfg['human_yaw_rad']})
        st={**restart_state(run['times']),'speed':1.,'follow':False,'overlay':False,'last_rgb':None,'sync':False}
        observation_times=np.array([e['observation']['sim_time'] for e in run['events']])
        guide_root=UsdGeom.Xform.Define(stage,'/World/SearchGuides')

        def curve(name,points,color,width=.04):
            c=UsdGeom.BasisCurves.Define(stage,'/World/SearchGuides/'+name)
            c.CreateTypeAttr('linear');c.CreateWrapAttr('nonperiodic');c.CreateWidthsAttr([width]);c.SetWidthsInterpolation('constant')
            c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            change(c,points);return c
        def change(c,points,z=.07):
            c.CreateCurveVertexCountsAttr([len(points)])
            c.CreatePointsAttr([Gf.Vec3f(float(x),float(y),z) for x,y in points])
        curve('Baseline',run['xy'],[0.,.95,1.])
        if is_cart:
            reference=load_saved(cfg['reference_baseline_run']);curve('NoCartReference',reference['xy'],[.7,.7,.7],.025)
        old=curve('OLDRemaining',[[0,0],[0,0]],[1.,.4,0.])
        fresh_guide=curve('FRESH',[[0,0],[0,0]],[.2,.4,1.])
        UsdGeom.Imageable(fresh_guide).MakeInvisible()
        bypass=curve('LocalBypass',[[0,0],[0,0]],[0.,1.,.15])
        proxy=curve('ConflictProxy',[[0,0],[0,0]],[1.,0.,0.],.02)
        human_guide=curve('HumanFootprint',[[0,0],[0,0]],[1.,0.,.8],.05)
        occluder=curve('OccluderBounds',[[0,0],[0,0]],[1.,1.,.1],.025)
        conflict=UsdGeom.Sphere.Define(stage,'/World/SearchGuides/ConflictPoint');conflict.CreateRadiusAttr(.11)
        conflict.CreateDisplayColorAttr([Gf.Vec3f(1,0,0)]);conflict_pos=conflict.AddTranslateOp()
        overview_cam=UsdGeom.Camera.Define(stage,'/World/SearchOverview');overview_cam.CreateProjectionAttr('orthographic')
        overview_cam.CreateClippingRangeAttr(Gf.Vec2f(3.5,100));overview_op=overview_cam.AddTransformOp()
        follow_cam=UsdGeom.Camera.Define(stage,'/World/SearchFollow');follow_cam.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
        follow_cam.CreateFocalLengthAttr(18.);follow_op=follow_cam.AddTransformOp()
        viewport=get_active_viewport()
        def capture_file(target):
            capture=capture_viewport_to_file(viewport,str(target))
            completed=asyncio.ensure_future(capture.wait_for_result())
            deadline=time.monotonic()+30.
            while not completed.done() and app.is_running() and time.monotonic()<deadline:app.update()
            if not completed.done():raise RuntimeError('Viewport capture timed out')
            completed.result()
            # Kit's result signals GPU capture scheduling; PNG encoding/writing
            # can finish later on a worker. Require a fully decodable file.
            while app.is_running() and time.monotonic()<deadline:
                try:
                    with Image.open(target) as captured:captured.load()
                    return
                except (FileNotFoundError,OSError):app.update();time.sleep(.005)
            raise RuntimeError('Viewport PNG write did not finish')
        def look(op,eye,target,up=(0,0,1)):
            op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(*up)).GetInverse())
        def overview():
            st['follow']=False;c=nav.current
            points=np.vstack([run['xy'],c['position'][:2]]) if c else run['xy']
            if c and c['bypass']['centerline_xy']:points=np.vstack([points,c['bypass']['centerline_xy']])
            if c and not c.get('remaining_world_xy'):
                # First CLEAR can be C1: include the saved robot observation
                # without inventing an OLD curve or a later reveal pair.
                rid=c.get('fresh_request_id') or 1
                points=np.vstack([points,run['events'][rid-1]['agent_pose_at_observation'][:2]])
            lo,hi=points.min(0)-1.7,points.max(0)+1.7;center=(lo+hi)/2
            width=max(hi[0]-lo[0],(hi[1]-lo[1])*1.65,6.)
            overview_cam.CreateHorizontalApertureAttr(float(width*10));overview_cam.CreateVerticalApertureAttr(float(width/1.65*10))
            look(overview_op,[*center,6],[*center,0],(0,1,0));viewport.camera_path=str(overview_cam.GetPath())
        def front():
            st['follow']=False;viewport.camera_path=run['cfg']['camera']['prim']
        def follow():
            st['follow']=True;viewport.camera_path=str(follow_cam.GetPath())
        def jump(kind):
            rid=((nav.current.get('first_visible_request_id') or 1) if kind=='visible' else nav.jump('fresh' if kind=='application' else kind))
            t=run['events'][rid-1]['application']['sim_time'] if kind=='application' else observation_times[rid-1]
            st.update(time=float(t),playing=False)
        def restart():
            st.update(restart_state(run['times']));nav.request_id=1
        def speed(value):
            st['speed']=value;speed_label.text=f"Playback speed: {value:g}x"
        def obs_move(delta):
            rid=int(np.searchsorted(observation_times,st['time']+1e-9,side='right'))
            nav.request_id=max(1,min(len(run['events']),rid+delta));st.update(time=float(observation_times[nav.request_id-1]),playing=False)
        def select():
            if stage.GetPrimAtPath(human['prim']):stage.RemovePrim(human['prim'])
            c=nav.current;st.update(playing=False,last_rgb=None)
            if not c:
                for guide in [old,fresh_guide,bypass,proxy,human_guide,occluder]:UsdGeom.Imageable(guide).MakeInvisible()
                UsdGeom.Imageable(conflict).MakeInvisible();return
            if is_cart:
                from research.hospital_cart import author
                author(stage,cfg['cart_frozen'])
            else:author_human(stage,{**human,'start_position':c['position']},asset,physical=False)
            theta=np.linspace(0,2*np.pi,65);circle=np.column_stack([np.cos(theta),np.sin(theta)])
            for guide,points in [(old,c.get('remaining_world_xy',[])),(fresh_guide,c.get('fresh_world_xy',[])),(bypass,c['bypass']['centerline_xy']),
                                 (proxy,c['inflated_outline'] if is_cart else c['position'][:2]+circle*(cfg['robot_radius_m']+cfg['human_radius_m'])),
                                 (human_guide,c['cart_outline'] if is_cart else c['position'][:2]+circle*cfg['human_radius_m'])]:
                if len(points)>1:change(guide,points);UsdGeom.Imageable(guide).MakeVisible()
                else:UsdGeom.Imageable(guide).MakeInvisible()
            hit=c.get('conflict')
            if hit:
                conflict_pos.Set(Gf.Vec3d(*hit['point'],.15));UsdGeom.Imageable(conflict).MakeVisible()
            else:UsdGeom.Imageable(conflict).MakeInvisible()
            path=c.get('occluding_prim_path');prim=stage.GetPrimAtPath(path) if path else None
            if prim:
                box=UsdGeom.BBoxCache(0,['default','render','proxy']).ComputeWorldBound(prim).ComputeAlignedRange()
                lo,hi=np.array(box.GetMin()),np.array(box.GetMax())
                change(occluder,[[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]],[lo[0],lo[1]]]);UsdGeom.Imageable(occluder).MakeVisible()
            else:UsdGeom.Imageable(occluder).MakeInvisible()
            restart();overview()
        def move(delta):nav.move(delta);select()
        def filter_mode(mode):nav.filter(mode);select()
        def top():
            nav.filter('all');target=summary.get('selected_candidate_id') or summary.get('diagnostic_candidate_id')
            nav.index=next((i for i,c in enumerate(nav.items) if c['id']==target),0);select()
        def replay():
            c=nav.current;rid=max(1,(c.get('fresh_request_id') or 4)-3) if c else 1
            st.update(time=float(observation_times[rid-1]),playing=True);follow()
        window=ui.Window('Reveal window | MODEL-FREE saved replay',width=490,height=925);window.setPosition(10,45)
        with window.frame:
            with ui.VStack(spacing=4):
                ui.Label('NO MODEL CALLS | NO NAVIGATION PHYSICS',height=25)
                if summary.get('single_manual_placement'):
                    ui.Label('PREFLIGHT ONLY — NO HUMAN NAVIGATION RUN',height=32,word_wrap=True)
                if is_cart:ui.Label('STATIONARY HOSPITAL CART | SAVED REAL MODEL RUN',height=32,word_wrap=True)
                ui.Label('Cyan path | Orange OLD | Magenta '+('cart' if is_cart else 'human')+' | Red conflict | Green bypass',height=36,word_wrap=True)
                info=ui.Label('',height=185,word_wrap=True)
                with ui.HStack(height=28):
                    ui.Button('Previous candidate',clicked_fn=lambda:move(-1));ui.Button('Next candidate',clicked_fn=lambda:move(1))
                ui.Button('Top / selected (diagnostic if none)',height=28,clicked_fn=top)
                with ui.HStack(height=28):
                    ui.Button('Show strict only',clicked_fn=lambda:filter_mode('strict'))
                    ui.Button('Show near misses',clicked_fn=lambda:filter_mode('near'))
                    ui.Button('All candidates',clicked_fn=lambda:filter_mode('all'))
                with ui.HStack(height=28):
                    ui.Button('Previous observation',clicked_fn=lambda:obs_move(-1));ui.Button('Next observation',clicked_fn=lambda:obs_move(1))
                with ui.HStack(height=28):
                    ui.Button('Jump OLD',clicked_fn=lambda:jump('old'));ui.Button('Jump FRESH' if is_cart else 'Jump first CLEAR',clicked_fn=lambda:jump('fresh'))
                    ui.Button('Jump switch/application',clicked_fn=lambda:jump('application'))
                if is_cart:ui.Button('Jump first cart visibility',height=25,clicked_fn=lambda:jump('visible'))
                with ui.HStack(height=28):
                    ui.Button('Restart from start',clicked_fn=restart);ui.Button('Play',clicked_fn=lambda:st.update(playing=True));ui.Button('Pause',clicked_fn=lambda:st.update(playing=False))
                with ui.HStack(height=25):
                    speed_label=ui.Label('Playback speed: 1x')
                    for value in [.25,.5,1.,2.]:ui.Button(f'{value:g}x',clicked_fn=lambda v=value:speed(v))
                slider=ui.FloatSlider(min=float(run['times'][0]),max=float(run['times'][-1]),height=24)
                slider.model.add_value_changed_fn(lambda m:None if st['sync'] else st.update(time=m.as_float,playing=False))
                with ui.HStack(height=28):
                    ui.Button('Overview',clicked_fn=overview);ui.Button('Nova front Hawk',clicked_fn=front);ui.Button('Follow robot',clicked_fn=follow)
                ui.Button('Toggle saved semantic overlay',height=25,clicked_fn=lambda:st.update(overlay=not st['overlay'],last_rgb=None))
                image_title=ui.Label('Actual archived preflight Hawk render',height=24)
                rgb_panel=ui.Image('',height=260,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                ui.Label('Cart fixed throughout; grey no-cart reference, cyan actual path, orange OLD, blue FRESH.\nArchived RGB is the original model observation. No physics or model reexecution.' if is_cart else 'Stationary human is fixed during each saved replay. Candidate changes are independent alternatives.\nOverview clips the roof for inspection; original preflight RGB has no guides.\nA near miss is never a selected strict candidate.',height=70,word_wrap=True)

        def update_pose():
            c=nav.current;t=st['time'];i=max(0,int(np.searchsorted(run['times'],t+1e-9,side='right')-1));row=run['robot'][i]
            pos=[float(row[k]) for k in ['x','y','z']];q=[float(row[k]) for k in ['qw','qx','qy','qz']]
            render.position.Set(Gf.Vec3d(*pos));render.orientation.Set(Gf.Quatd(q[0],Gf.Vec3d(*q[1:])))
            if st['follow']:
                yaw=float(row['yaw']);f=np.array([np.cos(yaw),np.sin(yaw),0.]);p=np.array(pos)
                look(follow_op,p-2*f+[0,0,2.4],p+.5*f+[0,0,.4])
            rid=max(1,int(np.searchsorted(observation_times,t+1e-9,side='right')));nav.request_id=rid
            if c:
                v=c['visibility'][rid-1] if c['visibility'] else None
                source_image=Path(v['rgb']).resolve() if v else None
                if v and st['overlay']:
                    overlay=out/f"overlays/c{c['id']:04d}_C{rid:02d}.png"
                    if not overlay.exists():
                        overlay.parent.mkdir(exist_ok=True);im=np.asarray(Image.open(source_image)).copy();mask=np.asarray(Image.open(v['mask']))>0
                        im[mask]=(im[mask]*.45+np.array([255,0,200])*.55).astype(np.uint8);Image.fromarray(im).save(overlay)
                    source_image=overlay
                url=str(source_image) if source_image else ''
                if url!=st['last_rgb']:rgb_panel.source_url=url;st['last_rgb']=url
                info.text=(f"Candidate {c['id']} | strict rank {c['selection_rank']} | XYZ {c['position']}\n"
                    f"{'STRICT QUALIFIED' if c['strict_qualified'] else 'NOT SELECTED / near miss'} | failures {c['failures']}\n"
                    f"Saved t={t:.3f}s tick={row['tick']} | C{rid}: {v['state'] if v else 'VISIBILITY NOT MEASURED'}\n"
                    f"Pixels {v['human_visible_pixel_count'] if v else None} / fraction {v['human_visible_fraction'] if v else None}\n"
                    f"OLD C{c.get('old_request_id')} -> {'primary FRESH' if is_cart else 'first CLEAR'} C{c.get('fresh_request_id')}\n"
                    f"OLD min clearance {c.get('old_min_clearance_m')} m\n"
                    f"Current reveal clearance {c.get('robot_current_clearance_at_reveal_m')} m\n"
                    f"Lead {c.get('reveal_lead_s')} s | switch margin {c.get('switch_to_conflict_margin_s')} s\n"
                    f"Bypass {c['bypass']['pass']} | clearance {c['bypass'].get('clearance_m')} m")
                image_title.text=f"Archived candidate {c['id']} C{rid} {'overlay' if st['overlay'] else 'RGB'}" if v else 'No rendered evidence for this geometry-filtered candidate'
            else:
                info.text='No candidates in this filter. Choose near misses or all candidates.';rgb_panel.source_url=''
            st['sync']=True;slider.model.set_value(t);st['sync']=False
            return rid
        top();update_pose()
        for _ in range(25):app.update()
        checks={}
        if a.self_test:
            target=nav.current['id'];move(1);update_pose();move(-1);update_pose()
            checks['candidate_next_previous']=nav.current['id']==target
            filter_mode('strict');update_pose();checks['strict_filter']=all(c['strict_qualified'] for c in nav.items)
            filter_mode('near');update_pose();checks['near_filter']=all(not c['strict_qualified'] for c in nav.items)
            top();c=nav.current
            jump('old');checks['jump_old']=update_pose()==(c.get('old_request_id') or 1)
            jump('fresh');checks['jump_first_clear']=update_pose()==(c.get('fresh_request_id') or 1)
            jump('application');update_pose()
            checks['jump_application']=st['time']==run['events'][(c.get('fresh_request_id') or 1)-1]['application']['sim_time']
            if is_cart:
                jump('visible');checks['jump_first_cart_visibility']=update_pose()==(c.get('first_visible_request_id') or 1)
            jump('fresh')
            obs_move(-1);update_pose();obs_move(1)
            checks['observation_navigation']=update_pose()==min(len(run['events']),max(1,(c.get('fresh_request_id') or 1)-1)+1)
            human_transform=UsdGeom.Xformable(stage.GetPrimAtPath(human['prim'])).ComputeLocalToWorldTransform(0)
            jump('old');update_pose();before=render.position.Get()
            replay();update_pose();st['time']=min(st['time']+.5,float(run['times'][-1]));update_pose()
            checks['saved_pose_replay_moves_robot']=render.position.Get()!=before
            checks['replay_keeps_human_fixed']=human_transform==UsdGeom.Xformable(stage.GetPrimAtPath(human['prim'])).ComputeLocalToWorldTransform(0)
            st.update(playing=False,overlay=True,last_rgb=None);update_pose()
            checks['archived_overlay_panel']=Path(rgb_panel.source_url).is_file()
            st.update(overlay=False,last_rgb=None);update_pose()
            checks['visibility_archived_masks']=[]
            for rid in sorted(set([max(1,(c.get('fresh_request_id') or 4)-3),c.get('old_request_id') or 1,c.get('fresh_request_id') or 1])):
                st['time']=float(observation_times[rid-1]);update_pose();saved=c['visibility'][rid-1]
                mask=np.asarray(Image.open(saved['mask']))>0;ys,xs=np.where(mask)
                stats={'human_visible_pixel_count':int(mask.sum()),'human_visible_fraction':float(mask.mean()),
                       'human_bbox_if_visible':[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)] if len(xs) else None}
                actual=visibility_state(stats,cfg['visibility']);expected=saved['state']
                checks['visibility_archived_masks'].append({'request_id':rid,'expected':expected,'actual':actual,'stats':stats,'pass':actual==expected})
            for name,fn in [('overview',overview),('hawk',front),('follow',follow)]:
                jump('fresh');fn();update_pose()
                for _ in range(8):app.update()
                capture_file(out/(name+'.png'))
            checks.update(hospital_present=bool(stage.GetPrimAtPath('/Root')),robot_present=bool(stage.GetPrimAtPath(run['cfg']['robot']['prim'])),
                          human_present=bool(stage.GetPrimAtPath(human['prim'])),old_remaining_present=bool(c.get('remaining_world_xy')),
                          conflict_point_present=c.get('conflict') is not None,conflict_proxy_present=True,
                          physics_stopped=not timeline.is_playing(),model_calls=0,new_semantic_captures=0,
                          verification_mode='Archived masks/RGB plus display-only viewport; no preflight rerender')
            if not all(checks[k] for k in ['candidate_next_previous','strict_filter','near_filter','jump_old','jump_first_clear','jump_application','observation_navigation','saved_pose_replay_moves_robot','replay_keeps_human_fixed','archived_overlay_panel','hospital_present','robot_present','human_present','physics_stopped']):
                raise RuntimeError('GUI control verification failed')
            if not all(r['pass'] for r in checks['visibility_archived_masks']):raise RuntimeError('Archived mask differs from visibility log')
        if a.export_full:
            frames=out/'frames';frames.mkdir();manifest=[];follow();st.update(playing=False)
            fixed_transform=UsdGeom.Xformable(stage.GetPrimAtPath(human['prim'])).ComputeLocalToWorldTransform(0)
            font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
            times=replay_frame_times(run['times']);fps=10
            for number,t in enumerate(times):
                st['time']=t;rid=update_pose()
                for _ in range(3):app.update()
                target=frames/f'frame_{number:05d}.png'
                capture_file(target)
                assert not timeline.is_playing() and timeline.get_current_time()==frozen_timeline_time
                assert fixed_transform==UsdGeom.Xformable(stage.GetPrimAtPath(human['prim'])).ComputeLocalToWorldTransform(0)
                # Annotated display export only; the original saved Hawk files
                # remain untouched and are explicitly timestamped sample-and-hold.
                im=Image.open(target).convert('RGB');im.load();draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,im.width,92),fill='#14212b')
                draw.text((14,8),'STATIONARY HOSPITAL CART — SAVED REAL MODEL RUN' if is_cart else 'PREFLIGHT ONLY — NO HUMAN NAVIGATION RUN',font=font,fill='white')
                draw.text((14,35),f'Saved {"cart run" if is_cart else "baseline"} replay t={t:.3f}s | C{rid} | model calls 0 | physics reexecution false',font=font,fill='white')
                v=nav.current['visibility'][rid-1] if nav.current['visibility'] else None
                if v:
                    rgb=Image.open(v['rgb']).convert('RGB');mask=np.asarray(Image.open(v['mask']))>0
                    overlay=np.array(rgb);overlay[mask]=(overlay[mask]*.45+np.array([255,0,200])*.55).astype('uint8')
                    thumb=Image.fromarray(overlay);thumb.thumbnail((480,270));im.paste(thumb,(im.width-thumb.width-10,102))
                    draw.text((14,62),f"Archived Hawk C{rid} at t={observation_times[rid-1]:.3f}s: {v['state']} (held until next observation)",font=font,fill='#edc8ea')
                im.save(target)
                i=max(0,int(np.searchsorted(run['times'],t+1e-9,side='right')-1))
                manifest.append({'frame':target.name,'replay_time_s':t,'saved_tick':int(run['robot'][i]['tick']),
                    'saved_pose_time_s':float(run['times'][i]),'robot_position_world_m':list(render.position.Get()),
                    'request_id':rid,'rgb':v['rgb'] if v else None,'mask':v['mask'] if v else None})
                if number%25==0:print('FULL_REPLAY_EXPORT',number+1,'/',len(times),flush=True)
            write_json(out/'frames.json',manifest)
            video={'ffmpeg_installed':bool(shutil.which('ffmpeg')),'fps':fps,'frame_count':len(manifest),
                'start_s':times[0],'end_s':times[-1],'complete_saved_run':True,'model_calls':0,'navigation_physics_reexecuted':False}
            if video['ffmpeg_installed']:
                result=subprocess.run(['ffmpeg','-nostdin','-n','-framerate',str(fps),'-i',str(frames/'frame_%05d.png'),
                    '-c:v','libx264','-pix_fmt','yuv420p','-vf','scale=trunc(iw/2)*2:trunc(ih/2)*2',str(out/'full_run_replay.mp4')],capture_output=True,text=True)
                (out/'ffmpeg.log').write_text(result.stderr);video['returncode']=result.returncode
                if result.returncode:raise RuntimeError('Installed ffmpeg export failed')
            write_json(out/'video.json',video);print('FULL_REPLAY_EXPORT_COMPLETE',len(times),flush=True)
        top();restart();overview();update_pose()
        checks['replay_starts_at_t0']=st['time']==0. and nav.request_id==1 and not st['playing']
        speed(.25);checks['speed_controls']=st['speed']==.25;speed(1.)
        if not checks['replay_starts_at_t0']:raise RuntimeError('Replay did not reset to t=0')
        verify_hashes(hashes)
        write_json(out/'GUI_READY.json',{'status':'GUI_READY','headless':False,'model_calls':0,'navigation_physics_reexecuted':False,
                   'candidate_id':nav.current['id'] if nav.current else None,'initial_replay_time_s':st['time'],
                   'initial_request_id':nav.request_id,'checks':checks,'source_unchanged':True})
        print('GUI_READY',out,flush=True)
        previous=time.monotonic()
        while app.is_running():
            now=time.monotonic()
            if st['playing']:
                st['time']=min(float(run['times'][-1]),st['time']+(now-previous)*st['speed'])
                if st['time']>=run['times'][-1]:st['playing']=False
            previous=now;update_pose();app.update();time.sleep(.01)
        verify_hashes(hashes)
    except Exception as exc:
        traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc)});raise
    finally:app.close()


if __name__=='__main__':main()
