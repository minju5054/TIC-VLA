"""Interactive saved Nova/human replay. No model calls or physics re-execution."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.nova_replay import ReplayData,validate_output
from research.records import write_json,provenance


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ["run-dir","analysis-dir","output-dir"]:p.add_argument("--"+name,required=True)
    p.add_argument("--export-primary",action="store_true",help="Export20fps viewport PNGs, optional installed ffmpeg MP4")
    p.add_argument("--self-test",action="store_true",help="Exercise recorded poses/selection/cameras before GUI_READY")
    a=p.parse_args();output=validate_output(a.run_dir,a.analysis_dir,a.output_dir)
    data=ReplayData(a.run_dir,a.analysis_dir);output.mkdir(parents=True,exist_ok=False)
    cfg=data.cfg
    write_json(output/"metadata.json",{"mode":"recorded_pose_GUI_replay","source_run":str(data.source),"analysis_dir":str(data.analysis),
        "source_files_sha256":data.hashes,"model_calls":0,"physics_reexecuted":False,
        "pose_rule":"Exact saved past tick, no pose resimulation; own-observation world chunks fixed",
        "display_note":"Retrospective selected predictions; display-only fill lighting, guides lifted above floor, overview near-plane clips roof; original RGB panel is archived evidence",
        **provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":False,"renderer":"RayTracedLighting","width":1440,"height":900})
    try:
        import omni.usd
        import omni.timeline
        import omni.ui as ui
        from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdLux
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        from research.human_actor import author_human
        timeline=omni.timeline.get_timeline_interface();timeline.stop()
        stage=omni.usd.get_context().get_stage()
        stage.GetRootLayer().subLayerPaths.append(cfg["scene"]["usd"])
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        # Display-only light makes the otherwise dim Hospital inspectable.
        # Archived camera inputs remain unmodified in the original RGB panel.
        fill=UsdLux.SphereLight.Define(stage,"/World/ReplayDisplayLight")
        fill.CreateIntensityAttr(15000.)
        fill.CreateRadiusAttr(.3)
        fill.CreateEnableColorTemperatureAttr(False)
        fill_position=UsdGeom.Xformable(fill).AddTranslateOp()
        robot=UsdGeom.Xform.Define(stage,cfg["robot"]["prim"]);robot.GetPrim().GetReferences().AddReference(cfg["robot"]["asset"])
        robot.ClearXformOpOrder()
        position=robot.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,"recorded")
        orientation=robot.AddOrientOp(UsdGeom.XformOp.PrecisionDouble,"recorded")
        human_position,_=author_human(stage,cfg["dynamic_human"],data.asset,physical=False)
        # Disable all robot rigid bodies in this display stage. Playback only
        # authors recorded transforms; no World, physics context or model client.
        for prim in Usd.PrimRange(robot.GetPrim(),Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.IsInstanceProxy():
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        def guard(event):
            if event.type==int(omni.timeline.TimelineEventType.PLAY):timeline.stop()
        subscription=timeline.get_timeline_event_stream().create_subscription_to_pop(guard)

        def curve(name,points,color,width=.035,z=.055):
            c=UsdGeom.BasisCurves.Define(stage,"/World/ReplayGuides/"+name)
            c.CreateTypeAttr("linear");c.CreateWrapAttr("nonperiodic");c.CreateCurveVertexCountsAttr([len(points)])
            c.CreatePointsAttr([Gf.Vec3f(float(x),float(y),z) for x,y in points])
            c.CreateWidthsAttr([width]);c.SetWidthsInterpolation("constant");c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            return c
        def change_curve(c,points,z=.055):
            c.GetCurveVertexCountsAttr().Set([len(points)])
            c.GetPointsAttr().Set([Gf.Vec3f(float(x),float(y),z) for x,y in points])
        xy=np.array([[float(r["x"]),float(r["y"])] for r in data.robot])
        hxy=np.array([[float(r["x"]),float(r["y"])] for r in data.human])
        curve("RecordedRobot",xy,[0.,.95,1.],.045);curve("RecordedHuman",hxy,[1.,.05,.8],.05)
        old_guide=curve("SelectedOLD",data.worlds[data.old-1],[1.,.36,.02],.045,z=.08)
        fresh_guide=curve("SelectedFRESH",data.worlds[data.fresh-1],[.05,1.,.1],.045,z=.085)
        def marker(name,color):
            s=UsdGeom.Sphere.Define(stage,"/World/ReplayGuides/"+name);s.CreateRadiusAttr(.075)
            s.CreateDisplayColorAttr([Gf.Vec3f(*color)]);return s.AddTranslateOp()
        boundary_marker=marker("SwitchBoundary",[1,1,1]);observation_marker=marker("Observation",[1,1,0])
        circle=curve("HumanConflictProxy",np.zeros((65,2)),[1.,.05,.05],.025,z=.06)
        overview_cam=UsdGeom.Camera.Define(stage,"/World/ReplayOverview")
        overview_cam.CreateProjectionAttr("orthographic");overview_cam.CreateClippingRangeAttr(Gf.Vec2f(3.5,100))
        overview_transform=overview_cam.AddTransformOp()
        follow_cam=UsdGeom.Camera.Define(stage,"/World/ReplayFollow")
        follow_cam.CreateClippingRangeAttr(Gf.Vec2f(.05,100));follow_transform=follow_cam.AddTransformOp()
        for _ in range(15):app.update()
        viewport=get_active_viewport();state={"time":data.jump_time(),"playing":False,"speed":.5,"old":data.old,"fresh":data.fresh,"syncing":False,"follow":False,"last_rgb":None}

        def lookat(op,eye,target,up=(0,0,1)):
            op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(*up)).GetInverse())
        def overview():
            state["follow"]=False
            points=np.vstack([data.worlds[state["old"]-1],data.worlds[state["fresh"]-1],hxy])
            lo,hi=points.min(0)-1.,points.max(0)+1.;center=(lo+hi)/2
            width=max(hi[0]-lo[0],(hi[1]-lo[1])*1.6,5.)
            overview_cam.CreateHorizontalApertureAttr(float(width*10))
            overview_cam.CreateVerticalApertureAttr(float(width/1.6*10))
            lookat(overview_transform,[*center,6.],[*center,0.],(0,1,0));viewport.camera_path=str(overview_cam.GetPath())
        def follow():
            state["follow"]=True;viewport.camera_path=str(follow_cam.GetPath())
        def front():
            state["follow"]=False;viewport.camera_path=cfg["camera"]["prim"]
        def selected():
            old,fresh=state["old"],state["fresh"]
            change_curve(old_guide,data.worlds[old-1],.08);change_curve(fresh_guide,data.worlds[fresh-1],.085)
            event=data.events[fresh-1]
            boundary_marker.Set(Gf.Vec3d(*event["application"]["position"][:2],.10))
            observation_marker.Set(Gf.Vec3d(*event["observation"]["position"][:2],.10))
            sample=data.sample(event["observation"]["sim_time"]);center=np.array(sample["human_position"][:2])
            angle=np.linspace(0,2*np.pi,65);radius=cfg["dynamic_analysis"]["robot_radius_m"]+cfg["dynamic_human"]["radius_m"]
            change_curve(circle,center+radius*np.column_stack([np.cos(angle),np.sin(angle)]),.06)
        def set_time(t,pause=True):
            state["time"]=float(np.clip(t,data.times[0],data.times[-1]))
            if pause:state["playing"]=False
        def jump():
            state.update(old=data.old,fresh=data.fresh,time=data.jump_time(),playing=True,speed=.5)
            state["syncing"]=True;old_model.set_value(data.old);fresh_model.set_value(data.fresh);state["syncing"]=False
            selected();overview()
        def change_request(delta):
            latest=data.sample(state["time"])["latest_request"] or 1
            rid=max(1,min(len(data.events),latest+delta));set_time(data.events[rid-1]["observation"]["sim_time"])
        window=ui.Window("TIC-VLA | Recorded Nova dynamic handoff",width=475,height=790)
        window.setPosition(15,55)
        with window.frame:
            with ui.VStack(spacing=4):
                ui.Label("RECORDED POSES ONLY — no model / no physics",height=25)
                ui.Label(data.source.name,height=20)
                ui.Label("Cyan robot | Magenta human | Orange OLD | Green FRESH",height=22)
                ui.Label("White switch B | Yellow observation | Red conflict proxy",height=22)
                phase=ui.Label("",height=34,word_wrap=True)
                info=ui.Label("",height=120,word_wrap=True)
                with ui.HStack(height=28):
                    ui.Button("Play",clicked_fn=lambda:state.update(playing=True))
                    ui.Button("Pause",clicked_fn=lambda:state.update(playing=False))
                    ui.Button("Restart",clicked_fn=lambda:set_time(data.times[0]))
                with ui.HStack(height=25):
                    for speed in [.25,.5,1.,2.]:ui.Button(str(speed)+"x",clicked_fn=lambda s=speed:state.update(speed=s))
                slider=ui.FloatSlider(min=float(data.times[0]),max=float(data.times[-1]),height=25)
                slider.model.add_value_changed_fn(lambda m:None if state["syncing"] else set_time(m.as_float))
                with ui.HStack(height=28):
                    ui.Button("Previous request",clicked_fn=lambda:change_request(-1))
                    ui.Button("Next request",clicked_fn=lambda:change_request(1))
                with ui.HStack(height=25):
                    ui.Label("Select OLD",width=85);old_model=ui.IntDrag(min=1,max=len(data.events)).model
                    ui.Label("Select FRESH",width=95);fresh_model=ui.IntDrag(min=1,max=len(data.events)).model
                def choose(key,model):
                    if state["syncing"]:return
                    state[key]=max(1,min(len(data.events),model.as_int));selected()
                old_model.add_value_changed_fn(lambda m:choose("old",m));fresh_model.add_value_changed_fn(lambda m:choose("fresh",m))
                with ui.HStack(height=28):
                    ui.Button(f"Primary pair C{data.old}→C{data.fresh}",clicked_fn=jump)
                    ui.Button("Jump to dynamic handoff",clicked_fn=jump)
                with ui.HStack(height=28):
                    ui.Button("Overview camera",clicked_fn=overview)
                    ui.Button("Follow robot camera",clicked_fn=follow)
                    ui.Button("Nova front Hawk",clicked_fn=front)
                ui.Label("Original recorded model observation",height=24)
                rgb=ui.Image("",height=240,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                ui.Label("Viewport = saved poses with display lighting. RGB above = original input.\nSelected curves are retrospective, fixed at own observation pose.\nOverview clips roof for display; source scene/data unchanged.",height=55,word_wrap=True)

        def render_pose():
            sample=data.sample(state["time"]);position.Set(Gf.Vec3d(*sample["robot_position"]))
            q=sample["robot_quaternion_wxyz"];orientation.Set(Gf.Quatd(q[0],Gf.Vec3d(*q[1:])))
            human_position.Set(Gf.Vec3d(*sample["human_position"]))
            # Put the inspection light inside the room, below its ceiling.
            light_xy=(np.array(sample["robot_position"][:2])+sample["human_position"][:2])/2
            fill_position.Set(Gf.Vec3d(*light_xy,2.1))
            if state["follow"]:
                p=np.array(sample["robot_position"]);f=np.array([np.cos(sample["robot_yaw"]),np.sin(sample["robot_yaw"]),0.])
                lookat(follow_transform,p-2*f+[0,0,1.7],p+f+[0,0,.6])
            rid=sample["latest_request"]
            if sample["rgb"]!=state["last_rgb"] and sample["rgb"]:
                rgb.source_url=sample["rgb"];state["last_rgb"]=sample["rgb"]
            if sample["pending_request"]:
                phase.text=f"FRESH C{sample['pending_request']} pending | OLD C{sample['active_request']} active"
            elif sample["raw_switch"] and sample["active_request"]:
                phase.text=f"RAW SWITCH C{sample['active_request']-1} → C{sample['active_request']}"
            else:phase.text=f"Accepted command C{sample['active_request']} active"
            metric=data.metrics[state["fresh"]-1];gap=metric["raw_boundary_position_gap_m"];tangent=metric["raw_executed_to_fresh_tangent_gap_deg"]
            info.text=(f"Replay sim {sample['time']:.3f}s | saved tick {sample['recorded_tick']} | latest C{rid}\n"
                f"Robot XY {sample['robot_position'][0]:.3f}, {sample['robot_position'][1]:.3f} | yaw {np.degrees(sample['robot_yaw']):.2f}°\n"
                f"Human XY {sample['human_position'][0]:.3f}, {sample['human_position'][1]:.3f}\n"
                f"Target v/w {sample['command'][0]:.3f} / {sample['command'][1]:.3f} | speed {state['speed']}x\n"
                f"Selected C{state['old']} → C{state['fresh']} | RAW seam {gap if gap is None else round(gap,4)} m\n"
                f"Executed→FRESH tangent {tangent if tangent is None else round(tangent,2)}°")
            state["syncing"]=True;slider.model.set_value(sample["time"]);state["syncing"]=False
            return sample
        jump();state["playing"]=False;render_pose()
        for _ in range(30):app.update()
        checks={}
        if a.self_test:
            before=data.sample(data.jump_time());after=data.sample(data.events[data.fresh-1]["application"]["sim_time"])
            for t in [before["time"],after["time"]]:
                set_time(t);s=render_pose();app.update()
                np.testing.assert_allclose(list(position.Get()),s["robot_position"])
                np.testing.assert_allclose(list(human_position.Get()),s["human_position"])
            assert np.linalg.norm(np.array(after["human_position"])-before["human_position"])>0
            assert np.linalg.norm(np.array(after["robot_position"])-before["robot_position"])>0
            selected();follow();render_pose();app.update();front();app.update();jump();state["playing"]=False;render_pose()
            checks={"recorded_robot_replay":True,"recorded_human_replay":True,"primary_jump":True,"camera_controls":True,
                    "old_fresh_guides_fixed":True,"rgb_source_exists":Path(state["last_rgb"]).is_file(),"timeline_stopped":not timeline.is_playing()}
        overview();render_pose()
        for _ in range(15):app.update()
        stage.GetRootLayer().Export(str(output/"viewer_scene.usda"))
        capture=capture_viewport_to_file(viewport,str(output/"overview.png"))
        for _ in range(20):app.update()
        if not (output/"overview.png").is_file():raise RuntimeError("Viewport capture did not finish")
        write_json(output/"ready.json",{"status":"GUI_READY","model_calls":0,"physics_reexecuted":False,
            "primary_pair":[data.old,data.fresh],"checks":checks,"source_immutable":data.verify_immutable()})
        print("GUI_READY",str(output),flush=True)
        if a.export_primary:
            frames=output/"frames";frames.mkdir();manifest=[]
            start=data.jump_time();end=min(data.times[-1],data.events[data.fresh-1]["application"]["sim_time"]+1.5)
            for number,t in enumerate(np.arange(start,end+1e-6,1/20)):
                set_time(t);sample=render_pose()
                for _ in range(3):app.update()
                target=frames/f"frame_{number:05d}.png"
                capture=capture_viewport_to_file(viewport,str(target))
                for _ in range(3):app.update()
                manifest.append({"frame":target.name,**sample})
            write_json(output/"frames.json",manifest)
            if shutil.which("ffmpeg"):
                result=subprocess.run(["ffmpeg","-nostdin","-n","-framerate","20","-i",str(frames/"frame_%05d.png"),
                    "-c:v","libx264","-pix_fmt","yuv420p","-vf","scale=trunc(iw/2)*2:trunc(ih/2)*2",str(output/"primary_handoff_replay.mp4")],capture_output=True,text=True)
                (output/"ffmpeg.log").write_text(result.stderr)
                write_json(output/"video.json",{"returncode":result.returncode,"frame_count":len(manifest),"fps":20,"model_calls":0})
            jump();state["playing"]=False;render_pose();print("REPLAY_EXPORT_COMPLETE",len(manifest),flush=True)
        last=time.monotonic()
        while app.is_running():
            if timeline.is_playing():timeline.stop()
            now=time.monotonic()
            if state["playing"]:
                set_time(state["time"]+(now-last)*state["speed"],pause=False)
                if state["time"]>=data.times[-1]:state["playing"]=False
            last=now;render_pose();app.update();time.sleep(.01)
    except Exception as exc:
        traceback.print_exc();write_json(output/"failure.json",{"error":repr(exc)})
    finally:
        data.verify_immutable();app.close()


if __name__=="__main__":main()
