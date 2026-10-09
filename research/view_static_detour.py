"""Interactive Isaac GUI for saved static evidence. No physics/model rerun."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output-dir", required=True, help="Fresh directory for viewer log/provenance/screenshot")
    args = parser.parse_args()
    source, output = Path(args.run_dir).resolve(), Path(args.output_dir).resolve()
    if output == source or source in output.parents:
        parser.error("Viewer artifacts must be outside the source run")
    output.mkdir(parents=True, exist_ok=False)
    import numpy as np
    from research.records import write_json, provenance
    from research.analyze_chunk_geometry import project_world
    from research.route_geometry import scene_obstacles, rectangle
    meta = json.loads((source/"metadata.json").read_text())
    cfg = meta["config"]
    if meta["mode"] != "static" or cfg["scenario"] != "asymmetric_offset_detour":
        raise ValueError("Expected saved asymmetric static detour")
    events = [json.loads(p.read_text()) for p in sorted((source/"raw/requests").glob("*.json"))]
    worlds = [project_world(np.load(source/f"raw/requests/request_{e['request_id']:06d}.npy", allow_pickle=False),
                            e["agent_pose_at_observation"]) for e in events]
    with (source/"robot_state.csv").open() as f:
        robot = list(csv.DictReader(f))
    times = np.array([float(r["sim_time"]) for r in robot])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in robot])
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob("*") if p.is_file()}
    write_json(output/"metadata.json", {"mode": "saved_evidence_GUI_only", "source_run": str(source),
        "source_files_sha256": hashes, "physics_reexecuted": False, "model_calls": 0,
        "replay_note": "Set recorded poses directly; no dynamics, wheel animation or new evidence",
        "curve_note": "World projection at own observation pose; visual Z lift only; native arrays unchanged",
        **provenance(REPO)})

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False, "renderer": "RayTracedLighting", "width": 1280, "height": 800})
    try:
        import omni.usd
        import omni.timeline
        import omni.ui as ui
        from pxr import Gf, UsdGeom, UsdLux
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from isaacsim.core.utils.viewports import set_camera_view
        from research.sim import Simulation
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        light = UsdLux.DomeLight.Define(stage, "/World/Light"); light.CreateIntensityAttr(800)
        # Reuse only the existing cube authoring helper, not Simulation's physics setup.
        class Scene:
            pass
        scene = Scene(); scene.stage = stage
        Simulation.cube(scene, "/World/ViewerFloor", [4, 0, -.055], [12, 7, .1], [.22, .25, .28])
        for obj in scene_obstacles(cfg):
            color = [.1, .65, .1] if obj["name"] == "GoalWall" else obj.get("color", [.55, .55, .6])
            Simulation.cube(scene, "/World/"+obj["name"], obj["position"], obj["size"], color)
        body = UsdGeom.Xform.Define(stage, cfg["robot"]["prim"])
        body.GetPrim().GetReferences().AddReference(cfg["robot"]["asset"])
        body.ClearXformOpOrder()
        translate = body.AddTranslateOp()
        orient = body.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        camera_cfg = cfg["camera"]
        camera = UsdGeom.Camera.Define(stage, camera_cfg["prim"])
        camera.AddTranslateOp().Set(Gf.Vec3d(*camera_cfg["translation"]))
        q = camera_cfg["quaternion_wxyz"]
        camera.AddOrientOp().Set(Gf.Quatf(q[0], Gf.Vec3f(*q[1:])))
        camera.CreateFocalLengthAttr(camera_cfg["focal_length_mm"])
        camera.CreateHorizontalApertureAttr(camera_cfg["horizontal_aperture_mm"])
        camera.CreateVerticalApertureAttr(camera_cfg["vertical_aperture_mm"])
        camera.CreateClippingRangeAttr(Gf.Vec2f(*camera_cfg["clipping_range"]))

        def curve(name, points, color, width=.025):
            path = UsdGeom.BasisCurves.Define(stage, "/World/Viewer/"+name)
            path.CreateTypeAttr("linear"); path.CreateWrapAttr("nonperiodic")
            path.CreateCurveVertexCountsAttr([len(points)])
            path.CreatePointsAttr([Gf.Vec3f(float(x), float(y), .025) for x, y in points])
            path.CreateWidthsAttr([width]); path.SetWidthsInterpolation("constant")
            path.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            return path

        curve("ActualRecordedPath", xy, [.05, .9, 1.0], .045)
        predicted = curve("SelectedPrediction", worlds[4], [1., .25, .05], .045)
        block = cfg["scene"]["obstacles"][0]
        r = rectangle(block, cfg["static_detour"]["robot_footprint_radius_m"])
        curve("InflatedBlocker", [[r[0],r[1]], [r[2],r[1]], [r[2],r[3]], [r[0],r[3]], [r[0],r[1]]], [1., .75, .1])
        gate = cfg["static_detour"]["decision_gate_x"]
        curve("DecisionGate", [[gate, -2.45], [gate, 2.45]], [.9, .1, .9], .012)
        for _ in range(12):
            app.update()
        viewport = get_active_viewport()

        def overview():
            viewport.camera_path = "/OmniverseKit_Persp"
            set_camera_view(np.array([4., -8., 8.]), np.array([4., 0., .2]), viewport_api=viewport)

        state = {"index": 4, "playing": False, "wall_start": 0.0}
        window = ui.Window("TIC-VLA | Saved static run", width=430, height=570)
        window.setPosition(20, 70)
        with window.frame:
            with ui.VStack(spacing=6):
                ui.Label("SAVED EVIDENCE — no inference / no physics", height=24)
                ui.Label(source.name, height=22)
                ui.Label("Orange: native prediction   Cyan: measured robot path", height=22)
                ui.Label("Yellow: inflated blocker   Magenta: x=3.8 gate", height=22)
                info = ui.Label("", height=48, word_wrap=True)
                with ui.HStack(height=28):
                    ui.Button("Overview", clicked_fn=overview)
                    ui.Button("Robot camera", clicked_fn=lambda: setattr(viewport, "camera_path", camera_cfg["prim"]))
                def select(index, stop=True):
                    if stop:
                        state["playing"] = False
                    index = max(0, min(len(events)-1, index)); state["index"] = index
                    e = events[index]; obs = e["observation"]
                    translate.Set(Gf.Vec3d(*obs["position"]))
                    q = obs["quaternion_wxyz"]; orient.Set(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
                    predicted.GetPointsAttr().Set([Gf.Vec3f(float(x), float(y), .025) for x,y in worlds[index]])
                    rgb.source_url = str(source/e["rgb_observation_reference"])
                    info.text = f"C{e['request_id']} | observation t={obs['sim_time']:.3f} s\nRobot XY=({obs['position'][0]:.3f}, {obs['position'][1]:.3f})"
                for start in range(0, len(events), 6):
                    with ui.HStack(height=28):
                        for index in range(start, min(start+6, len(events))):
                            ui.Button(f"C{index+1}", clicked_fn=lambda i=index: select(i))
                def replay():
                    state["playing"] = True; state["wall_start"] = time.monotonic()
                with ui.HStack(height=28):
                    ui.Button("Replay recorded poses", clicked_fn=replay)
                    ui.Button("Pause", clicked_fn=lambda: state.update(playing=False))
                ui.Label("Original saved front RGB (not this viewport render)", height=22)
                rgb = ui.Image(str(source/events[4]["rgb_observation_reference"]), height=270,
                               fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                ui.Label("Replay places recorded robot poses directly. Wheels are not replayed.\n"
                         "Scene floor/colored guides are viewer aids; raw RGB is original.", height=42, word_wrap=True)
        select(4); overview()
        for _ in range(30):
            app.update()
        stage.GetRootLayer().Export(str(output/"viewer_scene.usda"))
        capture = capture_viewport_to_file(viewport, str(output/"overview.png"))
        write_json(output/"ready.json", {"status": "GUI_READY", "selected_request_id": 5, "physics_reexecuted": False,
                                          "model_calls": 0, "source_run": str(source)})
        print("SAVED_STATIC_VIEWER_READY", str(output), flush=True)
        while app.is_running():
            if timeline.is_playing():
                timeline.stop()
            if state["playing"]:
                elapsed = min(time.monotonic()-state["wall_start"], times[-1])
                tick = max(0, int(np.searchsorted(times, elapsed, side="right")-1))
                index = max(0, max((i for i,e in enumerate(events) if e["observation"]["sim_time"] <= times[tick]), default=0))
                if index != state["index"]:
                    select(index, stop=False)
                row = robot[tick]
                translate.Set(Gf.Vec3d(*[float(row[k]) for k in ["x", "y", "z"]]))
                q = [float(row[k]) for k in ["qw", "qx", "qy", "qz"]]
                orient.Set(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
                info.text = f"Recorded robot replay t={times[tick]:.3f} s | last observation C{index+1}\nRGB/chunk remain tied to that observation, not replay time."
                if elapsed >= times[-1]:
                    state["playing"] = False
            app.update()
            time.sleep(.01)
    except Exception as exc:
        traceback.print_exc()
        write_json(output/"failure.json", {"error": repr(exc)})
    finally:
        app.close()


if __name__ == "__main__":
    main()
