"""Staged, controlled Isaac 6 experiment. Run via scripts/isaac6_python.sh."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import traceback

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(REPO/"configs/research/crossing_pedestrian.yaml"))
    parser.add_argument("--mode", choices=["wheel", "inference", "static", "dynamic"], required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--prerequisite-run")
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id or args.run_id in (".", ".."):
        parser.error("run-id must be a single directory name")
    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())
    if args.mode != "wheel":
        expected = {"inference": "wheel", "static": "inference", "dynamic": "static"}[args.mode]
        if not args.prerequisite_run:
            parser.error("--prerequisite-run is required")
        prior = json.loads((REPO/"outputs"/args.prerequisite_run/"summary.json").read_text())
        if prior["mode"] != expected or prior["status"] != "PASS":
            parser.error(f"A PASS {expected} run is required")
    from research.records import RunRecords, provenance, write_json
    records = RunRecords(REPO/"outputs"/args.run_id,
                         {"schema_version": 1, "mode": args.mode, "config": cfg,
                          "command_argv": sys.argv, "prerequisite_run": args.prerequisite_run, **provenance(REPO)})
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": cfg["simulation"]["headless"], "renderer": "RayTracedLighting",
                         "width": cfg["camera"]["resolution"][0], "height": cfg["camera"]["resolution"][1]})
    code = 1
    sim = None
    try:
        print("SIMULATION_APP_READY", flush=True)
        import numpy as np
        runtime_root = Path(os.environ["ISAAC_PATH"])
        write_json(records.path/"simulation-runtime.json", {"python": sys.version, "executable": sys.executable,
            "isaac_root": str(runtime_root), "isaac_version": (runtime_root/"VERSION").read_text().strip(),
            "numpy": np.__version__, "renderer": "RayTracedLighting", "physics_dt": cfg["simulation"]["dt"]})
        sim = Simulation(cfg, records, args.mode)
        if args.mode == "wheel":
            result = sim.validate_wheels()
        else:
            from research.loop import run_model_loop
            result = run_model_loop(sim, cfg, records, args.mode)
        write_json(records.path/"summary.json", {"mode": args.mode, "status": "PASS", **result})
        print("RUN_PASS", json.dumps(result), flush=True)
        code = 0
    except Exception as exc:
        traceback.print_exc()
        write_json(records.path/"failure.json", {"mode": args.mode, "status": "FAIL", "error": repr(exc)})
    finally:
        if sim:
            sim.close()
        app.close(exit_code=code)


class Simulation:
    def __init__(self, cfg, records, mode):
        import numpy as np
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
        from research.records import write_json
        self.cfg, self.records, self.mode = cfg, records, mode
        self.dt = cfg["simulation"]["dt"]
        self.world = World(physics_dt=self.dt, rendering_dt=self.dt, stage_units_in_meters=1.0)
        self.stage = self.world.stage
        self.world.scene.add_default_ground_plane()
        light = UsdLux.DomeLight.Define(self.stage, "/World/Light")
        light.CreateIntensityAttr(800)
        r = cfg["robot"]
        q = np.array([math.cos(r["start_yaw"]/2), 0, 0, math.sin(r["start_yaw"]/2)])
        self.robot = self.world.scene.add(WheeledRobot(
            prim_path=r["prim"], name="research_jackal", wheel_dof_names=r["wheel_joints"],
            create_robot=True, usd_path=r["asset"], position=np.array(r["start_position"]), orientation=q))
        # Verify actual USD geometry before using config kinematics.
        geometries = []
        for name, sign in zip(r["wheel_joints"], r["joint_signs"]):
            joint = UsdPhysics.RevoluteJoint(self.stage.GetPrimAtPath(r["body"]+"/"+name))
            if not joint:
                raise RuntimeError("Missing wheel joint: "+name)
            pos = joint.GetLocalPos0Attr().Get()
            rot = joint.GetLocalRot0Attr().Get()
            axis = Gf.Rotation(Gf.Quatd(rot)).TransformDir(Gf.Vec3d(1, 0, 0))
            if joint.GetAxisAttr().Get() != "X" or not np.allclose(axis, [0, sign, 0], atol=1e-5):
                raise RuntimeError("Wheel joint sign/axis differs from configuration")
            body = self.stage.GetPrimAtPath(str(joint.GetBody1Rel().GetTargets()[0]))
            box = UsdGeom.BBoxCache(0, ["default", "render", "proxy"]).ComputeLocalBound(body).ComputeAlignedRange()
            extent = box.GetSize()
            visual_radius = max(float(extent[0]), float(extent[2]))/2
            collision = UsdGeom.Cylinder(self.stage.GetPrimAtPath(str(body.GetPath())+"/collisions"))
            if not collision:
                raise RuntimeError("Expected Jackal wheel collision cylinder")
            radius = float(collision.GetRadiusAttr().Get())
            if not np.isclose(radius, r["wheel_radius"], atol=1e-4):
                raise RuntimeError(f"Wheel radius mismatch: USD {radius}, config {r['wheel_radius']}")
            geometries.append({"joint": name, "local_position": list(pos), "axis_body": list(axis),
                               "mesh_bound_size": list(extent), "visual_radius_from_mesh": visual_radius,
                               "collision_radius": radius})
        track = abs(geometries[0]["local_position"][1]-geometries[2]["local_position"][1])
        if not np.isclose(track, r["track_width"], atol=1e-6):
            raise RuntimeError("USD track width differs from configuration")
        write_json(records.path/"robot_asset.json", {"url": r["asset"], "wheels": geometries,
                   "track_width": track, "meters_per_unit": UsdGeom.GetStageMetersPerUnit(self.stage)})
        if mode != "wheel":
            half = cfg["scene"]["corridor_half_width"]
            height = cfg["scene"]["wall_height"]
            self.cube("/World/LeftWall", [4, half, height/2], [10, .1, height], [.55, .55, .6])
            self.cube("/World/RightWall", [4, -half, height/2], [10, .1, height], [.55, .55, .6])
            goal = cfg["scene"]["goal"]
            self.cube("/World/GoalWall", [goal[0], goal[1], goal[2]+height/2], [.1, 2, height], [.1, .65, .1])
        camcfg = cfg["camera"]
        camera = UsdGeom.Camera.Define(self.stage, camcfg["prim"])
        camera.AddTranslateOp().Set(Gf.Vec3d(*camcfg["translation"]))
        qcam = camcfg["quaternion_wxyz"]
        camera.AddOrientOp().Set(Gf.Quatf(qcam[0], Gf.Vec3f(*qcam[1:])))
        camera.CreateFocalLengthAttr(camcfg["focal_length_mm"])
        camera.CreateHorizontalApertureAttr(camcfg["horizontal_aperture_mm"])
        camera.CreateVerticalApertureAttr(camcfg["vertical_aperture_mm"])
        camera.CreateClippingRangeAttr(Gf.Vec2f(*camcfg["clipping_range"]))
        self.product = rep.create.render_product(camcfg["prim"], tuple(camcfg["resolution"]))
        self.rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb.attach([self.product])
        self.world.reset()
        for _ in range(cfg["simulation"]["settle_steps"]):
            self.world.step(render=False)
        for _ in range(8):
            self.world.render()
        self.origin_time = float(self.world.current_time)
        self.tick = 0
        self.command = [0.0, 0.0]
        self.wheel_command = [0.0]*4
        self.pedestrian = None
        self.state_file = (records.path/"robot_state.csv").open("x")
        self.writer = csv.DictWriter(self.state_file, fieldnames=["tick", "sim_time", "isaac_sim_time", "wall_time_ns",
            "x", "y", "z", "yaw", "qw", "qx", "qy", "qz", "vx_world", "vy_world", "vz_world",
            "wx_world", "wy_world", "wz_world", "wheel_fl", "wheel_rl", "wheel_fr", "wheel_rr",
            "command_v", "command_w", "command_fl", "command_rl", "command_fr", "command_rr"])
        self.writer.writeheader()
        self.log_state()
        print("ROBOT_READY", self.robot.dof_names, flush=True)

    def cube(self, path, position, scale, color):
        from pxr import UsdGeom, UsdPhysics, Gf
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*scale))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return cube

    def state(self):
        import numpy as np
        from research.control import planar_pose, rotation_wxyz
        from research.records import clocks
        p, q = self.robot.get_world_pose()
        lin, ang = self.robot.get_linear_velocity(), self.robot.get_angular_velocity()
        indices = [self.robot.get_dof_index(n) for n in self.cfg["robot"]["wheel_joints"]]
        wheels = self.robot.get_joint_velocities()[indices]
        rot = rotation_wxyz(q)
        return {"sim_time": float(self.world.current_time)-self.origin_time,
                "isaac_sim_time": float(self.world.current_time), "tick": self.tick,
                "position": p.tolist(), "quaternion_wxyz": q.tolist(), "pose": planar_pose(p, q),
                "linear_velocity_world": lin.tolist(), "angular_velocity_world": ang.tolist(),
                "linear_velocity_body_flu": (rot.T@lin).tolist(),
                "angular_velocity_body_flu": (rot.T@ang).tolist(), "wheel_velocities": wheels.tolist(), **clocks()}

    def log_state(self):
        s = self.state()
        values = [s["tick"], s["sim_time"], s["isaac_sim_time"], s["wall_time_ns"], *s["position"], s["pose"][2],
                  *s["quaternion_wxyz"], *s["linear_velocity_world"], *s["angular_velocity_world"],
                  *s["wheel_velocities"], *self.command, *self.wheel_command]
        self.writer.writerow(dict(zip(self.writer.fieldnames, values)))
        return s

    def apply(self, v, w):
        from research.control import wheel_speeds
        from isaacsim.core.utils.types import ArticulationAction
        r = self.cfg["robot"]
        speeds = wheel_speeds(v, w, r["wheel_radius"], r["track_width"], r["joint_signs"])
        self.robot.apply_wheel_actions(ArticulationAction(joint_velocities=speeds))
        self.command, self.wheel_command = [v, w], speeds.tolist()
        return self.wheel_command

    def step(self):
        before = float(self.world.current_time)
        if self.pedestrian:
            self.pedestrian.update(self.state()["sim_time"] + self.dt)
        self.world.step(render=False)
        self.tick += 1
        if self.pedestrian:
            self.pedestrian.measure(self.state()["sim_time"])
        if abs(float(self.world.current_time)-before-self.dt) > 1e-6:
            raise RuntimeError("Physics step time differs from configured dt")
        return self.log_state()

    def capture(self, name):
        import numpy as np
        import carb
        import omni.replicator.core as rep
        from PIL import Image
        before = self.state()
        # A few world.render() calls can leave annotator data at a previous pose.
        # Explicitly synchronize annotators without advancing simulation time.
        self.world.render() # Flush physics transforms into Fabric before capture.
        carb.settings.get_settings().set("/rtx-transient/post/dlss/forceParamReset", True)
        rep.orchestrator.step(rt_subframes=self.cfg["camera"]["capture_subframes"],
                              delta_time=0.0, pause_timeline=False, wait_for_render=True)
        after = self.state()
        if abs(after["sim_time"]-before["sim_time"]) > 1e-9:
            raise RuntimeError("Rendering unexpectedly advanced physics")
        data = np.asarray(self.rgb.get_data())[:, :, :3].copy()
        if data.shape != (self.cfg["camera"]["resolution"][1], self.cfg["camera"]["resolution"][0], 3) or data.std() < 1:
            raise RuntimeError(f"Invalid/blank RGB: {data.shape}")
        path = self.records.path/"diagnostics"/name
        with path.open("xb") as f:
            Image.fromarray(data).save(f, format="PNG")
        return data, after, path

    def validate_wheels(self):
        import numpy as np
        cfg = self.cfg["validation"]
        initial = self.state()
        self.capture("rgb_initial.png")
        self.apply(cfg["straight_v"], 0)
        for _ in range(round(cfg["straight_seconds"]/self.dt)):
            self.step()
        straight = self.state()
        forward = straight["position"][0]-initial["position"][0]
        if forward < cfg["minimum_forward_displacement"]:
            raise RuntimeError(f"Straight test failed: {forward}")
        self.apply(0, cfg["turn_w"])
        for _ in range(round(cfg["turn_seconds"]/self.dt)):
            self.step()
        turned = self.state()
        yaw_change = math.atan2(math.sin(turned["pose"][2]-straight["pose"][2]), math.cos(turned["pose"][2]-straight["pose"][2]))
        if yaw_change < cfg["minimum_yaw_change"]:
            raise RuntimeError(f"Positive turn test failed: {yaw_change}")
        self.capture("rgb_after_turn.png")
        self.apply(0, 0)
        return {"straight_forward_displacement": forward, "positive_turn_yaw_change": yaw_change,
                "initial": initial, "after_straight": straight, "after_turn": turned}

    def close(self):
        self.apply(0, 0)
        if self.pedestrian:
            self.pedestrian.close()
        self.state_file.close()
        self.world.stop()


if __name__ == "__main__":
    main()
