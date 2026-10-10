"""Nova Hospital runtime; reuse existing capture/physics semantics, distinct logs."""
import csv
import math
import numpy as np
from research.sim import Simulation
from research.records import write_json, clocks
from research.nova_source import source_audit
from research.robots.nova_carter import NovaCarter
from research.hospital_scene import HospitalDiagnostics, load_official_scene


class NovaSimulation(Simulation):
    def __init__(self, cfg, records, variant="direct", actor_factory=None):
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from pxr import UsdGeom, UsdPhysics, Gf
        from research.audit_nova_asset import inspect
        self.cfg, self.records, self.mode = cfg, records, "static"
        self.dt = cfg["simulation"]["dt"]
        self.world = World(physics_dt=self.dt, rendering_dt=self.dt, stage_units_in_meters=1.)
        self.stage = self.world.stage
        load_official_scene(self, cfg, records)
        r = cfg["robot"]; source = source_audit(cfg.get("blind_corner_episode_id", "episode_16"))
        if r["asset"] != source["asset"] or r["wheel_joints"] != source["constants"]["_wheel_joints"]:
            raise ValueError("Nova config does not match audited source")
        self.robot = self.world.scene.add(WheeledRobot(prim_path=r["prim"], name="research_nova",
            wheel_dof_names=r["wheel_joints"], create_robot=True, usd_path=r["asset"],
            position=np.asarray(r["start_position"]),
            orientation=np.array([math.cos(r["start_yaw"]/2), 0, 0, math.sin(r["start_yaw"]/2)])))
        camera = self.stage.GetPrimAtPath(cfg["camera"]["prim"])
        if not camera or not camera.IsA(UsdGeom.Camera):
            raise RuntimeError("Source front Hawk camera missing; no fabricated camera fallback")
        asset = inspect(self.stage, r["prim"], source)
        # Authored cylinder extent is stale (+/-50); use radius/height and actual
        # transforms, never that broad-phase display bound, to audit geometry.
        wheels = []
        body_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(r["body"])).ComputeLocalToWorldTransform(0)
        for joint_name, cylinder_path in zip(r["wheel_joints"], r["wheel_collision_prims"]):
            joint = UsdPhysics.RevoluteJoint(self.stage.GetPrimAtPath(r["prim"]+"/"+joint_name))
            rot = Gf.Rotation(Gf.Quatd(joint.GetLocalRot0Attr().Get()))
            axis = rot.TransformDir(Gf.Vec3d(0, 0, 1))
            cylinder = UsdGeom.Cylinder(self.stage.GetPrimAtPath(cylinder_path))
            matrix = UsdGeom.Xformable(cylinder).ComputeLocalToWorldTransform(0)*body_xform.GetInverse()
            transform = np.asarray(matrix)
            radius = float(cylinder.GetRadiusAttr().Get())
            if (joint.GetAxisAttr().Get() != "Z" or not np.allclose(axis, [0, 1, 0], atol=1e-6)
                    or not np.allclose(np.linalg.norm(transform[:3, :3], axis=1), 1., atol=1e-6)
                    or not np.isclose(radius, r["wheel_radius"], atol=1e-6)):
                raise ValueError("Loaded Nova wheel axis/scale/radius mismatch")
            wheels.append({"joint": joint_name, "axis_body": list(axis), "collision_radius": radius,
                           "center_body": transform[3, :3].tolist(), "collision_prim": cylinder_path})
        base = abs(wheels[0]["center_body"][1]-wheels[1]["center_body"][1])
        if not np.isclose(base, r["wheel_base"], atol=1e-6):
            raise ValueError("Loaded Nova wheel-center separation mismatch")
        camera_matrix = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(0)*body_xform.GetInverse()
        if not np.allclose(np.asarray(camera_matrix)[3, :3], cfg["camera"]["translation"], atol=1e-6):
            raise ValueError("Source camera translation differs from verified config")
        write_json(records.path/"robot_asset.json", {**asset, "verified_drive_wheels": wheels, "verified_wheel_base": base,
            "controller_radius": r["wheel_radius"], "controller_base": r["wheel_base"],
            "camera_body_transform": np.asarray(camera_matrix).tolist(),
            "geometry_note": "Source defaults .152/.413 differ from loaded analytical collision radius .14 / center separation .4132; no asset tuning"})
        self.hospital_diagnostics = HospitalDiagnostics(self)
        self.product = rep.create.render_product(cfg["camera"]["prim"], tuple(cfg["camera"]["resolution"]))
        self.rgb = rep.AnnotatorRegistry.get_annotator("rgb"); self.rgb.attach([self.product])
        # Optional actor is authored before reset/settling, so it exists from
        # simulation start without an extra post-origin initialization tick.
        actor = actor_factory(self) if actor_factory else None
        self.world.reset()
        self.adapter = NovaCarter(self.robot, r, source["constants"], variant)
        for _ in range(cfg["simulation"]["settle_steps"]):
            self.adapter.stop(); self.world.step(render=False)
        for _ in range(8): self.world.render()
        self.origin_time = float(self.world.current_time); self.tick = 0
        self.command = [0., 0.]; self.wheel_command = [0., 0.]
        self.continuous = True; self.pedestrian = actor
        self.active_control_source_request_id = None; self.pending_request_id = None
        self.physics_step_start_monotonic_ns = None
        self.source_tick = None; self.phase = "initial"; self.primary_window = False
        self.previous_logged_state = None
        self.state_file = (records.path/"robot_state.csv").open("x")
        self.writer = csv.DictWriter(self.state_file, fieldnames=["tick", "sim_time", "isaac_sim_time", "wall_time_ns", "monotonic_ns",
            "physics_step_start_monotonic_ns", "x", "y", "z", "yaw", "qw", "qx", "qy", "qz",
            "vx_world", "vy_world", "vz_world", "wx_world", "wy_world", "wz_world", "body_v_forward", "body_v_left", "body_v_up",
            "body_yaw_rate_measured", "pose_yaw_rate", "target_v", "target_w", "applied_v", "applied_w",
            "wheel_target_left", "wheel_target_right", "wheel_measured_left", "wheel_measured_right",
            "active_control_source_request_id", "pending_request_id", "source_tick", "phase", "primary_window"])
        self.writer.writeheader(); self.log_state()
        self.audit_spawn()
        if self.pedestrian:
            self.pedestrian.measure(self.state()["sim_time"])

    def state(self):
        return {**self.adapter.state(), "sim_time": float(self.world.current_time)-self.origin_time,
                "isaac_sim_time": float(self.world.current_time), "tick": self.tick, **clocks()}

    def apply(self, v, w):
        self.command = [float(v), float(w)]
        self.wheel_command = self.adapter.apply_target(v, w)
        return list(self.wheel_command)

    def step(self):
        self.adapter.before_step(self.dt)
        self.wheel_command = list(self.adapter.wheel_targets)
        return super().step()

    def log_state(self):
        s = self.state(); pose_rate = None
        if self.previous_logged_state is not None:
            old = self.previous_logged_state
            delta = math.atan2(math.sin(s["pose"][2]-old["pose"][2]), math.cos(s["pose"][2]-old["pose"][2]))
            pose_rate = delta/(s["sim_time"]-old["sim_time"])
        values = [s["tick"], s["sim_time"], s["isaac_sim_time"], s["wall_time_ns"], s["monotonic_ns"],
            self.physics_step_start_monotonic_ns, *s["position"], s["pose"][2], *s["quaternion_wxyz"],
            *s["linear_velocity_world"], *s["angular_velocity_world"], *s["linear_velocity_body_flu"],
            s["angular_velocity_body_flu"][2], pose_rate, *self.adapter.target.tolist(), *self.adapter.applied.tolist(),
            *self.adapter.wheel_targets, *s["wheel_velocities"], self.active_control_source_request_id,
            self.pending_request_id, self.source_tick, self.phase, self.primary_window]
        self.writer.writerow(dict(zip(self.writer.fieldnames, values))); self.previous_logged_state = s
        return s

    def audit_spawn(self):
        state = self.state(); contacts = self.hospital_diagnostics.contacts
        active = {}
        for e in contacts:
            key = (e["collider0"], e["collider1"])
            if "CONTACT_LOST" in e["type"]: active.pop(key, None)
            elif e["contacts"]: active[key] = e
        supported = [side for side in ("wheel_left", "wheel_right") if any(
            any(f"/{side}/" in e[k] for k in ("collider0", "collider1")) and
            any(abs(c["normal"][2]) > .9 for c in e["contacts"]) for e in active.values())]
        nonfloor = [e for e in active.values() if any(abs(c["normal"][2]) < .5 for c in e["contacts"])
                    and not all(e[k].startswith(self.cfg["robot"]["prim"]+"/") for k in ("collider0", "collider1"))]
        drift = float(np.linalg.norm(np.array(state["position"][:2])-self.cfg["robot"]["start_position"][:2]))
        camera = self.hospital_diagnostics.camera_query(state)
        valid = len(supported)==2 and not nonfloor and drift < .02 and not camera["camera_near_scene_collision_paths"]
        write_json(self.records.path/"nova_spawn_audit.json", {"state": state, "wheel_support": supported,
            "initial_nonfloor_contacts": nonfloor, "settling_xy_drift_m": drift, **camera, "placement_valid": valid,
            "spawn_z_note": "Official runner adds 0.1 m to episode Z=0.01; applied 0.11 m"})
        if not valid: raise RuntimeError("Nova spawn/floor audit failed before model call")

    def capture(self, name):
        try:
            return super().capture(name)
        except RuntimeError as exc:
            if not str(exc).startswith("Invalid/blank RGB:"): raise
            # Preserve the rejected *same annotator buffer*, without another
            # render, guard relaxation, command change or model call.
            from PIL import Image
            data=np.asarray(self.rgb.get_data())[:,:,:3].copy()
            state=self.state()
            with (self.records.path/"diagnostics"/("rejected_"+name)).open("xb") as f:
                Image.fromarray(data).save(f,format="PNG")
            self.hospital_diagnostics.capture(state,data,"rejected_"+name)
            write_json(self.records.path/"capture_failure.json",{"attempted_image":name,"observation":state,
                "reason":str(exc),"rgb_std":float(data.std()),"not_submitted_to_model":True})
            raise

    def close(self):
        self.adapter.stop()
        if self.pedestrian:
            self.pedestrian.close()
        self.hospital_diagnostics.close(); self.state_file.close(); self.world.stop()
