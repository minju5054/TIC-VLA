"""One human-looking USD with a prescribed kinematic crossing path, no crowd AI."""
import csv
import math
import numpy as np
from research.records import write_json


def crossing_position(config, sim_time):
    elapsed = np.clip(sim_time-config["start_time"], 0, config["stop_time"]-config["start_time"])
    return np.asarray(config["start_position"], dtype=float) + elapsed*np.asarray(config["velocity"])


def reveal_position(config, sim_time, trigger_sim_time):
    start = np.asarray(config["hidden_start_position"], dtype=float)
    if trigger_sim_time is None:
        return start
    velocity = np.asarray(config["velocity"], dtype=float)
    speed = np.linalg.norm(velocity)
    if speed <= 0 or velocity[0] != 0 or velocity[2] != 0:
        raise ValueError("Reveal requires a nonzero lateral-only velocity")
    elapsed = np.clip(sim_time-trigger_sim_time-config["reveal_delay_s"], 0,
                      config["max_displacement_m"]/speed)
    return start+elapsed*velocity


class Pedestrian:
    def __init__(self, sim, config):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        self.config, self.sim = config, sim
        self.motion_mode = config.get("motion_mode", "crossing")
        self.trigger_sim_time = None
        if self.motion_mode not in ("crossing", "triggered_lateral_reveal"):
            raise ValueError("Unknown pedestrian motion mode")
        if not config["asset"]:
            raise ValueError("A human asset is required; no silent primitive substitution")
        source = Usd.Stage.Open(config["asset"])
        root = source.GetDefaultPrim()
        unit = UsdGeom.GetStageMetersPerUnit(source)
        up = str(UsdGeom.GetStageUpAxis(source))
        if up not in ("Y", "Z"):
            raise ValueError("Unknown human asset up-axis")
        box = UsdGeom.BBoxCache(0, ["default", "render", "proxy"]).ComputeWorldBound(root).ComputeAlignedRange()
        low, high = np.array(box.GetMin()), np.array(box.GetMax())
        rotation = np.eye(3) if up == "Z" else np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        corners = np.array([[x, y, z] for x in [low[0], high[0]] for y in [low[1], high[1]] for z in [low[2], high[2]]])
        transformed = (rotation @ corners.T).T * unit
        low_m, high_m = transformed.min(0), transformed.max(0)
        height = float(high_m[2]-low_m[2])
        if not 1.3 < height < 2.5:
            raise ValueError(f"Unexpected human height after USD unit conversion: {height}")
        offset = [float(-(low_m[0]+high_m[0])/2), float(-(low_m[1]+high_m[1])/2), float(-low_m[2])]
        prim = UsdGeom.Xform.Define(sim.stage, "/World/Pedestrian")
        self.translate = prim.AddTranslateOp()
        prim.AddRotateZOp().Set(math.degrees(config["yaw"]))
        rigid = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        rigid.CreateKinematicEnabledAttr(True)
        visual = UsdGeom.Xform.Define(sim.stage, "/World/Pedestrian/VisualTransform")
        visual.AddTranslateOp().Set(Gf.Vec3d(*offset))
        if up == "Y":
            visual.AddRotateXOp().Set(90)
        visual.AddScaleOp().Set(Gf.Vec3f(unit, unit, unit))
        model = sim.stage.DefinePrim("/World/Pedestrian/VisualTransform/Model", "Xform")
        model.GetReferences().AddReference(config["asset"])
        capsule = UsdGeom.Capsule.Define(sim.stage, "/World/Pedestrian/Collider")
        capsule.CreateAxisAttr("Z")
        capsule.CreateRadiusAttr(config["collider_radius"])
        capsule.CreateHeightAttr(config["collider_cylinder_height"])
        capsule.AddTranslateOp().Set(Gf.Vec3d(0, 0, height/2))
        capsule.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
        self.file = (sim.records.path/"raw/pedestrian_state.csv").open("x")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["sim_time", "target_x", "target_y", "target_z", "physx_x", "physx_y", "physx_z"])
        self.initial = self.position_at(0)
        self.current = self.initial.copy()
        self.measured_initial = self.measured_current = None
        write_json(sim.records.path/"pedestrian_asset.json", {"url": config["asset"], "source_units": unit,
            "source_up_axis": up, "source_bounds": [low.tolist(), high.tolist()], "height_m": height,
            "visual_translation_m": offset, "source_to_z_up_rotation": rotation.tolist(),
            "motion": "USD kinematic root targets; frozen authored rest pose; no gait/crowd/avoidance AI",
            "collider": {"type": "capsule", "radius": config["collider_radius"], "cylinder_height": config["collider_cylinder_height"]}})
        self.update(0)
        for _ in range(15):
            sim.world.render()

    def update(self, sim_time):
        from pxr import Gf
        self.current = self.position_at(sim_time)
        self.translate.Set(Gf.Vec3d(*self.current))

    def position_at(self, sim_time):
        if self.motion_mode == "crossing":
            return crossing_position(self.config, sim_time)
        return reveal_position(self.config, sim_time, self.trigger_sim_time)

    def on_request_accepted(self, request_id, application):
        if self.motion_mode != "triggered_lateral_reveal" or request_id != self.config["reveal_after_request_id"]:
            return
        if self.trigger_sim_time is not None:
            raise RuntimeError("Reveal trigger must occur exactly once")
        from research.records import clocks
        self.trigger_sim_time = application["sim_time"]
        write_json(self.sim.records.path/"raw/reveal_event.json", {
            "trigger_request_id": request_id, "trigger_sim_time": self.trigger_sim_time,
            "event_source": "main_thread_after_prespecified_request_acceptance", **clocks(),
            "application": application, "measured_human_at_trigger": self.snapshot(),
            "depends_on_prediction_content": False, "motion_config": self.config})

    def measure(self, sim_time):
        import omni.physx
        pose = omni.physx.get_physx_interface().get_rigidbody_transformation("/World/Pedestrian")
        if not pose["ret_val"]:
            raise RuntimeError("Pedestrian rigid-body pose is unavailable from PhysX")
        self.measured_current = np.asarray(pose["position"], dtype=float)
        if self.measured_initial is None:
            self.measured_initial = self.measured_current.copy()
        self.writer.writerow([sim_time, *self.current, *self.measured_current])

    def evidence(self):
        self.file.flush()
        return {"asset": self.config["asset"], "target_initial": self.initial.tolist(), "target_final": self.current.tolist(),
                "measured_initial": self.measured_initial.tolist(), "measured_final": self.measured_current.tolist(),
                "displacement": float(np.linalg.norm(self.measured_current-self.measured_initial)),
                "pose_measurement_type": "PhysX global rigid-body translation sampled after each physics step"}

    def snapshot(self):
        """Read actual PhysX on the main thread, without authoring or advancing time."""
        import omni.physx
        from research.records import clocks
        pose = omni.physx.get_physx_interface().get_rigidbody_transformation("/World/Pedestrian")
        if not pose["ret_val"]:
            raise RuntimeError("Pedestrian rigid-body event pose is unavailable from PhysX")
        return {"position": list(pose["position"]), "sim_time": self.sim.state()["sim_time"],
                "event_source": "simulator_main_thread_PhysX", **clocks()}

    def close(self):
        self.file.close()
