"""Official Hospital sublayer and observational PhysX diagnostics; no control changes."""
import json
import numpy as np
from research.records import write_json
from research.hospital_episode import validate_config
from research.control import rotation_wxyz


def load_official_scene(sim, cfg, records):
    from pxr import Sdf, Usd, UsdGeom, UsdLux
    source = validate_config(cfg)
    url = cfg["scene"]["usd"]
    layer = Sdf.Layer.FindOrOpen(url)
    if not layer:
        raise RuntimeError("Official Hospital USD could not be opened")
    original = Usd.Stage.Open(layer, load=Usd.Stage.LoadNone)
    units, up = UsdGeom.GetStageMetersPerUnit(original), str(UsdGeom.GetStageUpAxis(original))
    if not np.isclose(units, 1) or up != "Z":
        raise ValueError("Official source units/up axis require investigation; no automatic rescaling")
    root = original.GetDefaultPrim()
    if not root:
        raise ValueError("Hospital source has no default prim")
    root_path = str(root.GetPath())
    source_transform = np.array(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    # Sublayer the full source, preserving all original world paths/transforms.
    # No ground, wall, goal marker, light or scene transform is added here.
    sim.stage.GetRootLayer().subLayerPaths.append(url)
    composed = sim.stage.GetPrimAtPath(root_path)
    world_transform = np.array(UsdGeom.Xformable(composed).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    if not np.allclose(source_transform, world_transform, atol=1e-12):
        raise ValueError("Official scene root transform changed during composition")
    meshes, lights, semantic_paths = 0, 0, []
    for prim in sim.stage.Traverse():
        meshes += prim.IsA(UsdGeom.Mesh)
        lights += prim.HasAPI(UsdLux.LightAPI)
        if any(word in str(prim.GetPath()).lower() for word in ("stair", "bed", "hall", "door", "floor", "vending")):
            semantic_paths.append(str(prim.GetPath()))
    if meshes == 0:
        raise ValueError("Hospital loaded without mesh geometry")
    result = {"source": source, "hospital_url": url, "composition": "full original URL as sublayer, no added scene transform",
        "source_meters_per_unit": units, "source_up_axis": up,
        "composed_meters_per_unit": UsdGeom.GetStageMetersPerUnit(sim.stage),
        "composed_up_axis": str(UsdGeom.GetStageUpAxis(sim.stage)), "source_default_prim": root_path,
        "source_root_world_transform": source_transform.tolist(), "composed_root_world_transform": world_transform.tolist(),
        "mesh_count": meshes, "authored_light_count": lights, "semantic_prim_paths": semantic_paths,
        "custom_ground_walls_goal_obstacles_created": False,
        "official_start": source["episode"]["start"],
        **({"nova_spawn_position": cfg["robot"]["start_position"]} if cfg["robot"].get("type") == "nova_carter"
           else {"jackal_spawn_position": cfg["robot"]["start_position"]}),
        "applied_yaw_radians": cfg["robot"]["start_yaw"], "goal_world_position": cfg["scene"]["goal"]}
    write_json(records.path/"hospital_scene.json", result)


class HospitalDiagnostics:
    """Contact reporting and capture-time queries only; never modify commands."""
    def __init__(self, sim):
        from pxr import Usd, UsdPhysics, PhysxSchema
        from omni.physx import get_physx_simulation_interface
        self.sim = sim
        self.contacts = []
        self.errors = []
        self.file = (sim.records.path/"raw/robot_contacts.jsonl").open("x")
        self.observations = sim.records.path/"raw/scene_observations"
        self.observations.mkdir()
        for prim in Usd.PrimRange(sim.stage.GetPrimAtPath(sim.cfg["robot"]["prim"]), Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.IsInstanceProxy():
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contacts)

    def on_contacts(self, headers, data):
        from pxr import PhysicsSchemaTools
        try:
            for h in headers:
                paths = [str(PhysicsSchemaTools.intToSdfPath(getattr(h, key))) for key in ("actor0", "actor1", "collider0", "collider1")]
                if not any(p.startswith(self.sim.cfg["robot"]["prim"]+"/") for p in paths):
                    continue
                stamp = float(self.sim.world.current_time)
                entry = {"callback_isaac_sim_time": stamp,
                    "sim_time": stamp-self.sim.origin_time if hasattr(self.sim, "origin_time") else None,
                    "phase": "run" if hasattr(self.sim, "origin_time") else "settling",
                    "type": str(h.type), "actor0": paths[0], "actor1": paths[1], "collider0": paths[2], "collider1": paths[3],
                    "contacts": [{"position": list(map(float, c.position)), "normal": list(map(float, c.normal)),
                                  "impulse": list(map(float, c.impulse)), "separation": float(c.separation)}
                                 for c in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]]}
                self.file.write(json.dumps(entry, allow_nan=False)+"\n")
                self.contacts.append(entry)
        except Exception as exc:
            self.errors.append(repr(exc))

    def camera_query(self, state):
        import carb
        from omni.physx import get_physx_scene_query_interface
        p = np.asarray(state["position"])+rotation_wxyz(state["quaternion_wxyz"])@self.sim.cfg["camera"]["translation"]
        hits = []
        def callback(hit):
            path = str(hit.rigid_body)
            if not path.startswith(self.sim.cfg["robot"]["prim"]):
                hits.append(path)
            return True
        radius = self.sim.cfg["hospital_validation"]["camera_probe_radius_m"]
        get_physx_scene_query_interface().overlap_sphere(radius, carb.Float3(*map(float, p)), callback, False)
        return {"camera_world_xyz": p.tolist(), "camera_near_scene_collision_paths": sorted(set(hits)),
                "probe_radius_m": radius, "query_note": "PhysX overlap proximity; not a general closed-mesh inside/outside classifier"}

    def capture(self, state, data, name):
        self.file.flush()
        if self.errors:
            raise RuntimeError("Hospital contact logger failed: "+str(self.errors[:3]))
        result = {"observation_sim_time": state["sim_time"], **self.camera_query(state),
            "rgb_mean_0_255": float(data.mean()), "rgb_std_0_255": float(data.std()),
            "rgb_p95_0_255": float(np.percentile(data, 95)),
            "degraded_near_black": bool(np.percentile(data, 95) <= self.sim.cfg["hospital_validation"]["near_black_p95_0_255"])}
        write_json(self.observations/(name+".json"), result)
        return result

    def close(self):
        self.subscription = None
        self.file.close()


def audit_start(sim):
    import carb
    from omni.physx import get_physx_scene_query_interface
    state = sim.state()
    cfg = sim.cfg
    diagnostics = sim.hospital_diagnostics
    if diagnostics.errors:
        raise RuntimeError(str(diagnostics.errors))
    # Sleeping rigid bodies stop emitting PERSIST callbacks. Retain the last
    # contact until an explicit LOST event; independently check settled geometry.
    active = {}
    for e in diagnostics.contacts:
        key = (e["collider0"], e["collider1"])
        if "CONTACT_LOST" in e["type"]:
            active.pop(key, None)
        elif e["contacts"]:
            active[key] = e
    supported = sorted({p for e in active.values() for p in [e["collider0"], e["collider1"]]
                        if "wheel" in p and any(abs(c["normal"][2]) > .9 for c in e["contacts"])})
    wheel_names = cfg["robot"]["wheel_joints"]
    wheels_supported = all(any(name.replace("_wheel", "_wheel_link") in path for path in supported) for name in wheel_names)
    spawn_xy = np.array(cfg["robot"]["start_position"][:2])
    displacement = float(np.linalg.norm(np.array(state["position"][:2])-spawn_xy))
    query = diagnostics.camera_query(state)
    floor_samples = []
    for dx, dy in [(0.45, 0), (-.45, 0), (0, .45), (0, -.45)]:
        origin = [float(spawn_xy[0]+dx), float(spawn_xy[1]+dy), state["position"][2]+.3]
        hit = get_physx_scene_query_interface().raycast_closest(carb.Float3(*origin), carb.Float3(0,0,-1), 2.)
        floor_samples.append({"origin": origin, "hit": bool(hit["hit"]),
            "position": list(map(float, hit["position"])) if hit["hit"] else None,
            "collision": str(hit["collision"]) if hit["hit"] else None})
    floor_z = float(np.median([h["position"][2] for h in floor_samples if h["hit"]]))
    asset = json.loads((sim.records.path/"robot_asset.json").read_text())
    bottoms = [(np.asarray(state["position"])+rotation_wxyz(state["quaternion_wxyz"])@w["local_position"])[2]
               -w["collision_radius"] for w in asset["wheels"]]
    floor_errors = [float(z-floor_z) for z in bottoms]
    result = {"official_episode_z": cfg["official_episode"]["episode"]["start"][2],
        "jackal_spawn_z": cfg["robot"]["start_position"][2], "measured_settled_state": state,
        "settling_xy_displacement_m": displacement, "wheels_with_upward_contact": supported,
        "all_four_wheels_supported": wheels_supported, "nearby_downward_floor_rays": floor_samples, **query,
        "wheel_bottom_minus_floor_m": floor_errors, "floor_height_m": floor_z,
        "contact_note": "Last contact remains active until CONTACT_LOST; sleeping bodies need not emit repeated callbacks",
        "settled_xy_within_prespecified_tolerance": displacement <= cfg["hospital_validation"]["maximum_settling_xy_m"]}
    result["placement_valid"] = bool(wheels_supported and result["settled_xy_within_prespecified_tolerance"]
                                      and max(map(abs, floor_errors)) < .005
                                      and not query["camera_near_scene_collision_paths"])
    write_json(sim.records.path/"hospital_start_audit.json", result)
    if not result["placement_valid"]:
        raise RuntimeError("Hospital robot placement audit failed; inspect saved audit before model inference")
    return result
