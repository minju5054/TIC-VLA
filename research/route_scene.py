"""Read-only USD geometry audit for the additive two-gap scene."""
import numpy as np
from research.records import write_json
from research.route_geometry import rectangle, segment_intersects


def audit_scene(sim, cfg, records):
    from pxr import Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], False, True)
    body = sim.stage.GetPrimAtPath(cfg["robot"]["prim"])
    bounds = []
    origin = np.array(cfg["robot"]["start_position"])
    if cfg["robot"]["start_yaw"] != 0:
        raise ValueError("This footprint audit requires the prespecified zero-yaw start")
    for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not box.IsEmpty():
                bounds.append({"path": str(prim.GetPath()), "min_body": (np.array(box.GetMin())-origin).tolist(),
                               "max_body": (np.array(box.GetMax())-origin).tolist()})
    if not bounds:
        raise RuntimeError("No verified Jackal collision bounds")
    lo = np.min([b["min_body"] for b in bounds], axis=0)
    hi = np.max([b["max_body"] for b in bounds], axis=0)
    radius = float(np.linalg.norm(np.maximum(np.abs(lo[:2]), np.abs(hi[:2]))))
    declared = cfg["route_switch"]["robot_footprint_radius_m"]
    if declared+1e-6 < radius:
        raise RuntimeError("Declared footprint does not conservatively enclose robot collisions")
    blocker = next(o for o in cfg["scene"]["obstacles"] if o["name"] == "central_blocker")
    rect = rectangle(blocker)
    inner_wall = cfg["scene"]["corridor_half_width"]-.05
    gaps = {"left_physical_width_m": float(inner_wall-rect[3]),
            "right_physical_width_m": float(inner_wall+rect[1])}
    if min(gaps.values()) <= 2*declared:
        raise RuntimeError("Both gaps must admit the conservative robot diameter")
    human = cfg["pedestrian"]["hidden_start_position"]
    human_radius = cfg["pedestrian"]["collider_radius"]
    if segment_intersects(human[:2], human[:2], rectangle(blocker, human_radius)):
        raise RuntimeError("Hidden human overlaps central collision geometry")
    camera_xy = (origin+cfg["camera"]["translation"])[:2]
    hidden = segment_intersects(camera_xy, human[:2], rect)
    if not hidden:
        raise RuntimeError("Initial center-to-center XY line of sight is not occluded")
    if blocker["size"][2] < 2.0:
        raise RuntimeError("Blocker does not cover the known human height")
    write_json(records.path/"route_scene_geometry.json", {
        "collision_bounds": bounds, "collision_union_min_body": lo.tolist(), "collision_union_max_body": hi.tolist(),
        "measured_circumscribed_radius_m": radius, "analysis_footprint_radius_m": declared,
        "footprint_note": "yaw-independent circumscribed disk; rectangle inflation is conservative at corners",
        "central_blocker_rectangle_xy": rect.tolist(), "corridor_inner_wall_abs_y": inner_wall, **gaps,
        "initial_camera_world_xy": camera_xy.tolist(), "hidden_human_xyz": human,
        "geometrically_occluded_by_central_blocker": hidden,
        "occlusion_note": "geometric XY center-line proxy, not pixel-level visibility",
        "hidden_human_does_not_overlap_blocker": True,
        "decision_gate_x": cfg["route_switch"]["decision_gate_x"]})
