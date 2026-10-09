"""USD/RGB preflight only: constructs the static scene, never imports the model loop."""
import argparse
import json
from pathlib import Path
import sys
import traceback

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def audit_scene(sim, cfg, records):
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics
    from research.records import write_json
    from research.route_geometry import rectangle, segment_intersects, scene_obstacles

    params = cfg["static_detour"]
    radius = params["robot_footprint_radius_m"]
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], False, True)
    origin = np.asarray(cfg["robot"]["start_position"])
    if cfg["robot"]["start_yaw"] != 0:
        raise ValueError("Footprint audit requires the fixed zero-yaw start")
    bounds = []
    for prim in Usd.PrimRange(sim.stage.GetPrimAtPath(cfg["robot"]["prim"]), Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not box.IsEmpty():
                bounds.append({"path": str(prim.GetPath()), "min_body": (np.array(box.GetMin())-origin).tolist(),
                               "max_body": (np.array(box.GetMax())-origin).tolist()})
    if not bounds:
        raise ValueError("No Jackal collision bounds")
    lo = np.min([b["min_body"] for b in bounds], axis=0)
    hi = np.max([b["max_body"] for b in bounds], axis=0)
    measured_radius = float(np.linalg.norm(np.maximum(np.abs(lo[:2]), np.abs(hi[:2]))))
    units = UsdGeom.GetStageMetersPerUnit(sim.stage)
    if not np.isclose(units, 1) or measured_radius > radius+1e-6:
        raise ValueError("Unit or conservative footprint audit failed")
    obstacle_bounds = []
    for obstacle in scene_obstacles(cfg):
        prim = sim.stage.GetPrimAtPath("/World/"+obstacle["name"])
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            raise ValueError("Static collision disabled")
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        center, half = np.array(obstacle["position"]), np.array(obstacle["size"])/2
        if not (np.allclose(box.GetMin(), center-half, atol=1e-6) and np.allclose(box.GetMax(), center+half, atol=1e-6)):
            raise ValueError("Authored collision bounds differ from analysis geometry")
        obstacle_bounds.append({"name": obstacle["name"], "min": list(box.GetMin()), "max": list(box.GetMax())})
    blocker = next(o for o in cfg["scene"]["obstacles"] if o["name"] == "offset_blocker")
    physical, inflated = rectangle(blocker), rectangle(blocker, radius)
    inner = cfg["scene"]["corridor_half_width"]-.05
    left_width = float(inner-physical[3])
    left_interval = [float(inflated[3]), float(inner-radius)]
    centerline_blocked = segment_intersects(origin[:2], cfg["scene"]["goal"][:2], physical)
    camera = (origin+cfg["camera"]["translation"])[:2]
    # Conservative horizontal line-of-sight samples; visibility above the blocker
    # is not counted. RGB is reviewed separately before the model run.
    goal = cfg["scene"]["goal"]
    probes = [[goal[0]-.05, float(y)] for y in np.linspace(goal[1]-.9, goal[1]+.9, 19)]
    visible = [p for p in probes if not segment_intersects(camera, p, physical)]
    if not centerline_blocked or left_width <= 2*radius or not visible:
        raise ValueError("Centerline/gap/goal visibility preflight failed")
    result = {"meters_per_unit": units, "collision_bounds": bounds,
        "collision_union_min_body": lo.tolist(), "collision_union_max_body": hi.tolist(),
        "measured_circumscribed_radius_m": measured_radius, "analysis_footprint_radius_m": radius,
        "footprint_note": "yaw-independent disk enclosed by conservative axis-aligned rectangle inflation",
        "static_collision_bounds": obstacle_bounds, "physical_blocker_rectangle_xy": physical.tolist(),
        "inflated_blocker_rectangle_xy": inflated.tolist(), "centerline_blocked": centerline_blocked,
        "left_physical_gap_width_m": left_width, "left_valid_center_y_interval_m": left_interval,
        "left_corridor_feasible": True, "scene_fully_blocked": False,
        "initial_camera_world_xy": camera.tolist(), "goal_los_samples_xy": probes,
        "unoccluded_goal_los_samples_xy": visible, "goal_not_fully_occluded_geometric_proxy": True,
        "visibility_note": "horizontal ray/rectangle proxy; saved front RGB requires separate visual review",
        "decision_gate_x": params["decision_gate_x"], "status": "PASS"}
    write_json(records.path/"static_detour_geometry.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    import yaml
    from research.records import RunRecords, provenance, write_json
    cfg = yaml.safe_load(Path(args.config).read_text())
    if cfg["scenario"] != "asymmetric_offset_detour" or "pedestrian" in cfg:
        parser.error("Expected the no-human static detour config")
    records = RunRecords(args.output_dir, {"mode": "preflight_no_inference", "config": cfg,
                                          "command_argv": sys.argv, **provenance(REPO)})
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 640, "height": 480})
    sim, code = None, 1
    try:
        from research.sim import Simulation
        sim = Simulation(cfg, records, "static")
        _, state, path = sim.capture("preflight_front.png")
        result = {"status": "PASS", "mode": "preflight_no_inference", "model_calls": 0,
                  "rgb_path": str(path), "observation": state, "visual_review_required": True}
        write_json(records.path/"summary.json", result)
        print("STATIC_DETOUR_PREFLIGHT_PASS", json.dumps(result), flush=True)
        code = 0
    except Exception as exc:
        traceback.print_exc()
        write_json(records.path/"failure.json", {"error": repr(exc)})
    finally:
        if sim:
            sim.close()
        app.close(exit_code=code)


if __name__ == "__main__":
    main()
