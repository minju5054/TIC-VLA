"""Pure geometric proxies. No spatial correspondence, controller correction or Isaac import."""
import numpy as np
from research.analyze_chunk_geometry import project_world, rotation, wrap


def rectangle(obstacle, inflation=0.0):
    center = np.asarray(obstacle["position"][:2], float)
    half = np.asarray(obstacle["size"][:2], float)/2 + inflation
    return np.r_[center-half, center+half]


def segment_intersects(a, b, rect, tolerance=1e-10):
    a, b, rect = np.asarray(a), np.asarray(b), np.asarray(rect)
    lower, upper = 0.0, 1.0
    for axis in (0, 1):
        delta = b[axis]-a[axis]
        if abs(delta) <= tolerance:
            if a[axis] < rect[axis]-tolerance or a[axis] > rect[axis+2]+tolerance:
                return False
        else:
            u, v = sorted([(rect[axis]-a[axis])/delta, (rect[axis+2]-a[axis])/delta])
            lower, upper = max(lower, u), min(upper, v)
            if lower > upper+tolerance:
                return False
    return True


def segment_clearance(a, b, rect):
    """Minimum signed Euclidean distance to an axis-aligned rectangle along a segment.

    Outside: edge/corner closest points. Inside: minimum of the convex maximum
    of four affine halfspace distances, attained at an endpoint or intersection.
    """
    a, b, rect = np.asarray(a, float), np.asarray(b, float), np.asarray(rect, float)
    d = b-a
    candidates = [0.0, 1.0]
    for axis in (0, 1):
        if abs(d[axis]) > 1e-15:
            candidates += [(rect[axis]-a[axis])/d[axis], (rect[axis+2]-a[axis])/d[axis]]
    norm2 = float(d@d)
    if norm2 > 1e-30:
        for x in (rect[0], rect[2]):
            for y in (rect[1], rect[3]):
                candidates.append(float((np.array([x, y])-a)@d/norm2))
    intercepts = [rect[0]-a[0], a[0]-rect[2], rect[1]-a[1], a[1]-rect[3]]
    slopes = [-d[0], d[0], -d[1], d[1]]
    for i in range(4):
        for j in range(i):
            if abs(slopes[i]-slopes[j]) > 1e-15:
                candidates.append((intercepts[j]-intercepts[i])/(slopes[i]-slopes[j]))
    points = a + np.asarray([u for u in candidates if 0 <= u <= 1])[:, None]*d
    outside = np.maximum(np.maximum(rect[:2]-points, points-rect[2:]), 0)
    signed = np.linalg.norm(outside, axis=1)
    inside = (outside == 0).all(axis=1)
    signed[inside] = -np.minimum(points[inside]-rect[:2], rect[2:]-points[inside]).min(axis=1)
    return float(signed.min())


def feasibility(points, obstacles, radius):
    rects = [rectangle(o, radius) for o in obstacles]
    if not rects:
        return {"predicted_static_obstacle_intersection": False, "minimum_static_obstacle_clearance_m": None}
    clearance = min(segment_clearance(a, b, r) for a, b in zip(points[:-1], points[1:]) for r in rects)
    return {"predicted_static_obstacle_intersection": clearance <= 0,
            "minimum_static_obstacle_clearance_m": clearance}


def route_side(points, gate_x, blocker, radius, corridor_half_width, wall_thickness=.1, tolerance=1e-8):
    points = np.asarray(points, float)
    crossings = []
    for a, b in zip(points[:-1], points[1:]):
        if abs(b[0]-a[0]) <= tolerance:
            if abs(a[0]-gate_x) <= tolerance:
                crossings.extend([float(a[1]), float(b[1])])
        elif min(a[0], b[0])-tolerance <= gate_x <= max(a[0], b[0])+tolerance:
            u = np.clip((gate_x-a[0])/(b[0]-a[0]), 0, 1)
            crossings.append(float(a[1]+u*(b[1]-a[1])))
    if not crossings:
        return {"route_side": "UNKNOWN/DOES_NOT_REACH", "gate_y": None}
    if max(crossings)-min(crossings) > tolerance:
        return {"route_side": "UNKNOWN/AMBIGUOUS", "gate_y": None}
    y = float(np.mean(crossings))
    rect = rectangle(blocker, radius)
    wall_limit = corridor_half_width-wall_thickness/2-radius
    side = "CENTER/BLOCKED"
    if rect[3]+tolerance < y < wall_limit-tolerance:
        side = "LEFT"
    elif -wall_limit+tolerance < y < rect[1]-tolerance:
        side = "RIGHT"
    return {"route_side": side, "gate_y": y}


def interpolate(times, points, query):
    times, points, query = np.asarray(times), np.asarray(points), np.asarray(query)
    if np.any(np.diff(times) <= 0) or np.any(query < times[0]-1e-9) or np.any(query > times[-1]+1e-9):
        raise ValueError("Temporal interpolation must not extrapolate")
    return np.stack([np.interp(query, times, points[:, i]) for i in range(2)], axis=-1)


def temporal_overlap(old, fresh, old_time, fresh_time, dt=.1, tangent_epsilon=1e-6):
    old_times = old_time + dt*np.arange(1, len(old)+1)
    fresh_times = fresh_time + dt*np.arange(1, len(fresh)+1)
    lo, hi = max(old_times[0], fresh_times[0]), min(old_times[-1], fresh_times[-1])
    if hi <= lo:
        return None
    # FRESH nominal knots plus exact common endpoints, not spatial matching.
    queries = np.unique(np.r_[lo, fresh_times[(fresh_times > lo+1e-8) & (fresh_times < hi-1e-8)], hi])
    op, fp = interpolate(old_times, old, queries), interpolate(fresh_times, fresh, queries)
    delta = fp-op
    os, fs = np.diff(op, axis=0), np.diff(fp, axis=0)
    valid = (np.linalg.norm(os, axis=1) > tangent_epsilon) & (np.linalg.norm(fs, axis=1) > tangent_epsilon)
    angle = np.abs(np.degrees(wrap(np.arctan2(fs[:, 1], fs[:, 0])-np.arctan2(os[:, 1], os[:, 0]))))[valid]
    return {"absolute_times": queries, "old": op, "fresh": fp, "delta": delta,
            "rmse": float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))),
            "tangent_mean_deg": float(angle.mean()) if len(angle) else None,
            "tangent_max_deg": float(angle.max()) if len(angle) else None}


def boundary_seam(world, observation, application, executed_times, executed_xy,
                  dt=.1, window=.1, tangent_epsilon=1e-6):
    elapsed = application["sim_time"]-observation["sim_time"]
    # This anchor belongs ONLY to this derived temporal diagnostic, never raw data.
    times = dt*np.arange(len(world)+1)
    points = np.vstack([observation["position"][:2], world])
    result = {"raw_boundary_position_gap_m": None, "raw_executed_to_fresh_tangent_gap_deg": None,
              "raw_robot_heading_to_fresh_tangent_gap_deg": None, "fresh_expected_xy": None,
              "fresh_tangent_rad": None, "executed_tangent_rad": None,
              "fresh_tangent_segment_length_m": None, "executed_window_displacement_m": None,
              "derived_temporal_anchor": "observation XY at tau=0; not a native TIC-VLA waypoint",
              "tangent_note": "derived geometric tangent; not native TIC-VLA yaw"}
    if elapsed < 0 or elapsed > times[-1]+1e-9:
        result["undefined_reason"] = "switch elapsed time outside nominal horizon; no extrapolation"
        return result
    expected = interpolate(times, points, elapsed)
    result["fresh_expected_xy"] = expected.tolist()
    result["raw_boundary_position_gap_m"] = float(np.linalg.norm(np.asarray(application["position"][:2])-expected))
    index = min(np.searchsorted(times, elapsed, side="right")-1, len(points)-2)
    tangent = points[index+1]-points[index]  # right-hand segment at exact knot
    result["fresh_tangent_segment_length_m"] = float(np.linalg.norm(tangent))
    phi = None if np.linalg.norm(tangent) <= tangent_epsilon else float(np.arctan2(tangent[1], tangent[0]))
    result["fresh_tangent_rad"] = phi
    end, begin = application["sim_time"], max(observation["sim_time"], application["sim_time"]-window)
    result["executed_window_start_sim_time"], result["executed_window_end_sim_time"] = begin, end
    if begin < end and begin >= executed_times[0]-1e-9 and end <= executed_times[-1]+1e-9:
        a, b = interpolate(executed_times, executed_xy, np.array([begin, end]))
        result["executed_window_displacement_m"] = float(np.linalg.norm(b-a))
        if np.linalg.norm(b-a) > tangent_epsilon:
            result["executed_tangent_rad"] = float(np.arctan2(*(b-a)[::-1]))
    if phi is not None:
        result["raw_robot_heading_to_fresh_tangent_gap_deg"] = float(abs(np.degrees(wrap(phi-application["pose"][2]))))
        if result["executed_tangent_rad"] is not None:
            result["raw_executed_to_fresh_tangent_gap_deg"] = float(abs(np.degrees(wrap(phi-result["executed_tangent_rad"]))))
    return result


def numerical_sign(value, epsilon):
    return 0 if abs(value) <= epsilon else (1 if value > 0 else -1)


def scene_obstacles(cfg):
    """Collision rectangles including corridor walls and the terminal goal wall."""
    scene = cfg["scene"]
    goal, h = scene["goal"], scene["wall_height"]
    return list(scene.get("obstacles", [])) + [
        {"name": "LeftWall", "position": [4, scene["corridor_half_width"], h/2], "size": [10, .1, h]},
        {"name": "RightWall", "position": [4, -scene["corridor_half_width"], h/2], "size": [10, .1, h]},
        {"name": "GoalWall", "position": [goal[0], goal[1], goal[2]+h/2], "size": [.1, 2, h]}]


def chunk_context(chunk, event, cfg, radius):
    world = project_world(chunk, event["agent_pose_at_observation"])
    context = {"request_id": event["request_id"], "route_side": "NOT_APPLICABLE", "gate_y": None,
               **feasibility(world, scene_obstacles(cfg), radius)}
    if "route_switch" in cfg:
        blocker = next(o for o in cfg["scene"]["obstacles"] if o["name"] == "central_blocker")
        context.update(route_side(world, cfg["route_switch"]["decision_gate_x"], blocker, radius, cfg["scene"]["corridor_half_width"]))
    return world, context


def static_route_gate(contexts, cfg):
    ids = cfg["route_switch"]["static_gate_request_ids"]
    chosen = [c for c in contexts if c["request_id"] in ids]
    valid = (len(chosen) == len(ids) and chosen[0]["route_side"] in ("LEFT", "RIGHT")
             and all(c["route_side"] == chosen[0]["route_side"] and not c["predicted_static_obstacle_intersection"] for c in chosen))
    return {"decision": "VALID BASE ROUTE CHOICE" if valid else "NO VALID BASE ROUTE CHOICE",
            "valid": valid, "request_ids": ids, "contexts": chosen,
            "rule": "Both prespecified pre-reveal requests choose the same gap and avoid all inflated static rectangles"}
