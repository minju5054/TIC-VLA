"""Native polyline controller extracted from Nova Carter lines 1367–1414.

No cumulative sum of native offsets, no waypoint yaw, no reconciliation.
"""
import math
import numpy as np


def validate_chunk(chunk):
    a = np.asarray(chunk)
    if a.shape != (30, 2) or not np.issubdtype(a.dtype, np.floating):
        raise ValueError(f"Expected floating native (30,2), got {a.shape} {a.dtype}")
    if not np.isfinite(a).all():
        raise ValueError("Non-finite native prediction")
    return a


def waypoint_command(chunk, yaw_filter=None, *, lookahead=1.0, angular_gain=0.8,
                     filter_gain=0.35, epsilon=1e-3, max_v=1.5, max_w=1.2):
    wps = validate_chunk(chunk)
    inc = np.diff(wps, axis=0)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(inc[:, 0], inc[:, 1]))])
    j = int(np.clip(np.searchsorted(s, lookahead, side="left"), 2, len(wps) - 3))
    x, y = map(float, wps[j])
    length = float(np.hypot(x, y))
    if length < epsilon:
        return (0.0, 0.0), yaw_filter, j
    error = math.atan2(y, x)
    if yaw_filter is None:
        yaw_filter = error
    delta = math.atan2(math.sin(error - yaw_filter), math.cos(error - yaw_filter))
    yaw_filter += filter_gain * delta
    curvature = 2 * y / length**2
    v = float(np.clip(min(max_v, max_w / (abs(curvature) + epsilon)), 0, max_v))
    w = float(np.clip(0.5 * v * curvature + angular_gain * yaw_filter, -max_w, max_w))
    return (v, w), yaw_filter, j


def wheel_speeds(v, w, radius, track, signs=(1, 1, 1, 1)):
    """Joint order: front-left, rear-left, front-right, rear-right; rad/s."""
    if radius <= 0 or track <= 0 or len(signs) != 4 or any(s not in (-1, 1) for s in signs):
        raise ValueError("Invalid verified wheel geometry/signs")
    if not np.isfinite([v, w, radius, track]).all():
        raise ValueError("Non-finite wheel command")
    left, right = (v - track*w/2)/radius, (v + track*w/2)/radius
    return np.array([left, left, right, right]) * signs


def rotation_wxyz(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def planar_pose(position, quaternion):
    r = rotation_wxyz(quaternion)
    return [float(position[0]), float(position[1]), math.atan2(r[1, 0], r[0, 0])]
