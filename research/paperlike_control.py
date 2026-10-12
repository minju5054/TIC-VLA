"""Additive Nova target/state-machine adapter; native XY is never modified.

Reference: unchanged DynaNav/behavior/nova_carter_test_ticvla.py.
Frame counters below are the official nominal 30 Hz behavior clock, NOT the
60 Hz PhysX tick or wall time. The declared upstream 3s inference timeout is
unused in executable upstream code; no new age-based rejection is invented.
"""
import numpy as np
from research.control import waypoint_command, validate_chunk
from research.nova_source import slew, source_audit


class PaperlikeController:
    def __init__(self):
        self.constants = source_audit()["constants"]
        self.target = np.zeros(2, dtype=np.float32)
        self.filtered = np.zeros(2)
        self.yaw_filter = None
        self.first_complete = False
        self.backup_start = None
        self.backup_complete = None
        self.waiting = False
        self.last_position = None
        self.last_movement_frame = None
        self.generation_starts = []
        self.active_request_id = None

    @property
    def mode(self):
        if self.backup_start is not None:
            return "BACKUP"
        if self.waiting:
            return "WAIT_FRESH"
        return "NORMAL" if self.first_complete else "WAIT_INITIAL"

    def before_update(self, frame, position):
        """Upstream pre-inference backup completion / stuck detection order."""
        p = np.asarray(position, float)
        if self.backup_start is not None and frame-self.backup_start >= 300:
            self.backup_complete = frame
            self.backup_start = None
            self.waiting = True
            self.target[:] = 0
            self.last_position = self.last_movement_frame = None
        if self.first_complete and self.backup_start is None and not self.waiting:
            if self.last_position is None or self.last_movement_frame is None:
                self.last_position, self.last_movement_frame = p.copy(), frame
            elif np.linalg.norm(p[:2]-self.last_position[:2]) >= .1:
                self.last_position, self.last_movement_frame = p.copy(), frame
            if frame-self.last_movement_frame >= 150:
                self.backup_start = frame
                self.target[:] = [-.5, 0]
                self.filtered[:] = [-.5, 0]

    def accept(self, chunk, request_id, frame, position, new_generation, kv_available):
        if self.backup_start is not None:
            raise ValueError("Official behavior skips action inference during backup")
        command, self.yaw_filter, index = waypoint_command(validate_chunk(chunk), self.yaw_filter)
        # _inference_task tests the last tracked generation BEFORE on_update
        # appends a new generation. Reproduce that heuristic, not an invented
        # exact KV-to-action association.
        prior_generation = self.generation_starts[-1] if self.generation_starts else None
        stale = (self.waiting and self.backup_complete is not None and
                 prior_generation is not None and prior_generation < self.backup_complete)
        self.target = np.array([0., 0.] if stale else command, dtype=np.float32)
        if self.waiting and prior_generation is not None and prior_generation >= self.backup_complete:
            self.waiting = False
            self.backup_complete = None
            self.last_position, self.last_movement_frame = np.array(position), frame
        if new_generation is not None:
            self.generation_starts = (self.generation_starts+[new_generation])[-2:]
        if not self.first_complete and kv_available:
            self.first_complete = True
            self.last_position, self.last_movement_frame = np.array(position), frame
        if not stale:
            self.active_request_id = request_id
        return {"controller_command": list(command), "lookahead_index": index,
                "yaw_filter_state": self.yaw_filter, "command_accepted": not stale,
                "stale_rejected": bool(stale), "prior_tracked_generation_step": prior_generation,
                "generation_tracking_note": "Exact upstream last-tracked-start heuristic; not proven KV identity"}

    def output(self, dt=1/30):
        if self.backup_start is not None:
            self.target[:] = [-.5, 0]
            self.filtered[:] = [-.5, 0]
        else:
            target = np.zeros(2) if self.waiting or not self.first_complete else self.target
            c = self.constants
            for i, axis in enumerate(["lin", "ang"]):
                self.filtered[i] = slew(float(self.filtered[i]), float(target[i]),
                    c[f"_max_accel_{axis}"], c[f"_max_decel_{axis}"], dt, c[f"_{axis}_deadband"])
        # Upstream WAIT_FRESH overrides even the slewed values immediately
        # before differential conversion. Conversion receives float32.
        final = np.zeros(2) if self.waiting else self.filtered
        return np.asarray(final, dtype=np.float32).copy()


class SimulationScheduler:
    """At most one capture per due frame; no catch-up loop or pending queue."""
    def __init__(self, period_frames=3):
        self.period = period_frames
        self.next_frame = 0

    def due(self, frame):
        if frame < self.next_frame:
            return False
        self.next_frame = frame+self.period
        return True


def termination(xy, goal, sim_time, goal_radius=1.5, timeout=85.):
    distance = float(np.linalg.norm(np.asarray(xy)[:2]-np.asarray(goal)[:2]))
    return ("GOAL_REGION" if distance <= goal_radius else
            "TIMEOUT" if sim_time+1e-5 >= timeout else None), distance


def project_observation(chunk, event):
    """Derived world metres: R_world_body(obs) times native forward/left."""
    x, y, yaw = event["agent_pose_at_observation"]
    if event["native_frame"] != "observation_body" or event["native_axes"] != ["forward", "left"]:
        raise ValueError("Unverified native frame")
    c, s = np.cos(yaw), np.sin(yaw)
    return validate_chunk(chunk) @ np.array([[c, s], [-s, c]]) + [x, y]


def human_path_metrics(xy, human, radius, heading):
    """Complete segments, not just endpoints; fixed baseline heading at C19."""
    from research.nova_dynamic_geometry import polyline_clearance
    p = np.asarray(xy, float)
    forward = np.array([np.cos(heading), np.sin(heading)])
    left = np.array([-forward[1], forward[0]])
    relative = p-np.asarray(human)[:2]
    longitudinal, lateral = relative@forward, relative@left
    crossing = None
    for i in range(len(p)-1):
        if longitudinal[i] < 0 <= longitudinal[i+1]:
            a = -longitudinal[i]/(longitudinal[i+1]-longitudinal[i])
            crossing = float((1-a)*lateral[i]+a*lateral[i+1])
            break
    downstream = np.flatnonzero(longitudinal >= radius)
    first = int(downstream[0]) if len(downstream) else None
    return {"minimum_clearance_m": float(polyline_clearance(p, human, radius)),
            "centre_plane_lateral_m": crossing,
            "side": "DOES_NOT_REACH" if crossing is None else
                    "LEFT" if crossing > radius else "RIGHT" if crossing < -radius else "CENTER",
            "downstream_index": first,
            "path_length_after_clearing_m": float(np.linalg.norm(np.diff(p[first:], axis=0), axis=1).sum()) if first is not None else None,
            "net_progress_after_clearing_m": float(longitudinal[-1]-longitudinal[first]) if first is not None else None}
