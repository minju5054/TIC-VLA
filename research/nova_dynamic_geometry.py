"""Pure baseline-derived intervention and human/trajectory diagnostics."""
import json
from pathlib import Path
import numpy as np
from research.analyze_chunk_geometry import project_world
from research.route_geometry import interpolate, rotation, wrap
from research.hospital_episode import digest

PRIMARY = (15, 16)


def human_position(config, sim_time, trigger_time):
    start, end = np.asarray(config["start_position"], float), np.asarray(config["end_position"], float)
    distance = np.linalg.norm(end-start)
    if distance <= 0 or config["speed_m_s"] <= 0:
        raise ValueError("Human requires a nonzero fixed path and speed")
    elapsed = 0. if trigger_time is None else max(0., sim_time-trigger_time)
    return start+(end-start)*min(1., elapsed*config["speed_m_s"]/distance)


class AcceptanceTrigger:
    def __init__(self, request_id=15):
        self.request_id, self.time = request_id, None

    def accept(self, request_id, sim_time):
        if request_id != self.request_id:
            return False
        if self.time is not None:
            raise ValueError("Human acceptance trigger must fire exactly once")
        self.time = float(sim_time)
        return True


def polyline_clearance(points, center, radius):
    points, center = np.asarray(points, float), np.asarray(center, float)[:2]
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Need a finite polyline")
    a, delta = points[:-1], np.diff(points, axis=0)
    norm2 = np.sum(delta**2, axis=1)
    u = np.divide(np.sum((center-a)*delta, axis=1), norm2, out=np.zeros(len(a)), where=norm2>1e-20)
    closest = a+np.clip(u,0,1)[:,None]*delta
    return float(np.linalg.norm(closest-center,axis=1).min()-radius)


def remaining_curve(world, observation_time, now, dt=.1):
    times = observation_time+dt*np.arange(1,len(world)+1)
    lo = max(float(now),float(times[0]))
    if lo >= times[-1]:
        return None, None
    q = np.unique(np.r_[lo,times[times>lo]])
    return q, interpolate(times,world,q)


def temporal_human_clearance(times, robot_points, human_times, human_xy, radius):
    """Piecewise-linear relative-motion minimum, no temporal extrapolation."""
    lo,hi=max(times[0],human_times[0]),min(times[-1],human_times[-1])
    if hi<=lo: return None
    q=np.unique(np.r_[lo,times[(times>lo)&(times<hi)],human_times[(human_times>lo)&(human_times<hi)],hi])
    robot=interpolate(times,robot_points,q);human=interpolate(human_times,human_xy,q)
    relative=robot-human
    return {"minimum_clearance_m":polyline_clearance(relative,[0,0],radius),
            "absolute_times":q.tolist(),"sample_clearance_m":(np.linalg.norm(relative,axis=1)-radius).tolist(),
            "domain":[float(lo),float(hi)],"note":"Measured human, nominal model target timing; linear temporal interpolation only"}


def baseline_design(run_dir, robot_radius, human_radius=.25, speed=1.0):
    """Fixed 1.5s OLD target; normal to baseline measured C15→C17 turn chord."""
    path=Path(run_dir).resolve()
    events=[json.loads((path/f"raw/requests/request_{i:06d}.json").read_text()) for i in [15,16,17]]
    chunk=np.load(path/"raw/requests/request_000015.npy",allow_pickle=False)
    world=project_world(chunk,events[0]["agent_pose_at_observation"])
    tangent=np.array(events[2]["agent_pose_at_observation"][:2])-events[0]["agent_pose_at_observation"][:2]
    tangent=tangent/np.linalg.norm(tangent);normal=np.array([-tangent[1],tangent[0]])
    center=world[14];radius=robot_radius+human_radius
    # Fixed geometric margin, chosen before dynamic inference. Both staging
    # sides are checked against the unchanged Hospital, then first valid chosen.
    offset=radius+.20
    candidates=[]
    gap=events[1]["observation"]["sim_time"]-events[0]["application"]["sim_time"]
    for sign in [1,-1]:
        direction=-sign*normal
        start=center+sign*offset*normal;end=center-sign*offset*normal
        human={"start_position":[*map(float,start),0.],"end_position":[*map(float,end),0.],"speed_m_s":speed,
               "yaw":float(np.arctan2(direction[1],direction[0])),"trigger_request_id":15}
        _,remaining=remaining_curve(world,events[0]["observation"]["sim_time"],events[1]["observation"]["sim_time"])
        candidates.append({"side_sign":sign,"human":human,
            "staged_old_clearance_m":polyline_clearance(world,start,radius),
            "expected_fresh_observation_old_clearance_m":polyline_clearance(remaining,human_position(human,gap,0),radius)})
    return {"baseline_run":str(path),"primary_pair":[15,16],"center_world_xy":center.tolist(),
            "center_rule":"C15 native target index14 (1-based point15), nominal +1.5s",
            "baseline_turn_tangent":tangent.tolist(),"crossing_normal":normal.tolist(),
            "staging_offset_m":offset,"robot_radius_m":robot_radius,"human_radius_m":human_radius,
            "baseline_application15_to_observation16_s":gap,"candidates":candidates,
            "source_sha256":{str(p):digest(p) for p in [path/"metadata.json",path/"robot_state.csv",
                *[path/f"raw/requests/request_{i:06d}.{ext}" for i in [15,16,17] for ext in ["json","npy"]]]}}


def validate_frozen_config(config, baseline):
    for key in ["robot","scene","official_episode","instruction","camera","controller","inference","simulation","seed","hospital_validation"]:
        if config[key]!=baseline[key]: raise ValueError("Frozen base platform changed: "+key)
    h=config["dynamic_human"]
    if h["trigger_request_id"]!=15 or config["dynamic_analysis"]["primary_pair"]!=[15,16]:
        raise ValueError("Primary intervention IDs are fixed")
