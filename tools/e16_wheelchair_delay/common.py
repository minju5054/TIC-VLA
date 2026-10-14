"""Frozen C10 spatial trigger and saved-only arithmetic, no simulator imports.

Hospital world origin, metres, +Z up; world yaw CCW from +X in radians.
Native forward/left points use their observation body->world SE(2) transform.
Derived nominal timing is 10 Hz; no native yaw/time or extrapolation is invented.
"""
import math
import numpy as np
from research.audit_e16_wheelchair_handoffs import (
    read, write, digest, snapshot, verify_snapshot, rotation, wrap, to_world,
    remaining, interpolate, tangent, proxy_clearance, build_geometry, is_clear,
)

PLAN = [('NO_HOOK', 0.), ('ZERO_HOOK', 0.), ('ZERO_HOOK', 0.),
        ('NO_HOOK', 0.), ('DELAY_025', .25), ('DELAY_025', .25)]


def ticks_for(seconds, dt=1/60):
    if not np.isfinite(seconds) or seconds < 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('Invalid physics duration')
    n = round(seconds/dt)
    if abs(n*dt-seconds) > 1e-9:
        raise ValueError('Delay must be on physics grid')
    return n


def unicycle(pose, command, seconds):
    """Exact constant-(v,w) planar kinematics; not a PhysX or wheel replay."""
    x, y, theta = map(float, pose); v, w = map(float, command)
    turn = w*seconds
    distance = v*seconds*np.sinc(turn/(2*np.pi))
    return np.array([x+distance*np.cos(theta+turn/2), y+distance*np.sin(theta+turn/2),
                     float(wrap(theta+turn))])


def contact_is_nonfloor(e, robot_prim):
    # Preserve the existing Hospital horizontal-contact policy; include proximity.
    return bool(e.get('phase') == 'run' and e.get('contacts')
                and not all(e[k].startswith(robot_prim+'/') for k in ('collider0', 'collider1'))
                and any(abs(c['normal'][2]) < .5 for c in e['contacts']))


def contact_kind(e):
    if not e:
        return None
    impulse = max((float(np.linalg.norm(c['impulse'])) for c in e['contacts']), default=0.)
    separation = min((c['separation'] for c in e['contacts']), default=None)
    return dict(max_impulse_Ns=impulse, min_separation_m=separation,
                positive_impulse=impulse > 0, penetration=separation is not None and separation < 0,
                zero_impulse_nonpenetrating_callback=impulse == 0 and separation is not None and separation >= 0)


def fully_passed(pose, rules):
    along = (np.asarray(pose[:2])-rules['c10_center_xy_m']) @ np.asarray(rules['c10_forward_world'])
    return bool(along > rules['c10_complete_passage_offset_m'])


def trigger_check(observation, rules, old_id, passed_before=False, contact_before=False):
    """Only observation state/past measured history; never consumes a prediction."""
    delta = np.asarray(observation['pose'][:2])-rules['center_world_xy_m']
    local = delta @ rotation(rules['progress_yaw_rad'])
    yaw_error = abs(float(wrap(observation['pose'][2]-rules['reference_yaw_rad'])))
    velocity = np.asarray(observation['linear_velocity_world'][:2])
    along_v = float(velocity @ rotation(rules['progress_yaw_rad'])[:, 0])
    forward_v = float(observation['linear_velocity_body_flu'][0])
    checks = dict(nonbootstrap=old_id is not None, in_longitudinal=abs(local[0]) <= rules['half_along_m'],
                  in_lateral=abs(local[1]) <= rules['half_lateral_m'], yaw=yaw_error <= rules['yaw_half_width_rad'],
                  not_passed=not passed_before and not fully_passed(observation['pose'], rules),
                  forward=forward_v > rules['min_forward_speed_m_s'] and along_v > rules['min_forward_speed_m_s'],
                  no_prior_contact=not contact_before)
    return dict(eligible=bool(all(checks.values())), checks={k: bool(v) for k,v in checks.items()},
                observation_progress_xy_m=local.tolist(), yaw_error_rad=yaw_error,
                forward_speed_m_s=forward_v, reference_progress_speed_m_s=along_v)


def movement(xy, yaw, a, b):
    xy, yaw = np.asarray(xy), np.asarray(yaw)
    path = xy[a:b+1]; delta = path[-1]-path[0]
    return dict(start_tick=int(a), end_tick=int(b), ticks=int(b-a),
                path_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                net_displacement_m=float(np.linalg.norm(delta)), delta_xy_m=delta.tolist(),
                yaw_change_rad=float(wrap(yaw[b]-yaw[a])))


def motion_direction(xy, tick, count=6):
    a = max(0, tick-count); d = np.asarray(xy[tick])-xy[a]
    length = float(np.linalg.norm(d))
    return dict(valid=length >= .02, yaw_rad=float(np.arctan2(d[1], d[0])) if length >= .02 else None,
                displacement_m=length, start_tick=a, end_tick=tick,
                rule='Last six completed physics intervals, >=2 cm net displacement; otherwise undefined')


def boundary_metrics(world, nominal, pose, sim_time, exec_direction, policy, fixed_time=None, observation_yaw=None):
    """Time-aligned error and optional SAME fixed-reference-time spatial error."""
    ts, future, expected = remaining(nominal, world, sim_time)
    tan = tangent(future, pose[2] if observation_yaw is None else observation_yaw, policy)
    result = dict(sim_time=float(sim_time), pose=list(map(float,pose)),
                  nominal_valid=expected is not None, expected_xy_m=expected.tolist() if expected is not None else None,
                  remaining_horizon_s=max(0., float(nominal[-1]-sim_time)), tangent=tan,
                  position_error_m=None, longitudinal_error_m=None, lateral_error_m=None,
                  direction_error_rad=None, body_yaw_error_rad=None,
                  fixed_reference_time_s=fixed_time, fixed_reference_position_error_m=None)
    if expected is not None:
        delta=np.asarray(pose[:2])-expected; result['position_error_m']=float(np.linalg.norm(delta))
        if tan['valid']:
            components=delta @ rotation(tan['yaw_rad'])
            result.update(longitudinal_error_m=float(components[0]), lateral_error_m=float(components[1]),
                          body_yaw_error_rad=abs(float(wrap(pose[2]-tan['yaw_rad']))))
            if exec_direction is not None:
                result['direction_error_rad']=abs(float(wrap(exec_direction-tan['yaw_rad'])))
    if fixed_time is not None and nominal[0]-1e-9 <= fixed_time <= nominal[-1]+1e-9:
        point=interpolate(nominal,world,fixed_time)
        result['fixed_reference_position_error_m']=float(np.linalg.norm(np.asarray(pose[:2])-point))
        result['fixed_reference_expected_xy_m']=point.tolist()
    return result
