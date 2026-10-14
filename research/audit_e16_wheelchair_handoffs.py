"""Saved-only E16 handoff audit. No simulator, inference or optimizer imports.

Native arrays remain observation-local forward/left offsets. Derived XY uses
the observed body-to-world SE(2) pose; derived tangents are never native yaw.
The immutable protocol is recorded before pair metrics in a fresh output dir.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np


FROZEN_POLICY = {'schema': 1,
 'nominal_dt_s': 0.1,
 'nominal_timing': 'analysis convention from package 10 Hz action contract and data STEP_SEC, not '
                   'native timestamps; no t=0 synthetic knot or extrapolation',
 'min_segment_m': 0.02,
 'max_adjacent_heading_change_deg': 45.0,
 'significant_heading_deg': 30.0,
 'boundary_position_threshold': 'robot disk radius / 4',
 'meaningful_lateral_threshold': 'robot disk radius / 2',
 'proxy_sample_step_m': 0.01,
 'safe_proxy_margin_m': 0.02,
 'proxy_robot_radius_source': 'config.spatial_turn_gate.robot_conservative_radius_m',
 'structure_filter': 'enabled wall/doorframe/trim AABB intersecting z in (0.15,0.65), verified source '
                     'inventory; wheelchairs use saved yaw/half-footprint OBB',
 'obstacle_window': 'first measured robot distance <= max_v*3 + robot_radius + wheelchair circumscribed '
                    'half-footprint radius, through first complete passage; intersect [obs,switch], '
                    'pre-goal only',
 'meaningful_intent': 'both predicted centre-plane crossings exist and sides flip, OR OLD proxy overlap '
                      'changes to FRESH proxy-clear predicted crossing with lateral RMS > '
                      'robot_radius/2; side labels only when horizon crosses',
 'candidate_A': 'pre-goal near obstacle; valid nominal boundary and stable tangent; actual OLD pending '
                'motion; meaningful revision; FRESH remaining proxy-clear; B proxy-clear; RAW connector '
                'introduces proxy overlap. Large RMSE/heading alone never enough.',
 'category_precedence': 'D unsafe/unqualified FRESH proxy; E missing time/tangent; A all required '
                        'evidence; B demonstrated side change with clear connector; C otherwise',
 'proxy_caution': 'D is proxy unsafe OR unqualified, never measured collision; nominal speed/turn cap '
                  'excess is separately reported and does not prove physical infeasibility under '
                  'retiming',
 'representative_selection': 'C10/C24 first pair in geometry-fixed window with valid FRESH predicted '
                             'crossing and tangent; else switch nearest measured passage. Controls '
                             'first C pair outside both windows with stable tangent, boundary '
                             'gap<=radius/4, clear FRESH and connector; if none explicitly report no '
                             'qualified control. All pair rows retained.',
 'final_decision': 'A present -> POTENTIAL_RECONCILIATION_HARD_CASE_FOUND; no A and required global '
                   'data valid -> NO_RECONCILIATION_HARD_CASE_EVIDENCE; absent core evidence -> '
                   'INSUFFICIENT_EVIDENCE or BLOCKED for missing/corrupt data',
 'frozen_before_pair_metrics': True}


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def snapshot(path):
    return {str(p.relative_to(path)): digest(p) for p in sorted(Path(path).rglob('*')) if p.is_file()}


def verify_snapshot(path, expected):
    actual = snapshot(path)
    if actual != expected:
        raise ValueError('Source file inventory/content changed')
    return len(actual)


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def to_world(local, observation_pose):
    return np.asarray(local) @ rotation(observation_pose[2]).T + np.asarray(observation_pose[:2])


def to_local(world, observation_pose):
    return (np.asarray(world)-observation_pose[:2]) @ rotation(observation_pose[2])


def interpolate(times, points, query):
    times, points, q = np.asarray(times), np.asarray(points), np.asarray(query)
    if len(times) < 2 or np.any(np.diff(times) <= 0) or not np.isfinite(q).all():
        raise ValueError('Invalid interpolation axis')
    if np.any(q < times[0]-1e-9) or np.any(q > times[-1]+1e-9):
        raise ValueError('Extrapolation forbidden')
    q = np.clip(q, times[0], times[-1])  # Floating-point boundary tolerance only.
    return np.stack([np.interp(q, times, points[:, j]) for j in range(points.shape[1])], axis=-1)


def remaining(times, points, switch):
    """No synthetic observation-origin knot; no extension past native support."""
    keep = np.asarray(times) > switch+1e-9
    ts, xy = np.asarray(times)[keep], np.asarray(points)[keep]
    at = None
    if times[0]-1e-9 <= switch <= times[-1]+1e-9:
        at = interpolate(times, points, switch)
        ts = np.r_[switch, ts]
        xy = np.vstack([at, xy])
    return ts, xy, at


def tangent(points, reference_yaw, policy):
    p = np.asarray(points)
    if len(p) < 2:
        return dict(valid=False, reason='NO_SEGMENT', length_m=None, yaw_rad=None)
    dif = np.diff(p, axis=0); lengths = np.linalg.norm(dif, axis=1)
    yaw = float(np.arctan2(dif[0, 1], dif[0, 0])) if lengths[0] > 0 else None
    reason = 'VALID'
    if lengths[0] < policy['min_segment_m']:
        reason = 'SHORT_FIRST_SEGMENT'
    elif np.cos(yaw-reference_yaw) <= 0:
        reason = 'BACKWARD_INITIAL_SEGMENT'
    elif len(lengths) > 1:
        if lengths[1] < policy['min_segment_m']:
            reason = 'SHORT_NEIGHBOR_SEGMENT'
        elif abs(wrap(np.arctan2(dif[1, 1], dif[1, 0])-yaw)) > math.radians(policy['max_adjacent_heading_change_deg']):
            reason = 'UNSTABLE_OR_REVERSING_SEGMENTS'
    return dict(valid=reason == 'VALID', reason=reason, length_m=float(lengths[0]), yaw_rad=yaw)


def time_comparison(old_t, old_xy, fresh_t, fresh_xy, switch, observation_yaw):
    start, end = max(old_t[0], fresh_t[0], switch), min(old_t[-1], fresh_t[-1])
    if end <= start:
        return dict(valid=False, reason='NO_COMMON_NOMINAL_SUPPORT')
    knots = np.unique(np.r_[start, old_t[(old_t > start) & (old_t < end)],
                            fresh_t[(fresh_t > start) & (fresh_t < end)], end])
    a = interpolate(old_t, old_xy, knots); b = interpolate(fresh_t, fresh_xy, knots)
    local = (b-a) @ rotation(observation_yaw)
    # Exact time integral of squared piecewise-linear discrepancy, not a knot-count average.
    dt = np.diff(knots)
    squared = (local[:-1]**2 + local[:-1]*local[1:] + local[1:]**2)/3
    mean_square = (dt[:, None]*squared).sum(0)/(end-start)
    signed = (dt[:, None]*(local[:-1]+local[1:])/2).sum(0)/(end-start)
    return dict(valid=True, start_sim_s=float(start), end_sim_s=float(end), knots=len(knots),
                position_rmse_m=float(np.sqrt(mean_square.sum())), longitudinal_rms_m=float(np.sqrt(mean_square[0])),
                lateral_rms_m=float(np.sqrt(mean_square[1])), longitudinal_mean_m=float(signed[0]), lateral_mean_m=float(signed[1]),
                max_position_difference_m=float(np.linalg.norm(local, axis=1).max()))


def crossing(points, center, yaw, half_width, radius):
    p = np.asarray(points)
    if len(p) < 2:
        return dict(status='NO_PREDICTED_CROSSING', reason='SHORT_HORIZON')
    local = to_local(p, [*center[:2], yaw])
    ids = np.flatnonzero((local[:-1, 0] < 0) & (local[1:, 0] >= 0))
    if not len(ids):
        return dict(status='NO_PREDICTED_CROSSING', reason='ALREADY_BEYOND' if local[0, 0] >= 0 else 'HORIZON_DOES_NOT_CROSS')
    i = int(ids[0]); u = float(-local[i, 0]/(local[i+1, 0]-local[i, 0]))
    lateral = float(local[i, 1]+u*(local[i+1, 1]-local[i, 1]))
    # Passage side is geometry, independent of conservative footprint clearance.
    side = 'LEFT' if lateral > 0 else 'RIGHT' if lateral < 0 else 'CENTRE'
    return dict(status='CROSSES', side=side, signed_left_m=lateral, world_xy=(p[i]+u*(p[i+1]-p[i])).tolist(),
                proxy_corridor_overlap=abs(lateral) <= half_width+radius,
                segment_index=i, fraction=u, forward_crossing_count=len(ids))


def sample_polyline(points, step):
    p = np.asarray(points)
    if len(p) < 2:
        return p.copy(), 0.
    lengths = np.linalg.norm(np.diff(p, axis=0), axis=1)
    counts = np.maximum(1, np.ceil(lengths/step).astype(int))
    pieces = [a+(b-a)*np.linspace(0, 1, n+1)[:, None] for a, b, n in zip(p[:-1], p[1:], counts)]
    return np.vstack(pieces), float(np.max(lengths/counts)/2)


def rectangle_signed_distance(points, center, half, yaw=0.):
    q = np.abs(to_local(points, [*center, yaw]))-half
    return np.linalg.norm(np.maximum(q, 0), axis=1)+np.minimum(np.max(q, axis=1), 0)


def proxy_clearance(points, boxes, radius, step):
    p, error = sample_polyline(points, step)
    if not len(p):
        return dict(valid=False, reason='NO_REMAINING_PATH')
    best, name = float('inf'), None
    for box in boxes:
        distances = rectangle_signed_distance(p, box['center'], box['half'], box['yaw'])-radius
        value = float(distances.min())
        if value < best:
            best, name = value, box['name']
    if name is None:
        return dict(valid=False, reason='NO_GEOMETRY')
    return dict(valid=True, sampled_min_m=best, certified_lower_m=best-error,
                sampling_error_bound_m=error, limiting_prim=name, proxy_overlap_observed=best < 0,
                actual_physx_collision_claim=False)


def is_clear(proxy, policy):
    return proxy['valid'] and proxy['certified_lower_m'] > policy['safe_proxy_margin_m']


def committed_history(robot, old_application_tick, fresh_application_tick):
    if not 0 <= old_application_tick <= fresh_application_tick < len(robot):
        raise ValueError('Invalid committed interval')
    return np.array([[float(r[k]) for k in ('x', 'y')] for r in robot[old_application_tick:fresh_application_tick+1]])


def is_primary(switch_time, goal_time):
    return switch_time < goal_time


def classify(m, policy):
    reasons = []
    if not m['primary']:
        return 'POST_GOAL_EXCLUDED', ['SWITCH_AT_OR_AFTER_FIRST_GOAL_ENTRY']
    if not is_clear(m['fresh_proxy'], policy):
        return 'UNSAFE_OR_UNQUALIFIED_FRESH', ['FRESH_REMAINING_NOT_CERTIFIED_CLEAR_BY_CONSERVATIVE_PROXY', 'NOT_A_MEASURED_COLLISION']
    if m['boundary_position_m'] is None or not m['tangent']['valid'] or not m['aligned']['valid']:
        return 'INSUFFICIENT_EVIDENCE', ['NOMINAL_BOUNDARY_OR_TANGENT_OR_COMMON_TIME_INVALID']
    meaningful = any(x['meaningful_revision'] for x in m['obstacles'].values() if x['in_window'])
    side_change = any(x['side_change'] for x in m['obstacles'].values() if x['in_window'])
    boundary_only = (is_clear(m['boundary_proxy'], policy) and m['connector_proxy']['valid']
                     and m['connector_proxy']['proxy_overlap_observed'])
    if meaningful and m['pending_path_m'] >= policy['min_segment_m'] and boundary_only:
        return 'POTENTIAL_RECONCILIATION_HARD_CASE', ['NEW_GEOMETRIC_FUTURE', 'FRESH_AND_BOUNDARY_PROXY_CLEAR',
                   'VIRTUAL_CONNECTOR_ADDS_PROXY_OVERLAP', 'MEASURED_PENDING_OLD_TRANSPORT', 'GRAPH_BENEFIT_UNTESTED']
    if side_change and is_clear(m['connector_proxy'], policy):
        return 'INTENT_CHANGE_WITHOUT_HANDOFF_PROBLEM', ['PREDICTED_PASSAGE_SIDE_FLIP', 'RAW_CONNECTOR_PROXY_CLEAR']
    if not meaningful:
        reasons.append('NO_DEMONSTRATED_NEW_OBSTACLE_BYPASS')
    if not boundary_only:
        reasons.append('NO_CONNECTOR_ONLY_PROXY_RISK')
    reasons.append('NUMERIC_MISMATCH_ALONE_IS_NOT_A_HARD_CASE')
    return 'MISMATCH_WITHOUT_DEMONSTRATED_RISK', reasons


def curvature_diagnostics(times, points, min_segment, max_v, max_w):
    if len(points) < 2:
        return dict(valid_segments=0, invalid_segments=0)
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1); dt = np.diff(times)
    valid = (ds >= min_segment) & (dt > 0)
    speed = np.divide(ds, dt, out=np.zeros_like(ds), where=dt > 0)
    angles = np.arctan2(np.diff(points, axis=0)[:, 1], np.diff(points, axis=0)[:, 0])
    turns = wrap(np.diff(angles)); ok = valid[:-1] & valid[1:] & (np.abs(turns) <= np.pi/4)
    curv = np.abs(turns[ok])/((ds[:-1]+ds[1:])[ok]/2)
    omega = np.abs(turns[ok])/((dt[:-1]+dt[1:])[ok]/2)
    return dict(valid_segments=int(valid.sum()), invalid_segments=int((~valid).sum()),
                max_nominal_segment_speed_m_s=float(speed.max()), nominal_speed_cap_excess_count=int((speed > max_v).sum()),
                max_valid_curvature_per_m=float(curv.max()) if len(curv) else None,
                max_valid_nominal_turn_rate_rad_s=float(omega.max()) if len(omega) else None,
                nominal_turn_cap_excess_count=int((omega > max_w).sum()),
                min_curvature_limited_speed_m_s=float(min(max_v, max_w/curv.max())) if len(curv) and curv.max() > 0 else max_v,
                interpretation='Nominal-time diagnostic only; controller does not track this timed queue; retiming may satisfy caps.')


def verify_data(source, root):
    from research.nova_evidence import load_nova
    from research.control import waypoint_command, planar_pose, rotation_wxyz
    run = load_nova(source); robot = run['robot']; events = run['events']; cfg = run['cfg']
    if len(events) != 90:
        raise ValueError('Expected exactly 90 real predictions')
    ticks = np.array([int(r['tick']) for r in robot]); times = np.array([float(r['sim_time']) for r in robot])
    if not np.array_equal(ticks, np.arange(len(robot))) or not np.allclose(np.diff(times), 1/60, rtol=0, atol=1e-7):
        raise ValueError('Noncontiguous physics state')
    filt = None; error = 0.; canonical_yaw_error = 0.
    for e, chunk in zip(events, run['chunks']):
        if chunk.shape != (30, 2) or not np.isfinite(chunk).all() or e['native_yaw'] is not None:
            raise ValueError('Invalid native shape/values')
        command, filt, index = waypoint_command(chunk, filt, **cfg['controller'])
        if not np.allclose(command, e['controller_command'], rtol=0, atol=1e-12) or index != e['lookahead_index']:
            raise ValueError('Controller does not match native saved action')
        for key in ('observation', 'response_detected', 'application'):
            state = e[key]; r = robot[state['tick']]
            pose = np.array([float(r[k]) for k in ('x', 'y', 'yaw')])
            quat = [float(r[k]) for k in ('qw', 'qx', 'qy', 'qz')]
            err = float(np.max(np.abs(pose-state['pose']))); error = max(error, err)
            if err > 1e-8 or not np.allclose(quat, state['quaternion_wxyz'], rtol=0, atol=1e-8) or abs(float(r['sim_time'])-state['sim_time']) > 1e-8:
                raise ValueError('Event pose/time disagrees with physics row')
            # Simulator provided float32 q: the original helper evaluates norm(q)
            # before casting its numerator to float64. Preserve that arithmetic
            # for the recorder check; world projection still uses recorded yaw.
            derived_pose = planar_pose(state['position'], np.asarray(quat, dtype=np.float32))
            arithmetic_bound = 8*np.finfo(np.float32).eps
            if not np.allclose(derived_pose, state['pose'], rtol=0, atol=arithmetic_bound):
                raise ValueError('Quaternion convention disagrees with yaw')
            canonical_error = abs(float(wrap(planar_pose(state['position'], quat)[2]-state['pose'][2])))
            canonical_yaw_error = max(canonical_yaw_error, canonical_error)
            if canonical_error > 8*np.finfo(np.float32).eps:
                raise ValueError('Canonical quaternion yaw exceeds float32 arithmetic bound')
            R = rotation_wxyz(np.asarray(quat, dtype=np.float32))
            if not np.allclose(R.T @ state['linear_velocity_world'], state['linear_velocity_body_flu'], atol=arithmetic_bound, rtol=0):
                raise ValueError('World/body velocity frame mismatch')
        # New wheel target begins on the interval after the application row.
        start = e['application']['tick']+1
        stop = events[e['request_id']]['application']['tick'] if e['request_id'] < len(events) else len(robot)-1
        for r in robot[start:stop+1]:
            if r['active_control_source_request_id'] != str(e['request_id']) or not np.allclose([float(r['applied_v']), float(r['applied_w'])], command, rtol=0, atol=1e-12):
                raise ValueError('Applied OLD history/source ownership mismatch')
    xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in robot]); yaw = np.unwrap([float(r['yaw']) for r in robot])
    goal_d = np.linalg.norm(xy-np.array(cfg['scene']['goal'][:2]), axis=1)
    inside = np.flatnonzero(goal_d < 2.)
    if not len(inside):
        raise ValueError('Selected successful run never entered goal')
    goal_tick = int(inside[0]); goal_time = float(times[goal_tick])
    contact_events = [json.loads(s) for s in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    nonfloor = [e for e in contact_events if e['sim_time'] is not None and not all(e[k].startswith(cfg['robot']['prim']+'/') for k in ('collider0', 'collider1')) and any(abs(c['normal'][2]) < .5 for c in e['contacts'])]
    wheels = read(source/'wheelchairs_asset.json')
    with (source/'raw/wheelchairs_state.csv').open() as f:
        states = list(csv.DictReader(f))
    if len(states) != 2*len(robot):
        raise ValueError('Missing wheelchair tick records')
    for name, asset in wheels.items():
        own = [r for r in states if r['id'] == name]
        if [int(r['tick']) for r in own] != ticks.tolist():
            raise ValueError('Wheelchair tick alignment failed')
        if any(float(r['matrix_max_error']) != 0 or not np.allclose([float(r[k]) for k in ('x', 'y', 'z')], asset['center_world_xyz'], rtol=0, atol=1e-10) for r in own):
            raise ValueError('Wheelchair moved')
    vis = [read(source/'raw/object_visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    if any(v['observation_sim_time'] != e['observation']['sim_time'] for e, v in zip(events, vis)):
        raise ValueError('Visibility time mismatch')
    runtime = read(source/'inference-runtime.json')
    inventory_path = root/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'
    if digest(inventory_path) != cfg['spatial_turn_gate']['source_inventory_sha256']:
        raise ValueError('Collision inventory identity mismatch')
    inventory = read(inventory_path)
    if hashlib.sha256(json.dumps(inventory, sort_keys=True).encode()).hexdigest() != read(source/'hospital_lighting_applied.json')['collision_inventory_sha256']:
        raise ValueError('Historical geometry differs from run geometry')
    # Nominal timing is source evidence, not a field invented on the native array.
    timing_source = root/'ticvla/models/ticvla.py'; data_source = root/'data/s01_batch_json_generation.py'
    if 'future steps at 10 Hz' not in timing_source.read_text() or 'DST_HZ = 10.0' not in data_source.read_text():
        raise ValueError('Nominal timing source contract not confirmed')
    validation = dict(predictions=90, raw_shape=[30, 2], robot_rows=len(robot), goal_tick=goal_tick, goal_sim_s=goal_time,
                      closest_goal_m=float(goal_d.min()), final_goal_m=float(goal_d[-1]), event_pose_max_error=error,
                      canonical_float64_yaw_max_error_rad=canonical_yaw_error,
                      nonfloor_reports_before_goal=sum(e['sim_time'] <= goal_time for e in nonfloor),
                      nonfloor_reports_after_goal=sum(e['sim_time'] > goal_time for e in nonfloor),
                      nonfloor_max_impulse=float(max((np.linalg.norm(c['impulse']) for e in nonfloor for c in e['contacts']), default=0)),
                      nonfloor_min_separation=min((c['separation'] for e in nonfloor for c in e['contacts']), default=None),
                      physics_ticks_inside_inference=run['inside_action_ticks'], runtime=runtime,
                      collision_inventory_hash=digest(inventory_path), collision_count=len(inventory),
                      nominal_timing_source_hashes={str(p.relative_to(root)): digest(p) for p in (timing_source, data_source)},
                      exact_inference_ready_robot_pose_available=False)
    return run, xy, yaw, times, wheels, vis, inventory, validation


def build_geometry(cfg, wheels, inventory):
    from research.hospital_spatial_gate import wall_inventory
    walls = wall_inventory(inventory)
    boxes = [dict(name=w['path'], center=(np.array(w['world_aabb_min'][:2])+w['world_aabb_max'][:2])/2,
                  half=(np.array(w['world_aabb_max'][:2])-w['world_aabb_min'][:2])/2, yaw=0.) for w in walls]
    wheelchair_boxes = [dict(name=w['copy_prim'], center=np.array(w['center_world_xyz'][:2]), half=np.array(w['footprint_half_m']), yaw=w['yaw_rad']) for w in wheels.values()]
    return boxes+wheelchair_boxes, walls, wheelchair_boxes


def obstacle_windows(run, xy, times, wheels, vis, radius, goal_time):
    result = {}
    for name, w in wheels.items():
        center = np.array(w['center_world_xyz'][:2]); forward = rotation(w['yaw_rad'])[:, 0]
        along = (xy-center) @ forward
        passed = np.flatnonzero(along > radius+max(w['footprint_half_m']))
        threshold = run['cfg']['controller']['max_v']*3+radius+np.linalg.norm(w['footprint_half_m'])
        near = np.flatnonzero(np.linalg.norm(xy-center, axis=1) <= threshold)
        if not len(near) or not len(passed):
            raise ValueError('No measured obstacle window/passage')
        measured = crossing(xy, center, w['yaw_rad'], w['footprint_half_m'][1], radius)
        i = measured['segment_index']; u = measured['fraction']
        measured['sim_s'] = float(times[i]+u*(times[i+1]-times[i]))
        visible = [e for e, v in zip(run['events'], vis) if v['objects'][name+'_wheelchair']['human_visible_pixel_count'] > 0]
        clear = [e for e, v in zip(run['events'], vis) if v['objects'][name+'_wheelchair']['state'] == 'VISIBLE']
        result[name] = dict(start_sim_s=float(times[near[0]]), end_sim_s=min(float(times[passed[0]]), goal_time),
                            measured_full_passage_sim_s=float(times[passed[0]]), distance_threshold_m=float(threshold), actual_crossing=measured,
                            first_visible_request=visible[0]['request_id'] if visible else None,
                            first_visible_sim_s=visible[0]['observation']['sim_time'] if visible else None,
                            first_clear_request=clear[0]['request_id'] if clear else None,
                            first_clear_sim_s=clear[0]['observation']['sim_time'] if clear else None,
                            last_visible_sim_s=visible[-1]['observation']['sim_time'] if visible else None)
    return result


def pair_metrics(run, xy, yaw, times, wheels, boxes, windows, validation, policy):
    events, cfg = run['events'], run['cfg']; radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']
    worlds = [to_world(c, e['agent_pose_at_observation']) for c, e in zip(run['chunks'], events)]
    nominal = [e['observation']['sim_time']+policy['nominal_dt_s']*np.arange(1, 31) for e in events]
    all_metrics = []
    clearance = lambda p: proxy_clearance(p, boxes, radius, policy['proxy_sample_step_m'])
    for index in range(1, len(events)):
        old, fresh = events[index-1:index+1]; obs, app = fresh['observation'], fresh['application']
        a, b = obs['tick'], app['tick']; sw = app['sim_time']; B = np.array(app['pose'])
        ft, fp, nominal_B = remaining(nominal[index], worlds[index], sw)
        ot, op, _ = remaining(nominal[index-1], worlds[index-1], sw)
        after = np.flatnonzero(nominal[index] > sw+1e-9)
        first = int(after[0]) if len(after) else None
        connector = np.vstack([B[:2], worlds[index][first]]) if first is not None else np.empty((0, 2))
        tan = tangent(fp, obs['pose'][2], policy)
        pending = committed_history(run['robot'], a, b)
        entire_old = committed_history(run['robot'], old['application']['tick'], b)
        movement = pending[-1]-pending[0]; move_norm = float(np.linalg.norm(movement))
        direction = float(np.arctan2(movement[1], movement[0])) if move_norm >= policy['min_segment_m'] else None
        pred = time_comparison(nominal[index-1], worlds[index-1], nominal[index], worlds[index], sw, obs['pose'][2])
        inside = [j for j in range(a+1, b+1) if fresh['inference_start']['monotonic_ns'] <= int(run['robot'][j]['physics_step_start_monotonic_ns']) and int(run['robot'][j]['monotonic_ns']) <= fresh['ready']['monotonic_ns']]
        ready_complete = [j for j in range(a+1, b+1) if int(run['robot'][j]['monotonic_ns']) <= fresh['ready']['monotonic_ns']]
        last_ready = ready_complete[-1] if ready_complete else a
        m = dict(old_id=old['request_id'], fresh_id=fresh['request_id'], primary=is_primary(sw, validation['goal_sim_s']),
                 observation_sim_s=obs['sim_time'], switch_sim_s=sw, observation_to_switch_sim_s=sw-obs['sim_time'],
                 inference_start_monotonic_ns=fresh['inference_start']['monotonic_ns'], ready_monotonic_ns=fresh['ready']['monotonic_ns'],
                 observation_to_switch_wall_s=(app['monotonic_ns']-obs['monotonic_ns'])/1e9,
                 inference_action_wall_s=(fresh['ready']['monotonic_ns']-fresh['inference_start']['monotonic_ns'])/1e9,
                 ready_to_switch_wall_s=(app['monotonic_ns']-fresh['ready']['monotonic_ns'])/1e9,
                 observation_pose=obs['pose'], boundary_pose=app['pose'], nominal_boundary_xy=nominal_B.tolist() if nominal_B is not None else None,
                 boundary_position_m=float(np.linalg.norm(B[:2]-nominal_B)) if nominal_B is not None else None,
                 boundary_yaw_vs_fresh_rad=float(wrap(tan['yaw_rad']-B[2])) if tan['valid'] else None,
                 executed_old_direction_vs_fresh_rad=float(wrap(tan['yaw_rad']-direction)) if tan['valid'] and direction is not None else None,
                 tangent=tan, fresh_native_initial_tangent=tangent(worlds[index], obs['pose'][2], policy), first_native_future_index=first,
                 first_native_future_dt_s=float(nominal[index][first]-sw) if first is not None else None,
                 pending_tick_count=b-a, pending_path_m=float(np.linalg.norm(np.diff(pending, axis=0), axis=1).sum()),
                 pending_displacement_m=move_norm, pending_delta_yaw_rad=float(yaw[b]-yaw[a]),
                 committed_old_tick_range=[old['application']['tick'], b], committed_old_path_m=float(np.linalg.norm(np.diff(entire_old, axis=0), axis=1).sum()),
                 inference_fully_enclosed_tick_count=len(inside), inference_fully_enclosed_path_m=float(sum(np.linalg.norm(xy[j]-xy[j-1]) for j in inside)),
                 inference_fully_enclosed_delta_yaw_rad=float(sum(yaw[j]-yaw[j-1] for j in inside)),
                 ready_last_completed_tick=last_ready, ready_last_completed_sim_s=float(times[last_ready]),
                 pending_path_completed_before_ready_m=float(np.linalg.norm(np.diff(xy[a:last_ready+1], axis=0), axis=1).sum()),
                 ready_pose_note='Last completed recorded physics row only, not exact service-ready pose; next row may bracket ready.',
                 aligned=pred, old_command=old['controller_command'], fresh_command=fresh['controller_command'],
                 delta_v=float(fresh['controller_command'][0]-old['controller_command'][0]), delta_w=float(fresh['controller_command'][1]-old['controller_command'][1]),
                 measured_boundary_body_velocity=app['linear_velocity_body_flu'], measured_boundary_yaw_rate=app['angular_velocity_body_flu'][2],
                 old_proxy=clearance(op), fresh_proxy=clearance(fp), connector_proxy=clearance(connector), boundary_proxy=clearance([B[:2]]),
                 old_full_native_proxy=clearance(worlds[index-1]), fresh_full_native_proxy=clearance(worlds[index]),
                 fresh_nominal_geometry=curvature_diagnostics(ft, fp, policy['min_segment_m'], cfg['controller']['max_v'], cfg['controller']['max_w']),
                 obstacles={})
        m['large_revision_small_command_change'] = bool(pred['valid'] and pred['position_rmse_m'] > radius/2 and abs(m['delta_v']) < .05 and abs(m['delta_w']) < .05)
        if len(connector) and np.linalg.norm(connector[1]-connector[0]) >= policy['min_segment_m']:
            conn_yaw = float(np.arctan2(*(connector[1]-connector[0])[::-1]))
            m['connector_heading_vs_body_rad'] = float(wrap(conn_yaw-B[2]))
            m['connector_heading_vs_fresh_rad'] = float(wrap(tan['yaw_rad']-conn_yaw)) if tan['valid'] else None
            m['connector_nominal_average_turn_rate_rad_s'] = abs(m['connector_heading_vs_body_rad'])/m['first_native_future_dt_s']
        else:
            m['connector_heading_vs_body_rad'] = m['connector_heading_vs_fresh_rad'] = m['connector_nominal_average_turn_rate_rad_s'] = None
        for name, w in wheels.items():
            cross_old = crossing(op, w['center_world_xyz'], w['yaw_rad'], w['footprint_half_m'][1], radius)
            cross_fresh = crossing(fp, w['center_world_xyz'], w['yaw_rad'], w['footprint_half_m'][1], radius)
            own_box = [box for box in boxes if box['name'] == w['copy_prim']]
            po, pf, pc = [proxy_clearance(p, own_box, radius, policy['proxy_sample_step_m']) for p in (op, fp, connector)]
            sides = (cross_old.get('side'), cross_fresh.get('side'))
            flip = sides in (('LEFT', 'RIGHT'), ('RIGHT', 'LEFT'))
            clear_revision = bool(po.get('proxy_overlap_observed') and is_clear(pf, policy) and cross_fresh.get('side') in ('LEFT', 'RIGHT') and pred['valid'] and pred['lateral_rms_m'] > radius/2)
            win = windows[name]
            m['obstacles'][name] = dict(in_window=m['primary'] and sw >= win['start_sim_s'] and obs['sim_time'] <= win['end_sim_s'],
                old_crossing=cross_old, fresh_crossing=cross_fresh, old_proxy=po, fresh_proxy=pf, connector_proxy=pc,
                side_change=flip, meaningful_revision=flip or clear_revision,
                revision_kind='SIDE_FLIP' if flip else 'PROXY_CONFLICT_TO_CLEAR_FUTURE' if clear_revision else 'NO_DEMONSTRATED_NEW_BYPASS',
                predicted_side_matches_actual=cross_fresh.get('side') == win['actual_crossing']['side'] if cross_fresh.get('side') in ('LEFT', 'RIGHT') else None)
        m['category'], m['reasons'] = classify(m, policy)
        for item in m['obstacles'].values():
            item['obstacle_evidence_category'] = ('INSUFFICIENT_EVIDENCE' if item['fresh_crossing']['status']=='NO_PREDICTED_CROSSING' else m['category'])
            item['obstacle_rejection_reason'] = ('NO_PREDICTED_CROSSING; DO_NOT_INFER_SIDE' if item['fresh_crossing']['status']=='NO_PREDICTED_CROSSING' else ';'.join(m['reasons']))
        all_metrics.append(m)
    return all_metrics, worlds, nominal


def flatten(record, prefix=''):
    result = {}
    for key, value in record.items():
        name = prefix+key
        if isinstance(value, dict):
            result.update(flatten(value, name+'.'))
        else:
            result[name] = json.dumps(value) if isinstance(value, list) else value
    return result


def csv_write(path, records):
    fields = list(dict.fromkeys(k for r in records for k in r))
    with path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(records)


def distributions(metrics):
    keys = ('boundary_position_m', 'boundary_yaw_vs_fresh_rad', 'executed_old_direction_vs_fresh_rad',
            'observation_to_switch_sim_s', 'inference_action_wall_s', 'pending_path_m', 'pending_delta_yaw_rad',
            'inference_fully_enclosed_path_m', 'delta_v', 'delta_w', 'aligned.position_rmse_m',
            'aligned.lateral_rms_m', 'aligned.longitudinal_rms_m')
    rows = [flatten(m) for m in metrics if m['primary']]
    result = {}
    for key in keys:
        values = [r.get(key) for r in rows if r.get(key) is not None]
        result[key] = dict(valid=len(values), missing=len(rows)-len(values),
                           minimum=float(min(values)) if values else None, median=float(np.median(values)) if values else None,
                           p95=float(np.quantile(values, .95)) if values else None, maximum=float(max(values)) if values else None,
                           max_absolute=float(max(map(abs, values))) if values else None)
    return result


def select_examples(metrics, windows, radius, policy):
    selected = {}
    for name, window in windows.items():
        rows = [m for m in metrics if m['obstacles'][name]['in_window']]
        valid = [m for m in rows if m['obstacles'][name]['fresh_crossing']['status'] == 'CROSSES' and m['tangent']['valid']]
        chosen = valid[0] if valid else min(rows, key=lambda m: abs(m['switch_sim_s']-window['actual_crossing']['sim_s']))
        selected[name] = dict(fresh_id=chosen['fresh_id'], reason='FIRST_VALID_TANGENT_AND_PREDICTED_CROSSING_IN_FIXED_WINDOW' if valid else 'CLOSEST_SWITCH_TO_MEASURED_PASSAGE_NO_VALID_PREDICTED_CROSSING')
    controls = [m for m in metrics if m['primary'] and m['category'] == 'MISMATCH_WITHOUT_DEMONSTRATED_RISK'
                and not any(o['in_window'] for o in m['obstacles'].values()) and m['tangent']['valid']
                and m['boundary_position_m'] is not None and m['boundary_position_m'] <= radius/4
                and is_clear(m['fresh_proxy'], policy) and is_clear(m['connector_proxy'], policy)]
    selected['control'] = dict(fresh_id=controls[0]['fresh_id'], reason='FIRST_OUTSIDE_WINDOWS_WITH_SMALL_GAP_AND_CLEAR_PROXIES') if controls else dict(fresh_id=None, reason='NO_QUALIFYING_CONTROL_UNDER_FROZEN_RULE')
    return selected


def figures(out, run, xy, times, wheels, boxes, windows, validation, metrics, worlds, nominal, examples, policy):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    radius=run['cfg']['spatial_turn_gate']['robot_conservative_radius_m']; goal_tick=validation['goal_tick']
    def geometry(ax):
        for box in boxes:
            corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*box['half']
            corners=to_world(corners,[*box['center'],box['yaw']])
            wheelchair=box['name'].startswith('/World/ExactWheelchair')
            ax.add_patch(Polygon(corners,facecolor='#b48aca' if wheelchair else '#cccccc',edgecolor='#79498c' if wheelchair else 'none',alpha=.65 if wheelchair else .3))
        ax.set_aspect('equal',adjustable='box');ax.set_xlabel('Hospital world X [m]');ax.set_ylabel('Hospital world Y [m]');ax.grid(alpha=.15)
    fig,ax=plt.subplots(figsize=(13,7));geometry(ax)
    ax.plot(*xy[:goal_tick+1].T,color='#17708e',label='Measured before goal');ax.plot(*xy[goal_tick:].T,'--',color='#929ba6',label='After goal (excluded)')
    for name,win in windows.items():
        choose=(times>=win['start_sim_s'])&(times<=win['end_sim_s']);ax.plot(*xy[choose].T,lw=5,alpha=.45,label=name+' audit window')
        ax.scatter(*win['actual_crossing']['world_xy'],marker='D',s=50)
        ax.annotate(name+' measured passage',win['actual_crossing']['world_xy'],xytext=(-10,12) if name=='C10' else (6,10),textcoords='offset points',ha='right' if name=='C10' else 'left')
    goal=run['cfg']['scene']['goal'][:2];ax.add_patch(Circle(goal,2,color='#ebd991',alpha=.3));ax.scatter(*goal,marker='*',s=100,color='#a78b00');ax.scatter(*xy[goal_tick],s=65,marker='s',label=f'First goal entry {validation["goal_sim_s"]:.3f} s')
    lo=xy.min(0)-2;hi=xy.max(0)+2;ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.legend(fontsize=8);ax.set_title('Figure 1 | E16 successful passage and first goal entry')
    fig.tight_layout();fig.savefig(out/'figure1-overview.png',dpi=170);plt.close(fig)
    for number,name in ((2,'C10'),(3,'C24'),(5,'control')):
        rid=examples[name]['fresh_id'];fig,ax=plt.subplots(figsize=(10,8))
        if rid is None:
            ax.text(.5,.5,'No qualifying control under frozen rule',ha='center',transform=ax.transAxes);ax.axis('off')
        else:
            m=metrics[rid-2];i=rid-1;e=run['events'][i];old=run['events'][i-1]
            geometry(ax)
            history=committed_history(run['robot'],old['application']['tick'],e['application']['tick'])
            a=e['observation']['tick'];b=e['application']['tick'];_,fresh,_=remaining(nominal[i],worlds[i],e['application']['sim_time'])
            ax.plot(*worlds[i-1].T,'--',color='#db8d20',label=f'OLD predicted C{rid-1}')
            ax.plot(*worlds[i].T,color='#3973ad',label=f'FRESH predicted C{rid}')
            ax.plot(*history.T,lw=4,color='#202d3d',label='Committed OLD measured history')
            ax.plot(*xy[a:b+1].T,lw=3,color='#f03d54',label='Measured pending OLD')
            B=np.array(e['application']['pose']);O=np.array(e['observation']['pose'])
            ax.scatter(*O[:2],marker='o',s=65,facecolor='white',edgecolor='blue',label='FRESH observation anchor')
            ax.scatter(*B[:2],marker='X',s=85,color='red',label='Actual switch B');ax.add_patch(Circle(B[:2],radius,fill=False,linestyle=':',color='red',label='Conservative robot disk at B'))
            if m['first_native_future_index'] is not None:
                target=worlds[i][m['first_native_future_index']];ax.plot([B[0],target[0]],[B[1],target[1]],':',lw=2.5,color='magenta',label='Virtual RAW connector (not executed)')
            if m['nominal_boundary_xy'] is not None:ax.scatter(*m['nominal_boundary_xy'],marker='+',s=100,color='green',label='FRESH nominal switch position')
            points=np.vstack([worlds[i-1],worlds[i],history]);center=(points.min(0)+points.max(0))/2
            # Identical 8 m by 8 m world scale for both obstacles and control.
            ax.set_xlim(center[0]-4,center[0]+4);ax.set_ylim(center[1]-4,center[1]+4)
            ax.legend(fontsize=8,loc='upper left');ax.set_title(f'Figure {number} | {name}: C{rid-1} -> C{rid}\n'+m['category'],fontsize=10)
        fig.tight_layout();fig.savefig(out/f'figure{number}-{name.lower()}.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(5,1,figsize=(14,12),sharex=True);t=np.array([m['switch_sim_s'] for m in metrics])
    def line(ax,key,label,scale=1):
        vals=[flatten(m).get(key) for m in metrics];ax.plot(t,[scale*v if v is not None else np.nan for v in vals],'.-',label=label,ms=3)
    line(axes[0],'boundary_position_m','Boundary position [m]');axes[0].axhline(radius/4,color='gray',ls=':',label='r/4 descriptive threshold')
    line(axes[1],'boundary_yaw_vs_fresh_rad','B yaw vs FRESH [deg]',180/np.pi);line(axes[1],'executed_old_direction_vs_fresh_rad','OLD motion vs FRESH [deg]',180/np.pi)
    invalid=[m['switch_sim_s'] for m in metrics if not m['tangent']['valid']];axes[1].scatter(invalid,[0]*len(invalid),marker='x',color='red',label='Invalid tangent (not zero heading)')
    line(axes[2],'aligned.lateral_rms_m','Lateral revision RMS [m]');line(axes[2],'aligned.longitudinal_rms_m','Longitudinal revision RMS [m]')
    line(axes[3],'pending_path_m','Pending OLD travel [m]');line(axes[3],'inference_fully_enclosed_path_m','Travel during fully enclosed inference ticks [m]')
    line(axes[4],'delta_v','delta_v [m/s]');line(axes[4],'delta_w','delta_w [rad/s]')
    for ax in axes:
        for name,win in windows.items():
            ax.axvspan(win['start_sim_s'],win['end_sim_s'],alpha=.09,color='blue' if name=='C10' else 'orange')
            if win['first_visible_sim_s'] is not None:ax.axvline(win['first_visible_sim_s'],color='blue' if name=='C10' else 'orange',ls=':',lw=.8)
            ax.axvline(win['measured_full_passage_sim_s'],color='blue' if name=='C10' else 'orange',ls='--',lw=.8)
        ax.axvspan(validation['goal_sim_s'],times[-1],color='gray',alpha=.15);ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
    for name,win in windows.items():
        axes[0].text((win['start_sim_s']+win['end_sim_s'])/2,.93,name,ha='center',va='top',transform=axes[0].get_xaxis_transform())
    axes[-1].set_xlabel('Switch simulation time [s]');fig.suptitle('Figure 4 | All handoffs; shaded obstacle windows, dotted visibility, dashed passage; gray after goal')
    fig.tight_layout(rect=[0,0,1,.975]);fig.savefig(out/'figure4-metrics.png',dpi=160);plt.close(fig)


def prepare_only(source, out, root):
    """Reserve fresh destination and freeze unchanged rules before any metrics."""
    if out == source or out.is_relative_to(source) or source.is_relative_to(out):
        raise ValueError('Output must be separate from source')
    if out.exists():
        raise FileExistsError('New output directory required')
    if subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip() != 'TIC-VLA-TEST':
        raise ValueError('Only TIC-VLA-TEST authorized')
    before = snapshot(source)
    raw = read(source/'raw-sha256.json')
    if any(before.get(k) != h for k,h in raw.items()):
        raise ValueError('Original raw manifest mismatch')
    out.mkdir(parents=True,exist_ok=False)
    write(out/'protocol.json', {**FROZEN_POLICY, 'source':str(source)})
    write(out/'source-before.json', before)
    write(out/'start-provenance.json', {'branch':'TIC-VLA-TEST',
        'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        'remote_head':subprocess.check_output(['git','rev-parse','origin/TIC-VLA-TEST'],cwd=root,text=True).strip(),
        'status':subprocess.check_output(['git','status','--short'],cwd=root,text=True),
        'raw_hashes_verified':len(raw), 'protected_run_files':len(before)})
    print('PREPARED_ONLY',out)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true',help='Reserve fresh output and freeze analysis protocol; no metrics')
    args=parser.parse_args();source=args.source.resolve();out=args.output.resolve();root=Path(__file__).resolve().parents[1]
    if args.prepare_only:
        prepare_only(source,out,root);return 0
    if out==source or out.is_relative_to(source) or source.is_relative_to(out):raise ValueError('Output must be separate from source')
    if not (out/'protocol.json').is_file() or (out/'final-verdict.json').exists():raise ValueError('Fresh prepared output with frozen protocol required')
    policy=read(out/'protocol.json');before=read(out/'source-before.json');verify_snapshot(source,before)
    raw_manifest=read(source/'raw-sha256.json')
    if any(before.get(k)!=h for k,h in raw_manifest.items()):raise ValueError('Original raw manifest mismatch')
    git=lambda *a: subprocess.check_output(['git',*a],cwd=root,text=True).strip()
    if git('branch','--show-current')!='TIC-VLA-TEST':raise ValueError('Only TIC-VLA-TEST authorized')
    provenance=dict(branch=git('branch','--show-current'),head=git('rev-parse','HEAD'),remote_head=git('rev-parse','origin/TIC-VLA-TEST'),
                    dirty_status=git('status','--short'),tracked_diff_sha256=hashlib.sha256(git('diff','HEAD').encode()).hexdigest(),
                    python=sys.version,numpy=np.__version__,argv=sys.argv,raw_manifest_entries=len(raw_manifest),
                    analyzer_sha256=digest(Path(__file__)),loader_sha256=digest(root/'research/nova_evidence.py'),
                    protocol_sha256=digest(out/'protocol.json'),forbidden_graph_sources_imported=False)
    write(out/'analysis-provenance.json',provenance)
    try:
        run,xy,yaw,times,wheels,vis,inventory,validation=verify_data(source,root)
        boxes,walls,wheelboxes=build_geometry(run['cfg'],wheels,inventory);radius=run['cfg']['spatial_turn_gate']['robot_conservative_radius_m']
        windows=obstacle_windows(run,xy,times,wheels,vis,radius,validation['goal_sim_s'])
        validation['measured_trace_proxy_before_goal']=proxy_clearance(xy[:validation['goal_tick']+1],boxes,radius,policy['proxy_sample_step_m'])
        validation['measured_trace_wheelchair_proxies']={name:proxy_clearance(xy[:validation['goal_tick']+1],
            [box for box in boxes if box['name']==w['copy_prim']],radius,policy['proxy_sample_step_m']) for name,w in wheels.items()}
        write(out/'windows-before-metrics.json',windows)
        metrics,worlds,nominal=pair_metrics(run,xy,yaw,times,wheels,boxes,windows,validation,policy)
        assert [m['fresh_id'] for m in metrics]==list(range(2,91))
        primary=[m for m in metrics if m['primary']];counts=dict(Counter(m['category'] for m in primary))
        examples=select_examples(metrics,windows,radius,policy)
        candidates=[m['fresh_id'] for m in primary if m['category']=='POTENTIAL_RECONCILIATION_HARD_CASE']
        verdict='POTENTIAL_RECONCILIATION_HARD_CASE_FOUND' if candidates else 'NO_RECONCILIATION_HARD_CASE_EVIDENCE'
        final=dict(verdict=verdict,primary_pairs=len(primary),post_goal_pairs=len(metrics)-len(primary),all_pairs=len(metrics),bootstrap_excluded=1,
                   category_counts=counts,candidate_fresh_ids=candidates,representatives=examples,windows=windows,
                   geometry_scope=dict(robot_proxy_radius_m=radius,structure_AABBs=len(walls),wheelchair_OBBs=2,exact_swept_mesh_available=False),
                   policy_sha256=digest(out/'protocol.json'),source_run=source.name,graph_executed=False,
                   interpretation='No positive A evidence is not proof Graph is globally unnecessary. Proxy ambiguity/invalid headings remain explicitly unqualified.')
        write(out/'data-validation.json',validation);write(out/'pair-metrics.json',metrics);write(out/'distributions.json',distributions(metrics))
        csv_write(out/'pair-metrics.csv',[flatten(m) for m in metrics])
        csv_write(out/'pair-accounting.csv',[dict(old_id=m['old_id'],fresh_id=m['fresh_id'],primary=m['primary'],observation_sim_s=m['observation_sim_s'],switch_sim_s=m['switch_sim_s'],boundary_valid=m['boundary_position_m'] is not None,tangent_valid=m['tangent']['valid'],common_time_valid=m['aligned']['valid'],category=m['category'],reasons=';'.join(m['reasons'])) for m in metrics])
        for name in windows:
            own=[dict(old_id=m['old_id'],fresh_id=m['fresh_id'],category=m['category'],reasons=m['reasons'],**m['obstacles'][name]) for m in metrics if m['obstacles'][name]['in_window']]
            write(out/(name.lower()+'-candidate-rejections.json'),own)
        figures(out,run,xy,times,wheels,boxes,windows,validation,metrics,worlds,nominal,examples,policy)
        protected=verify_snapshot(source,before);final['protected_files_unchanged']=protected
        write(out/'final-verdict.json',final)
        print(json.dumps(final,indent=2))
    except Exception as exc:
        verify_snapshot(source,before)
        write(out/'blocked.json',dict(status='BLOCKED',error=repr(exc),no_synthetic_replacement=True))
        raise


if __name__=='__main__':
    raise SystemExit(main())
