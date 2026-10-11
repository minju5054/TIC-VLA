"""Bounded cart reveal qualification. All geometry is derived; no model/Isaac imports."""
import numpy as np
from research.cart_geometry import rotation, clearance, path_clearance
from research.cart_bypass import forward_crossing, travel_coordinates
from research.nova_dynamic_geometry import remaining_curve
from research.reveal_window import first_clear


def cart_at(events, bounds, request_id, left_offset, floor_z=0.):
    """World metres; yaw CCW about +Z. USD row-vector matrix includes pivot offset."""
    i = request_id - 1
    if [e['request_id'] for e in events[i-1:i+2]] != [request_id-1, request_id, request_id+1]:
        raise ValueError('Exact adjacent observation IDs required')
    lo, hi = np.asarray(bounds, float)
    if lo.shape != (3,) or np.any(hi <= lo) or not np.isfinite([lo, hi]).all():
        raise ValueError('Invalid cart bounds')
    delta = np.asarray(events[i+1]['agent_pose_at_observation'][:2]) - events[i-1]['agent_pose_at_observation'][:2]
    if np.linalg.norm(delta) < 1e-9:
        raise ValueError('Undefined route tangent')
    tangent = float(np.arctan2(delta[1], delta[0]))
    forward, normal = rotation(tangent).T
    xy = np.asarray(events[i]['agent_pose_at_observation'][:2]) + left_offset * normal
    axis = int(np.argmax((hi-lo)[:2]))
    yaw = float((tangent-axis*np.pi/2+np.pi) % (2*np.pi)-np.pi)
    center = np.r_[xy, floor_z]
    pivot = np.r_[(lo[:2]+hi[:2])/2, lo[2]]
    matrix = np.eye(4); matrix[:2, :2] = rotation(yaw).T
    matrix[3, :3] = center-pivot@matrix[:3, :3]
    return {'center_world_xyz': center.tolist(), 'yaw_rad': yaw, 'tangent_rad': tangent,
            'long_axis_local': axis, 'footprint_half_m': ((hi-lo)[:2]/2).tolist(),
            'height_m': float(hi[2]-lo[2]), 'matrix_row_vector': matrix.tolist(),
            'local_bounds': [lo.tolist(), hi.tolist()]}


def temporal_proxy(run, cart, fresh_id, radius, dt=.1):
    if not 2 <= fresh_id <= len(run['events']):
        raise ValueError('Adjacent OLD/FRESH required')
    old, fresh = run['events'][fresh_id-2:fresh_id]
    ot, ft = old['observation']['sim_time'], fresh['observation']['sim_time']
    t, p = remaining_curve(run['worlds'][fresh_id-2], ot, ft, dt)
    result = path_clearance(p, cart, radius, t) if p is not None else {'minimum_clearance_m': None, 'first_conflict': None}
    hit = result['first_conflict']
    current = float(clearance([fresh['agent_pose_at_observation'][:2]], cart, radius)[0])
    lead = hit['sim_time']-ft if hit else None
    margin = hit['sim_time']-fresh['application']['sim_time'] if hit else None
    return {'old_request_id': fresh_id-1, 'fresh_request_id': fresh_id,
            'observation_sim_time': ft, 'application_sim_time': fresh['application']['sim_time'],
            'old_horizon_end_s': ot+dt*len(run['worlds'][fresh_id-2]),
            'old_remaining_clearance_m': result['minimum_clearance_m'], 'conflict': hit,
            'current_robot_clearance_m': current, 'reveal_lead_s': lead,
            'switch_to_conflict_margin_s': margin,
            'pass': bool(hit and current > 0 and lead > 0 and margin > 0),
            'remaining_times': t.tolist() if t is not None else [],
            'remaining_world_xy': p.tolist() if p is not None else []}


def reveal_decision(rows, temporal, physical_pass, expected_count=48):
    """A complete chronological prefix through first CLEAR suffices, never a later pair."""
    if not rows:
        raise ValueError('No measured visibility evidence')
    for row in rows:
        count = row['cart_visible_pixel_count']
        if row['state'] not in ('HIDDEN', 'MARGINAL', 'CLEAR') or (row['state']=='HIDDEN') != (count==0):
            raise ValueError('HIDDEN requires exactly zero semantic pixels')
    event = first_clear(rows)
    if event is None and len(rows) != expected_count:
        raise ValueError('No first CLEAR and incomplete history')
    failures = []
    if not physical_pass: failures.append('PHYSICAL_OR_CONNECTED_BYPASS_FAIL')
    if rows[0]['state'] != 'HIDDEN': failures.append('C1_NOT_HIDDEN')
    metric = None
    if event is None:
        failures.append('NEVER_CLEAR')
    else:
        if not event['three_prior_hidden']: failures.append('THREE_PRIOR_NOT_HIDDEN')
        if not event['immediate_old_hidden']: failures.append('IMMEDIATE_OLD_NOT_HIDDEN')
        fid = event['fresh_request_id']
        metric = next((p for p in temporal if p['fresh_request_id']==fid), None)
        if metric is None or not metric['conflict']: failures.append('NO_OLD_CONFLICT_AT_FIRST_CLEAR')
        if metric:
            if metric['current_robot_clearance_m'] <= 0: failures.append('CURRENT_PROXY_CONTACT')
            if metric['conflict'] and (metric['reveal_lead_s'] <= 0 or metric['switch_to_conflict_margin_s'] <= 0):
                failures.append('NO_POSITIVE_RESPONSE_OPPORTUNITY')
    return {'qualified': not failures, 'failures': failures,
            'first_visible_request_id': next((r['request_id'] for r in rows if r['cart_visible_pixel_count']>0), None),
            'first_clear_request_id': event['fresh_request_id'] if event else None,
            'old_request_id': event['old_request_id'] if event else None,
            'event': event, 'temporal_at_first_clear': metric,
            'evidence_role': 'saved no-cart predictions and rendered hypothetical static cart; not a cart-conditioned pair'}


def densify(polyline, spacing):
    points = np.asarray(polyline, float)
    return np.vstack([np.linspace(a, b, max(1, int(np.ceil(np.linalg.norm(b-a)/spacing))), endpoint=False)
                      for a, b in zip(points[:-1], points[1:])] + [points[-1:]])


def bypass_paths(run, cart, radius, settings):
    """Connected piecewise-linear local bypasses, with measured-path entry/exit anchors."""
    extent = max(cart['footprint_half_m'])+radius
    lead = extent+settings['bypass_connector_length_m']
    before = forward_crossing(run['xy'], run['times'], cart, -lead)
    after = forward_crossing(run['xy'], run['times'], cart, lead)
    if before is None or after is None or before['sim_time'] >= after['sim_time']:
        return []
    f, n = rotation(cart['tangent_rad']).T; c = np.asarray(cart['center_world_xyz'][:2])
    result = []
    for side in [-1, 1]:
        for extra in settings['bypass_extra_offsets_m']:
            offset = side*(min(cart['footprint_half_m'])+radius+extra)
            via = [before['world_xy'], c-(extent+.3)*f+offset*n,
                   c+(extent+.3)*f+offset*n, after['world_xy']]
            points = densify(via, settings['bypass_sample_spacing_m'])
            result.append({'side': side, 'offset_m': float(offset), 'world_xy': points.tolist(),
                           'entry': before, 'exit': after,
                           'cart_clearance_m': path_clearance(points, cart, radius)['minimum_clearance_m']})
    return result


def predicted_passage(points, times, cart, radius, support=.5):
    """Finite native support must pass the entire inflated cart and continue beyond it."""
    points = np.asarray(points, float); times = np.asarray(times, float)
    if len(points) < 2 or len(points) != len(times) or not np.isfinite(points).all() or np.any(np.diff(times)<=0):
        raise ValueError('Finite increasing native support required')
    geometry = path_clearance(points, cart, radius, times)
    extent = max(cart['footprint_half_m'])+radius
    center = forward_crossing(points, times, cart)
    exit_plane = forward_crossing(points, times, cart, extent)
    support_plane = forward_crossing(points, times, cart, extent+support)
    starts_before = bool(travel_coordinates(points[:1], cart)[0, 0] < 0)
    return {**geometry, 'center_crossing': center, 'inflated_exit': exit_plane,
            'support_beyond': support_plane, 'starts_before_obstacle': starts_before,
            'obstacle_bypass_supported': bool(starts_before and center and exit_plane and support_plane
                                             and geometry['minimum_clearance_m']>0),
            'structural_clearance_required_separately': True,
            'not_executed_navigation_success': True}
