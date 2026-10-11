"""Reference construction and geometric evaluation, using saved data only."""
import numpy as np
from research.route_geometry import interpolate, wrap
from research.nova_dynamic_geometry import remaining_curve
from research.analyze_e16_handoff_audit import exact_event_pose
from research.analyze_handoffs import metrics as timing_metrics
from research.se2_graph import reference_at_switch, correspondence, relative_error, tangent_headings


def execution_direction(run, event, config):
    cfg = config['execution_direction']; st = event['application']['sim_time']
    lo = max(float(run['times'][0]), st-cfg['window_s'])
    if lo >= st:return None, dict(valid=False, reason='No preceding executed interval')
    # Only actual measurements at/before B are accessed, never future execution.
    mask = run['times'] <= st
    xy = interpolate(run['times'][mask], run['xy'][mask], np.array([lo, st])); d = xy[1]-xy[0]
    length = float(np.linalg.norm(d)); angle = float(np.arctan2(d[1], d[0]))
    difference = float(abs(np.degrees(wrap(angle-event['application']['pose'][2]))))
    speed = event['application']['linear_velocity_body_flu'][0]
    reason = ('Short preceding executed displacement' if length < cfg['minimum_displacement_m'] else
              'No reliable forward motion' if speed < cfg['minimum_forward_speed_m_s'] else
              'Executed chord disagrees with measured heading' if difference > cfg['maximum_heading_difference_deg'] else None)
    return (angle if reason is None else None), dict(valid=reason is None, reason=reason,
        start_sim_time=lo, end_sim_time=st, displacement_m=length, forward_speed_m_s=speed,
        chord_heading_rad=angle, measured_heading_difference_deg=difference)


def build_pair(run, old_id, config, phase=None):
    old, fresh = run['events'][old_id-1:old_id+1]; ow, fw = run['worlds'][old_id-1:old_id+1]
    for e in [old, fresh]:
        exact_event_pose(e, run['robot'], 'observation'); exact_event_pose(e, run['robot'], 'application')
    st, ft, ot = fresh['application']['sim_time'], fresh['observation']['sim_time'], old['observation']['sim_time']
    boundary = np.array(fresh['agent_pose_at_application'], dtype=float)
    history = run['times'] <= st
    base = dict(old_request_id=old_id, fresh_request_id=old_id+1, bootstrap=old_id==1, phase=phase,
        source_run=str(run['path']), old_native_file=str(run['path']/f'raw/requests/request_{old_id:06d}.npy'),
        fresh_native_file=str(run['path']/f'raw/requests/request_{old_id+1:06d}.npy'),
        old_observation_pose=old['agent_pose_at_observation'], fresh_observation_pose=fresh['agent_pose_at_observation'],
        old_observation_sim_time=ot, fresh_observation_sim_time=ft, switch_sim_time=st, boundary=boundary,
        service_ready_clocks=fresh['ready'], service_ready_sim_pose=None, timing=timing_metrics(fresh),
        executed_times=run['times'][history].copy(), executed_xy=run['xy'][history].copy(),
        native_yaw=None, native_timestamps=None, derived_semantics='Nominal +0.1..3.0s; geometric tangent, not native TIC-VLA yaw')
    rt, rp = remaining_curve(ow, ot, st, config['native_dt_s']);base.update(old_remaining_times=rt, old_remaining_xy=rp)
    try:
        times, ref, diagnostics = reference_at_switch(fw, fresh['agent_pose_at_observation'][:2], ft, st, config)
    except ValueError as exc:
        return base | dict(eligibility='INELIGIBLE', exclusion_reason=str(exc), valid_node_count=0, correspondence_count=0)
    valid = np.isfinite(ref).all(axis=1);segments = np.diff(ref[:, :2], axis=0)
    base.update(times=times, reference=ref, tangent_diagnostics=diagnostics, valid_node_count=int(valid.sum()),
                reference_node_count=len(ref), short_segment_count=int((np.linalg.norm(segments, axis=1)<config['tangent']['short_segment_m']).sum()))
    if not valid.all():
        return base | dict(eligibility='INELIGIBLE', exclusion_reason='Unreliable reference tangent at derived nodes '+str(np.flatnonzero(~valid).tolist()), correspondence_count=0)
    # A backward micro-segment is reported relative to the local chord orientation,
    # never flipped or removed from the XY reference.
    along = np.sum(segments*np.column_stack([np.cos(ref[:-1, 2]), np.sin(ref[:-1, 2])]), axis=1)
    base['backward_chord_relative_segment_count'] = int((along < 0).sum())
    direction, direction_info = execution_direction(run, fresh, config)
    join_reason = direction_info['reason']
    if direction is not None and np.linalg.norm(segments[0]) < config['tangent']['short_segment_m']:
        join_reason = 'Short first remaining FRESH segment; disable join direction factor'
    corr = correspondence(times, ref, ow, ot, config)
    return base | dict(eligibility='ELIGIBLE', exclusion_reason=None, correspondence=corr,
        correspondence_count=len(corr['indices']), executed_direction=direction,
        execution_direction_diagnostic=direction_info, join_direction=direction if join_reason is None else None,
        join_disabled_reason=join_reason)


def rms(values):
    a = np.asarray(values, float)
    return float(np.sqrt(np.mean(a*a))) if a.size else None


def curve_geometry(xy, min_segment):
    d = np.diff(xy, axis=0); lengths = np.linalg.norm(d, axis=1);valid = lengths >= min_segment
    phi = np.arctan2(d[:, 1], d[:, 0]); turns = wrap(np.diff(phi));turn_valid = valid[:-1]&valid[1:]
    curvature = np.divide(turns, .5*(lengths[:-1]+lengths[1:]), out=np.zeros_like(turns), where=turn_valid)
    return dict(lengths=lengths, phi=phi, valid=valid, turns=turns, turn_valid=turn_valid, curvature=curvature)


def evaluate(reference, output, times, boundary, executed_direction, config):
    """No later measured execution used. All comparisons use the same node times."""
    if output is None:return None
    f, x = np.asarray(reference), np.asarray(output);b = np.asarray(boundary);tau = times-times[0];cfg = config['evaluation']
    fresh_error = np.linalg.norm(x[1:, :2]-f[1:, :2], axis=1);rel = relative_error(f, x)
    raw_geom = curve_geometry(f[:, :2], cfg['minimum_segment_m']);geo = curve_geometry(x[:, :2], cfg['minimum_segment_m'])
    near = tau[1:] <= cfg['near_window_s']+1e-9;far = tau[1:] >= cfg['far_start_s']
    turn_near = tau[2:] <= cfg['near_window_s']+1e-9;turn_late = ~turn_near
    near_valid = turn_near&geo['turn_valid'];both_near = near_valid&raw_geom['turn_valid']
    both_all = geo['turn_valid']&raw_geom['turn_valid']
    delta = wrap(geo['turns']-raw_geom['turns']);first_valid = bool(geo['valid'][0])
    bridge = x[1, :2]-b[:2];bridge_valid = np.linalg.norm(bridge)>=cfg['minimum_segment_m']
    bridge_heading = np.arctan2(bridge[1], bridge[0])
    headings, diagnostics = tangent_headings(times, x[:, :2], config['tangent']);hv = np.isfinite(headings[1:])
    gap = lambda a, c: float(abs(np.degrees(wrap(a-c))))
    return dict(boundary_position_m=float(np.linalg.norm(x[0, :2]-b[:2])),
        boundary_pose_heading_deg=gap(x[0, 2], b[2]),
        first_segment_robot_heading_deg=gap(geo['phi'][0], b[2]) if first_valid else None,
        executed_to_future_tangent_deg=gap(geo['phi'][0], executed_direction) if first_valid and executed_direction is not None else None,
        post_connection_first_segment_robot_heading_deg=gap(bridge_heading, b[2]) if bridge_valid else None,
        post_connection_note='B to method node1; diagnostic for RAW only, RAW geometry is not reanchored',
        first_segment_length_m=float(geo['lengths'][0]), first_segment_valid=first_valid,
        relative_translation_rmse_m=rms(np.linalg.norm(rel[:, :2], axis=1)), relative_rotation_rmse_deg=rms(np.degrees(rel[:, 2])),
        segment_length_distortion_rmse_m=rms(geo['lengths']-raw_geom['lengths']),
        shape_turn_distortion_rmse_deg=rms(np.degrees(delta[both_all])), shape_valid_turn_count=int(both_all.sum()),
        pose_heading_change_rmse_deg=rms(np.degrees(wrap(x[1:, 2]-f[1:, 2]))),
        derived_curve_heading_change_rmse_deg=rms(np.degrees(wrap(headings[1:][hv]-f[1:, 2][hv]))),
        pose_to_curve_heading_rmse_deg=rms(np.degrees(wrap(x[1:, 2][hv]-headings[1:][hv]))),
        valid_output_chord_heading_count=int(hv.sum()),
        fresh_xy_rmse_m=rms(fresh_error), near_fresh_xy_rmse_m=rms(fresh_error[near]),
        far_fresh_xy_rmse_m=rms(fresh_error[far]), far_point_count=int(far.sum()),
        endpoint_displacement_m=float(fresh_error[-1]),
        near_heading_total_variation_deg=float(np.abs(np.degrees(geo['turns'][near_valid])).sum()) if near_valid.any() else None,
        near_heading_max_step_deg=float(np.abs(np.degrees(geo['turns'][near_valid])).max()) if near_valid.any() else None,
        late_heading_max_step_deg=float(np.abs(np.degrees(geo['turns'][turn_late&geo['turn_valid']])).max()) if (turn_late&geo['turn_valid']).any() else None,
        near_curvature_change_rmse_rad_per_m=rms((geo['curvature']-raw_geom['curvature'])[both_near]),
        near_curvature_max_abs_rad_per_m=float(np.abs(geo['curvature'][near_valid]).max()) if near_valid.any() else None,
        near_valid_turn_count=int(near_valid.sum()), near_common_curvature_count=int(both_near.sum()),
        invalid_short_segment_count=int((~geo['valid']).sum()),
        tangent_note='Geometric chord/segment angles; optimized pose yaw need not equal XY tangent. Curvature is a polyline diagnostic, not physical turning radius.')
