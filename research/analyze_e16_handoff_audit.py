"""Saved-only boundary / future audit of two successful E16 no-cart runs.

Native XY remains unchanged. No model, simulation, correspondence or correction.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_no_cart_speed import analyze_run, RULES, INVENTORY
from research.analyze_handoffs import metrics as latency_metrics
from research.route_geometry import temporal_overlap, boundary_seam, interpolate, rotation, wrap
from research.nova_dynamic_geometry import remaining_curve
from research.compare_e16_cart_shift import write_csv
from research.records import write_json, provenance
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination

CONFIG = ROOT / 'configs/research/e16_handoff_audit.json'
PHASES = ['initial_travel', 'corner_approach', 'left_turn', 'hallway_straight', 'near_goal']


def exact_event_pose(event, rows, kind):
    """Application snapshot is B; CSV at that tick still carries OLD targets."""
    state = event[kind]
    row = rows[state['tick']]
    pose = [float(row[k]) for k in ('x', 'y', 'yaw')]
    if (int(row['tick']) != state['tick'] or float(row['sim_time']) != state['sim_time']
            or state['pose'] != pose or state['position'] != [float(row[k]) for k in ('x', 'y', 'z')]
            or event['agent_pose_at_' + kind] != pose):
        raise ValueError(f"C{event['request_id']}: {kind} pose/tick mismatch")
    return row


def command_jump(old, fresh):
    if fresh['old_controller_command'] != old['controller_command']:
        raise ValueError('Handoff OLD differs from preceding applied command')
    a, b = old['controller_command'], fresh['controller_command']
    return dict(old_v=a[0], fresh_v=b[0], delta_v=b[0]-a[0],
                old_w=a[1], fresh_w=b[1], delta_w=b[1]-a[1], abs_delta_w=abs(b[1]-a[1]))


def overlap_metrics(aligned, fresh_yaw, epsilon):
    fields = ['rmse_m', 'max_m', 'mean_abs_forward_m', 'max_abs_forward_m', 'mean_abs_lateral_m', 'max_abs_lateral_m',
              'mean_signed_lateral_m', 'mean_abs_tangent_deg', 'max_abs_tangent_deg']
    if aligned is None:
        return dict.fromkeys(fields) | dict(point_count=0, valid_tangent_count=0,
            invalid_tangent_count=0, start_sim_time=None, end_sim_time=None,
            undefined_reason='No positive-duration common nominal horizon')
    delta = aligned['delta']; local = delta @ rotation(fresh_yaw); lateral = local[:, 1]
    norm = np.linalg.norm(delta, axis=1)
    a, b = np.diff(aligned['old'], axis=0), np.diff(aligned['fresh'], axis=0)
    valid = (np.linalg.norm(a, axis=1) > epsilon) & (np.linalg.norm(b, axis=1) > epsilon)
    angle = np.abs(np.degrees(wrap(np.arctan2(b[:, 1], b[:, 0])-np.arctan2(a[:, 1], a[:, 0]))))[valid]
    return dict(rmse_m=float(np.sqrt(np.mean(norm**2))), max_m=float(norm.max()),
        mean_abs_forward_m=float(np.abs(local[:, 0]).mean()), max_abs_forward_m=float(np.abs(local[:, 0]).max()),
        mean_abs_lateral_m=float(np.abs(lateral).mean()), max_abs_lateral_m=float(np.abs(lateral).max()),
        mean_signed_lateral_m=float(lateral.mean()),
        mean_abs_tangent_deg=float(angle.mean()) if len(angle) else None,
        max_abs_tangent_deg=float(angle.max()) if len(angle) else None,
        point_count=len(norm), valid_tangent_count=int(valid.sum()), invalid_tangent_count=int((~valid).sum()),
        start_sim_time=float(aligned['absolute_times'][0]), end_sim_time=float(aligned['absolute_times'][-1]),
        undefined_reason='OLD or FRESH aligned segment <= numerical length epsilon' if not valid.all() else None)


def editable_overlap(full, old, fresh, old_time, fresh_time, switch_time, dt):
    """Clip the existing nominal overlap and insert the exact switch if interior.

    Interpolate ORIGINAL curves: interpolating an already aligned OLD curve
    could erase OLD knots lying between FRESH knots.
    """
    if full is None or switch_time >= full['absolute_times'][-1]:
        return None
    if switch_time <= full['absolute_times'][0]:
        return full
    q = np.r_[switch_time, full['absolute_times'][full['absolute_times'] > switch_time + 1e-8]]
    if q[-1] != full['absolute_times'][-1]:
        q = np.r_[q, full['absolute_times'][-1]]
    op = interpolate(old_time + dt*np.arange(1, len(old)+1), old, q)
    fp = interpolate(fresh_time + dt*np.arange(1, len(fresh)+1), fresh, q)
    return dict(absolute_times=q, old=op, fresh=fp, delta=fp-op)


def seam_metrics(world, event, times, xy, config):
    s = boundary_seam(world, event['observation'], event['application'], times, xy,
        dt=config['native_waypoint_dt_s'], window=config['executed_tangent_window_s'],
        tangent_epsilon=config['minimum_tangent_displacement_m'])
    reason = s.get('undefined_reason')
    s['robot_heading_undefined_reason'] = (reason or 'FRESH segment <= numerical length epsilon') if s['fresh_tangent_rad'] is None else None
    s['executed_heading_undefined_reason'] = (s['robot_heading_undefined_reason'] or
        'No valid preceding time interval or executed displacement <= numerical length epsilon') if s['raw_executed_to_fresh_tangent_gap_deg'] is None else None
    return s


def phase_at(state, cfg, navigation, goal_radius, dt):
    t = state['sim_time']; xy = np.asarray(state['position'][:2])
    entry = navigation['corner_entry']['sim_time']
    if entry <= t <= navigation['corner_passage_completion_sim_time']:
        return 'left_turn'
    if t > navigation['corner_passage_completion_sim_time']:
        return 'near_goal' if np.linalg.norm(xy-cfg['scene']['goal'][:2]) < goal_radius else 'hallway_straight'
    approach_y = cfg['spatial_turn_gate']['center_y_min_m'] - cfg['controller']['max_v'] * 30 * dt
    return 'corner_approach' if xy[1] >= approach_y else 'initial_travel'


def distribution(values):
    a = np.asarray([v for v in values if v is not None and np.isfinite(v)], float)
    return dict(valid_count=len(a), median=float(np.median(a)) if len(a) else None,
                p95=float(np.percentile(a, 95)) if len(a) else None, max=float(a.max()) if len(a) else None)


def analyze(run, config, inventory):
    events = run['events']; n = config['expected_requests']
    saved = json.loads((run['path']/'summary.json').read_text())
    if (not run['complete'] or len(events) != n or saved['successful_predictions'] != n
            or not saved['all_finite'] or saved['status'] != 'PASS'):
        raise ValueError('Expected completed 48 finite predictions')
    rules = json.loads(RULES.read_text())
    base = analyze_run(run, rules, inventory)  # Reuse command/pose checks and original navigation gates.
    nav = base['summary']
    if not nav['left_turn_completed'] or not nav['goal_region_reached'] or run['contacts']:
        raise ValueError('Expected successful contact-free E16 evidence')
    for e in events:
        exact_event_pose(e, run['robot'], 'observation')
        exact_event_pose(e, run['robot'], 'application')
        if (e['response_detected']['sim_time'] != e['application']['sim_time']
                or e['response_detected']['tick'] != e['application']['tick']):
            raise ValueError('Unlogged physics between detection and application')
    rows, details = [], {}
    dt = config['native_waypoint_dt_s']; eps = config['minimum_tangent_displacement_m']
    for i in range(1, n):
        old, fresh = events[i-1:i+1]; ow, fw = run['worlds'][i-1:i+1]
        ot, ft, st = old['observation']['sim_time'], fresh['observation']['sim_time'], fresh['application']['sim_time']
        full = temporal_overlap(ow, fw, ot, ft, dt, eps)
        edit = editable_overlap(full, ow, fw, ot, ft, st, dt)
        seam = seam_metrics(fw, fresh, run['times'], run['xy'], config)
        r = dict(old_request_id=i, fresh_request_id=i+1,
            phase=phase_at(fresh['application'], run['cfg'], nav, rules['goal_region_radius_m'], dt),
            old_observation_sim_time=ot, observation_sim_time=ft, observation_gap_sim_s=ft-ot,
            switch_sim_time=st, observation_to_switch_sim_s=st-ft,
            model_ready_sim_time=None, model_ready_pose=None,
            ready_sim_undefined_reason='Service ready has wall/monotonic clock only; no simulator pose',
            switch_x_m=fresh['application']['pose'][0], switch_y_m=fresh['application']['pose'][1],
            switch_yaw_rad=fresh['application']['pose'][2],
            boundary_position_m=seam['raw_boundary_position_gap_m'],
            boundary_robot_heading_deg=seam['raw_robot_heading_to_fresh_tangent_gap_deg'],
            boundary_executed_heading_deg=seam['raw_executed_to_fresh_tangent_gap_deg'],
            fresh_tangent_segment_length_m=seam['fresh_tangent_segment_length_m'],
            executed_window_displacement_m=seam['executed_window_displacement_m'],
            position_undefined_reason=seam.get('undefined_reason'),
            robot_heading_undefined_reason=seam['robot_heading_undefined_reason'],
            executed_heading_undefined_reason=seam['executed_heading_undefined_reason'],
            **command_jump(old, fresh), **latency_metrics(fresh))
        r['abs_robot_yaw_change_deg'] = abs(r['robot_yaw_change_deg'])
        for kind in ('observation', 'inference_start', 'ready', 'response_detected', 'application'):
            for clock in ('wall_time_ns', 'monotonic_ns'):
                r[kind+'_'+clock] = fresh[kind][clock]
        for prefix, alignment in [('full', full), ('editable', edit)]:
            r.update({prefix+'_'+k: v for k, v in overlap_metrics(alignment, fresh['agent_pose_at_observation'][2], eps).items()})
        # CSV switch row is pre-application OLD; the next tick measures FRESH response.
        j = fresh['application']['tick']; before, after = run['robot'][j:j+2]
        if (before['active_control_source_request_id'] != str(i)
                or after['active_control_source_request_id'] != str(i+1)
                or [float(after[k]) for k in ('target_v', 'target_w')] != fresh['controller_command']):
            raise ValueError('Switch command/tick sequence mismatch')
        for key, suffix in [('body_v_forward', 'forward_m_s'), ('body_yaw_rate_measured', 'yaw_rad_s')]:
            r['measured_at_switch_'+suffix] = float(before[key])
            r['measured_next_tick_'+suffix] = float(after[key])
            r['measured_next_tick_delta_'+suffix] = float(after[key])-float(before[key])
        r['measured_response_dt_sim_s'] = float(after['sim_time'])-st
        rt, rp = remaining_curve(ow, ot, st, dt)
        rows.append(r)
        details[str(i)] = dict(old_full_world=ow, fresh_full_world=fw, old_remaining_times=rt,
            old_remaining_world=rp, full=full, editable=edit, boundary=seam,
            old_observation=old['observation'], fresh_observation=fresh['observation'], switch=fresh['application'])
    if len(rows) != n-1:
        raise ValueError('Incomplete adjacent pair count')
    keys = ['boundary_position_m', 'boundary_robot_heading_deg', 'boundary_executed_heading_deg',
            'delta_v', 'delta_w', 'abs_delta_w', 'action_wall_s', 'observation_to_switch_wall_s',
            'observation_to_switch_sim_s', 'robot_translation_xy_m', 'robot_yaw_change_deg', 'abs_robot_yaw_change_deg',
            'measured_next_tick_delta_forward_m_s', 'measured_next_tick_delta_yaw_rad_s']
    keys += [prefix+'_'+k for prefix in ('full', 'editable') for k in
             ('rmse_m', 'max_m', 'mean_abs_forward_m', 'max_abs_forward_m', 'mean_abs_lateral_m', 'max_abs_lateral_m', 'mean_abs_tangent_deg', 'max_abs_tangent_deg')]
    groups = {'all': rows, 'excluding_bootstrap_old': rows[1:],
              **{phase: [r for r in rows if r['phase'] == phase] for phase in PHASES}}
    stats = {phase: {k: distribution([r[k] for r in group]) for k in keys} for phase, group in groups.items()}
    # Diagnose the resolution/interpretation of unsmoothed geometric tangents.
    # These are actual saved steps, not estimated physical yaw or uncertainty.
    tangent_diagnostics = []
    for e, chunk in zip(events, run['chunks']):
        segments = np.diff(chunk.astype(float), axis=0); lengths = np.linalg.norm(segments, axis=1)
        tangent_diagnostics.append(dict(request_id=e['request_id'], segment_lengths_m=lengths.tolist(),
            negative_native_forward_segment_count=int((segments[:, 0] < 0).sum()),
            degenerate_segment_count=int((lengths <= eps).sum()),
            native_first_two_points=chunk[:2].tolist(), first_segment_delta_native=segments[0].tolist()))
    return dict(rows=rows, details=details, navigation=nav, stats=stats, tangent_diagnostics=tangent_diagnostics)


def select_cases(results):
    selections = []
    for name, d in results.items():
        rows = d['rows']; med = d['stats']['all']
        small = [r for r in rows if r['boundary_position_m'] is not None and r['boundary_position_m'] <= med['boundary_position_m']['median']]
        normal = [r for r in small if r['boundary_robot_heading_deg'] is not None and r['boundary_robot_heading_deg'] <= med['boundary_robot_heading_deg']['median']]
        choices = [
            ('largest_position', rows, 'boundary_position_m', max),
            ('largest_robot_heading', rows, 'boundary_robot_heading_deg', max),
            ('future_with_lower_half_boundary', normal, 'full_rmse_m', max),
            ('turn_lateral_revision', [r for r in rows if r['phase'] == 'left_turn'], 'full_max_abs_lateral_m', max),
            ('small_boundary_and_future', normal, 'full_rmse_m', min)]
        for category, pool, key, choose in choices:
            valid = [r for r in pool if r[key] is not None]
            chosen = choose(valid, key=lambda r: r[key]) if valid else None
            ranks = {k: (1+sum(r[k] > chosen[k] for r in rows if r[k] is not None) if chosen[k] is not None else None) for k in
                     ('boundary_position_m', 'boundary_robot_heading_deg', 'full_rmse_m', 'full_max_abs_lateral_m')} if chosen else None
            selections.append(dict(run=name, category=category, pair=chosen, descending_ranks=ranks,
                selection_pool_count=len(valid), qualification='Relative descriptive example only, not a hard case' if chosen else 'No valid example'))
    return selections


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {k: serial(v) for k, v in value.items()}
    if isinstance(value, list):
        return [serial(v) for v in value]
    return value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--historical-run', type=Path, required=True)
    p.add_argument('--reproduction-run', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    a = p.parse_args(); sources = [a.historical_run, a.reproduction_run]
    out = validate_destination(a.output_dir, sources)
    hashes = evidence_hashes([*sources, CONFIG, RULES, INVENTORY, Path(__file__), ROOT/'research/plot_e16_handoff_audit.py'])
    config = json.loads(CONFIG.read_text()); inventory = json.loads(INVENTORY.read_text())
    runs = {k: load_saved(path) for k, path in zip(['historical', 'reproduction'], sources)}
    if runs['historical']['cfg'] != runs['reproduction']['cfg']:
        raise ValueError('Expected identical no-cart configuration')
    results = {k: analyze(r, config, inventory) for k, r in runs.items()}
    cases = select_cases(results)
    out.mkdir(parents=True); (out/'derived').mkdir(); (out/'figures').mkdir()
    for name, d in results.items():
        write_json(out/(name+'_pairs.json'), d['rows']); write_csv(out/(name+'_pairs.csv'), d['rows'])
        write_json(out/'derived'/(name+'_geometry.json'), serial(d['details']))
        write_csv(out/(name+'_executed_path.csv'), runs[name]['robot'])
        write_json(out/(name+'_navigation.json'), d['navigation'])
        write_json(out/(name+'_tangent_diagnostics.json'), d['tangent_diagnostics'])
    write_json(out/'summary.json', {k: d['stats'] for k, d in results.items()})
    write_json(out/'selected_cases.json', cases)
    write_csv(out/'phase_statistics.csv', [dict(run=k, phase=phase, metric=metric, **s)
        for k, d in results.items() for phase, stats in d['stats'].items() for metric, s in stats.items()])
    from research.plot_e16_handoff_audit import figures
    figures(out/'figures', runs, results, cases, inventory)
    verify_hashes(hashes)
    if evidence_hashes(sources) != {k: v for k, v in hashes.items() if any(Path(s).resolve() in Path(k).parents for s in sources)}:
        raise ValueError('Source inventory changed')
    write_json(out/'metadata.json', dict(source_files_unchanged=True, source_sha256=hashes,
        analysis_provenance=provenance(ROOT), config=config, requests_per_run=48, pairs_per_run=47,
        total_pairs=94, independent_runs=2, python_version=sys.version, numpy_version=np.__version__,
        command=sys.argv, model_calls=0, simulation_launches=0,
        frame='Native forward/left metres -> world XY via own observation yaw CCW from world +X; Z up; saved quaternion wxyz',
        boundary_note='Actual measured B vs nominal FRESH interpolation, with separate derived tau=0 observation anchor; not commanded teleportation',
        tangent_note='Derived geometric tangent; not native TIC-VLA yaw'))
    print(json.dumps({k: d['stats']['all'] for k, d in results.items()}, indent=2))


if __name__ == '__main__':
    main()
