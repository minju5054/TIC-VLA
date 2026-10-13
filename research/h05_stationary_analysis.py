"""Saved-only H05 evaluation. Native curves have no yaw or waypoint timestamps."""
import argparse
from collections import Counter
import csv
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json
from research.two_hz_preflight import read, digest, point_path, RADIUS, HUMAN_RADIUS, MARGIN, SEPARATION
from research.control import rotation_wxyz

CLASSES = ['PREDICTED_COLLISION_OR_LOW_CLEARANCE', 'HORIZON_INSUFFICIENT',
           'SAFE_PREFIX_ONLY', 'COMPLETE_SAFE_BYPASS_PREDICTED']
RULES = {
    'primary_window': 'Through first measured dense XY goal distance strictly <2 m, else normal bounded C48 period remainder; no extension',
    'goal_radius_m_strict': 2., 'robot_radius_m': RADIUS, 'human_radius_m': HUMAN_RADIUS,
    'human_surface_margin_m': MARGIN, 'required_center_separation_m': SEPARATION,
    'downstream_additional_m': 1.,
    'world': 'Hospital original world, metres, +Z up; west forward=(-1,0), LEFT=(0,-1), RIGHT=(0,+1)',
    'native_transform': 'Embed native body forward/left as [dx,dy,0]; world=observation position+R(body-to-world quaternion wxyz)*local; project XY. No reanchoring/cumsum/yaw/timestamps',
    'prediction_geometry': 'Continuous segments of one native curve plus separately identified observation-origin connector; H05 disk/capsule clearance only, no predicted structural guarantee',
    'class_priority': 'Below 0.05 m clearance => unsafe; max west progress < upstream safety plane => horizon insufficient; else endpoint < downstream safety plane+1 m or no center-plane crossing => safe prefix; otherwise complete',
    'relevant': 'Observation and application before first measured westward human-center-plane passage (or primary end); observation upstream; raw pre-CLEAR chunks retained separately',
    'safe_fresh': 'At least one relevant COMPLETE_SAFE_BYPASS_PREDICTED with current CLEAR semantic mask and brightness PASS',
    'clear': 'Existing VISIBLE state: fraction >=0.002, bbox width>=20, height>=80; report as CLEAR',
    'visibility_before_interaction': 'CLEAR/brightness PASS before first continuous disk overlap; if no overlap, before crossing upstream safety plane',
    'useful_early': 'At least 5 m of measured path arc between CLEAR observation and first potential disk conflict; if no conflict, upstream safety-plane crossing is explicitly labeled substitute',
    'contacts': 'Human: measured robot/human PhysX contact with contact points, independent of proxy overlap. Structure: existing near-horizontal-normal nonfloor classifier on /Root contacts',
    'lateral_deviation': 'Maximum absolute world-Y offset from frozen no-human baseline, at matching west progress, within shared straight-corridor X range; also report human-centered offsets',
    'qualification': 'Navigation PASS requires valid preinteraction visibility, downstream safety plane+1m, clearance>=0.05m, no human/structural contact, strict goal entry. Full qualification additionally requires safe_fresh',
    'scientific_runs': 1, 'retry_or_tuning': False,
}


def crossing(xy, human, progress=0.):
    xy = np.asarray(xy, float)
    s = float(human[0])-xy[:, 0]
    hits = np.flatnonzero((s[:-1] < progress) & (s[1:] >= progress))
    if not len(hits):
        return None
    i = int(hits[0]); u = float((progress-s[i])/(s[i+1]-s[i]))
    p = xy[i]+u*(xy[i+1]-xy[i]); lateral = float(human[1]-p[1])
    return {'segment': i, 'fraction': u, 'xy': p.tolist(), 'progress_m': progress,
            'left_offset_m': lateral, 'side': 'LEFT' if lateral > 1e-6 else 'RIGHT' if lateral < -1e-6 else 'CENTER'}


def world_curve(chunk, observation):
    local = np.column_stack([chunk, np.zeros(len(chunk))])
    return (local@rotation_wxyz(observation['quaternion_wxyz']).T + observation['position'])[:, :2]


def classify_prediction(world, origin, human):
    connected = np.vstack([origin, world])
    minimum = point_path(connected, human[:2], RADIUS+HUMAN_RADIUS)
    clearance = minimum['distance_m']-RADIUS-HUMAN_RADIUS
    s = human[0]-connected[:, 0]
    passed = crossing(connected, human)
    if clearance < MARGIN:
        label = CLASSES[0]
    elif max(s) < -SEPARATION:
        label = CLASSES[1]
    elif s[-1] < SEPARATION+1. or passed is None:
        label = CLASSES[2]
    else:
        label = CLASSES[3]
    return {'classification': label, 'minimum_center_m': minimum['distance_m'],
        'minimum_surface_clearance_m': clearance, 'closest': minimum,
        'native_only_minimum_surface_clearance_m': point_path(world, human[:2])['distance_m']-RADIUS-HUMAN_RADIUS,
        'max_west_progress_m': float(max(s)), 'endpoint_west_progress_m': float(s[-1]),
        'center_plane_crossing': passed, 'side': passed['side'] if passed else 'NOT_REACHED'}


def interpolate(values, location):
    i, u = location['segment'], location['fraction']
    return float(values[i]+u*(values[i+1]-values[i]))


def summarize_numbers(values):
    a = np.asarray(values, float)
    return {'min': float(min(a)), 'median': float(np.median(a)), 'p95': float(np.percentile(a, 95)), 'max': float(max(a))}


def evaluate(source):
    from research.nova_evidence import load_nova
    from research.analyze_hospital_turn import nonfloor_contacts
    run = load_nova(source, require_complete=not (source/'failure.json').exists())
    cfg, events = run['cfg'], run['events']
    receipt = read(Path(run['metadata']['authorization_receipt']))
    if receipt['rules'] != RULES:
        raise ValueError('Analysis differs from preregistered rules')
    for p, sha in read(source/'raw-sha256.json').items():
        if digest(source/p) != sha:
            raise ValueError('Raw evidence changed: '+p)
    allxy = np.array([[float(r[k]) for k in ['x', 'y']] for r in run['robot']])
    allt = np.array([float(r['sim_time']) for r in run['robot']])
    if np.any(np.diff(allt) <= 0):
        raise ValueError('Nonmonotonic dense simulation trace')
    human_rows = list(csv.DictReader((source/'raw/human_state.csv').open()))
    humans = np.array([[float(r[k]) for k in ['x', 'y', 'z']] for r in human_rows])
    if len(humans) != len(allxy) or np.max(np.linalg.norm(humans-humans[0], axis=1)) > 1e-6:
        raise ValueError('Missing or nonstationary measured human trace')
    human = humans[0]
    goal = np.asarray(cfg['scene']['goal'][:2])
    goals = np.flatnonzero(np.linalg.norm(allxy-goal, axis=1) < 2.)
    cut = int(goals[0]) if len(goals) else len(allxy)-1
    xy, times = allxy[:cut+1], allt[:cut+1]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    closest = point_path(xy, human[:2], RADIUS+HUMAN_RADIUS, arc)
    margin = point_path(xy, human[:2], SEPARATION, arc)
    passage = crossing(xy, human)
    downstream = crossing(xy, human, SEPARATION)
    beyond = crossing(xy, human, SEPARATION+1.)
    for location in [closest, margin, passage, downstream, beyond, closest['first_conflict']]:
        if location is not None:
            location['sim_time'] = interpolate(times, location)
    interaction = closest['first_conflict'] or crossing(xy, human, -SEPARATION)
    if interaction:
        interaction = {**interaction, 'sim_time': interpolate(times, interaction),
                       'arc_m': interpolate(arc, interaction),
                       'basis': 'continuous disk conflict' if closest['first_conflict'] else 'upstream safety-plane crossing (no disk conflict)'}
    contact_events = [__import__('json').loads(s) for s in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    human_contacts = [e for e in contact_events if e['contacts'] and any(e[k].startswith(cfg['stationary_human']['prim']+'/') for k in ['collider0', 'collider1'])]
    structural = [s for s in nonfloor_contacts(contact_events, cfg['robot']['prim']) if any(p.startswith('/Root/') for p in s['colliders'])]
    primary_human_contacts = [e for e in human_contacts if e['sim_time'] is not None and 0 <= e['sim_time'] <= times[-1]]
    primary_structural = [s for s in structural if s['start_sim_time'] <= times[-1]]
    visibility, predictions = [], []
    passage_time = passage['sim_time'] if passage else times[-1]
    for event, chunk in zip(events, run['chunks']):
        obs = event['observation']; rid = event['request_id']; name = f'rgb_{rid:06d}.png.json'
        vis = read(source/'raw/visibility'/name)
        brightness = read(source/'raw/brightness'/name)
        item = {'request_id': rid, 'observation_sim_time': obs['sim_time'], **vis,
                'state': 'CLEAR' if vis['state'] == 'VISIBLE' else vis['state'],
                'brightness': brightness, 'robot_to_human_center_m': float(np.linalg.norm(np.array(obs['position'][:2])-human[:2])),
                'rgb': event['rgb_observation_reference']}
        item['before_interaction'] = bool(interaction and obs['sim_time'] < interaction['sim_time'])
        item['remaining_measured_path_to_interaction_m'] = float(interaction['arc_m']-np.interp(obs['sim_time'], times, arc)) if interaction else None
        item['valid_early_5m'] = bool(item['before_interaction'] and item['state'] == 'CLEAR' and brightness['pass'] and item['remaining_measured_path_to_interaction_m'] >= 5.)
        visibility.append(item)
        world = world_curve(chunk, obs)
        pred = classify_prediction(world, obs['position'][:2], human)
        relevant = obs['position'][0] > human[0] and obs['sim_time'] < passage_time and event['application']['sim_time'] < passage_time
        j = event['lookahead_index']
        predictions.append({'request_id': rid, 'relevant_pre_passage': bool(relevant),
            'current_clear_bright': item['state'] == 'CLEAR' and brightness['pass'],
            'observation': obs, 'application': event['application'], 'world_xy': world.tolist(),
            'observation_origin_connector_separate_from_native': [obs['position'][:2], world[0].tolist()],
            'native_array': f'raw/requests/request_{rid:06d}.npy', 'native_shape': [30, 2],
            'lookahead_index': j, 'lookahead_native_fl_m': chunk[j].tolist(), 'lookahead_world_xy_m': world[j].tolist(),
            'command_vw': event['controller_command'], 'action_call_wall_s': (event['ready']['monotonic_ns']-event['inference_start']['monotonic_ns'])/1e9,
            'observation_to_application_wall_s': (event['application']['monotonic_ns']-obs['monotonic_ns'])/1e9,
            'observation_to_application_sim_s': event['application']['sim_time']-obs['sim_time'],
            'reasoning_new_start_step': event['reasoning_new_start_step'],
            'reasoning_completion_poll_step': event['reasoning_completion_poll_step'],
            'sampled_image_request_ids': event['sampled_image_request_ids'],
            'delay_seconds_input': event['delay_seconds_input'], **pred})
    first_visible = next((v for v in visibility if v['human_visible_pixel_count'] > 0), None)
    first_clear = next((v for v in visibility if v['state'] == 'CLEAR'), None)
    valid_visibility = any(v['before_interaction'] and v['state'] == 'CLEAR' and v['brightness']['pass'] for v in visibility)
    eligible = [p for p in predictions if p['relevant_pre_passage']]
    conditioned = [p for p in eligible if p['current_clear_bright']]
    complete = [p['request_id'] for p in conditioned if p['classification'] == CLASSES[3]]
    clearance = closest['distance_m']-RADIUS-HUMAN_RADIUS
    flags = []
    for condition, flag in [(primary_human_contacts, 'HUMAN_CONTACT'), (primary_structural, 'STRUCTURAL_CONTACT'),
            (clearance < MARGIN, 'LOW_CLEARANCE'), (beyond is None, 'FAILED_TO_BYPASS'),
            (not len(goals), 'GOAL_NOT_REACHED'), (not valid_visibility, 'VISIBILITY_NOT_ESTABLISHED')]:
        if condition:
            flags.append(flag)
    navigation_pass = not flags
    if any(p['classification'] == CLASSES[0] for p in conditioned):
        flags.append('UPSTREAM_PREDICTION_UNSAFE')
    if not complete and any(p['classification'] in CLASSES[1:3] for p in conditioned):
        flags.append('PREDICTION_HORIZON_INSUFFICIENT')
    runtime_pass = (source/'summary.json').exists() and read(source/'summary.json')['status'] == 'PASS' and len(events) == 48
    if not runtime_pass:
        flags.append('IMPLEMENTATION_FAILURE')
    navigation_pass = navigation_pass and runtime_pass
    from research.two_hz_preflight import PRIMARY, load_reference
    reference = load_reference(ROOT/PRIMARY)
    refxy = reference['xy']; gate = cfg['spatial_turn_gate']
    refxy = refxy[refxy[:, 0] <= gate['exit_center_x_max_m']]
    order = np.argsort(refxy[:, 0]); shared = xy[(xy[:, 0] >= min(refxy[:, 0])) & (xy[:, 0] <= max(refxy[:, 0]))]
    deviation = float(np.max(np.abs(shared[:, 1]-np.interp(shared[:, 0], refxy[order, 0], refxy[order, 1])))) if len(shared) else None
    closest_rgb = min(visibility, key=lambda v: abs(v['observation_sim_time']-closest['sim_time']))
    full_min = point_path(allxy, human[:2])
    result = {'run_id': source.name, 'scientific_trials': 1, 'runtime_pass': runtime_pass,
        'predictions': len(events), 'human_navigation': 'HUMAN_NAVIGATION_PASS' if navigation_pass else 'HUMAN_NAVIGATION_FAIL',
        'safe_fresh': 'SAFE_FRESH_CONFIRMED' if complete else 'SAFE_FRESH_INSUFFICIENT_EVIDENCE',
        'qualification': 'TICVLA_H05_UPSTREAM_QUALIFIED' if navigation_pass and complete else 'TICVLA_H05_NOT_QUALIFIED',
        'failure_flags': flags, 'first_visible': first_visible, 'first_clear': first_clear,
        'valid_preinteraction_visibility': valid_visibility, 'useful_visibility_5m': any(v['valid_early_5m'] for v in visibility),
        'interaction_reference': interaction, 'complete_safe_fresh_ids': complete,
        'relevant_prediction_counts': {k: Counter(p['classification'] for p in eligible)[k] for k in CLASSES},
        'clear_conditioned_prediction_counts': {k: Counter(p['classification'] for p in conditioned)[k] for k in CLASSES},
        'primary': {'end_sim_time': float(times[-1]), 'end_tick': int(run['robot'][cut]['tick']),
            'closest': closest, 'minimum_center_m': closest['distance_m'], 'minimum_surface_clearance_m': clearance,
            'margin_violation': clearance < MARGIN, 'first_margin_violation': margin['first_conflict'],
            'human_contact_count': len(primary_human_contacts), 'first_human_contact': primary_human_contacts[0] if primary_human_contacts else None,
            'structural_contact_spans': primary_structural, 'center_plane_passage': passage,
            'downstream_safety_crossing': downstream, 'additional_1m_crossing': beyond,
            'maximum_west_progress_relative_to_human_m': float(np.max(human[0]-xy[:, 0])),
            'goal_entry': {'sim_time': float(times[-1]), 'xy': xy[-1].tolist(), 'distance_m': float(np.linalg.norm(xy[-1]-goal))} if len(goals) else None,
            'maximum_lateral_deviation_from_baseline_m': deviation},
        'full_48': {'end_sim_time': float(allt[-1]), 'last_observation_sim_time': events[-1]['observation']['sim_time'],
            'end_xy': allxy[-1].tolist(), 'end_goal_distance_m': float(np.linalg.norm(allxy[-1]-goal)),
            'minimum_surface_clearance_m': full_min['distance_m']-RADIUS-HUMAN_RADIUS,
            'human_contact_count': len(human_contacts), 'structural_contact_spans': structural,
            'post_goal_duration_sim_s': float(allt[-1]-times[-1])},
        'timing': {k: summarize_numbers([p[k] for p in predictions]) for k in ['action_call_wall_s', 'observation_to_application_wall_s', 'observation_to_application_sim_s']},
        'commands': {k: summarize_numbers([p['command_vw'][i] for p in predictions]) for i, k in enumerate(['v_m_s', 'w_rad_s'])},
        'closest_approach_rgb': {**closest_rgb, 'offset_from_continuous_closest_sim_s': closest_rgb['observation_sim_time']-closest['sim_time']},
        'identity': {'local_sha_at_run': run['metadata']['local_sha'], 'upstream_sha': run['metadata']['upstream_sha'],
            'git_status': run['metadata']['git_status'], 'diff_sha256': run['metadata']['diff_sha256'],
            'authorization_sha256': digest(run['metadata']['authorization_receipt']),
            'selection_sha256': receipt['selection_sha256'], 'baseline_config_sha256': receipt['baseline_config_sha256'], 'weights': receipt['weights']},
        'limitations': ['One adapted Isaac6 trial, Isaac5 model process; no general reliability estimate',
            'Human capsule excludes outstretched arms; geometric proxy overlap and measured PhysX contact differ',
            'Native geometric classes assess H05 only, not global structural planning; actual structural contacts evaluated separately',
            'Nearest saved RGB to closest physical approach may differ in simulation time',
            'Action-call latency is separate from background reasoning age; exact worker-ready timestamp unavailable']}
    out = source/'derived/h05'
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/'evaluation.json', result)
    write_json(out/'predictions.json', predictions)
    write_json(out/'visibility.json', visibility)
    plot(out, result, predictions, xy, allxy, human, goal)
    return result


def plot(out, result, predictions, xy, allxy, human, goal):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(allxy[:, 0], allxy[:, 1], color='.7', label='Full bounded run')
    ax.plot(xy[:, 0], xy[:, 1], color='#087e8b', label='Primary execution')
    ax.scatter(*xy[0], marker='s', c='#087e8b', label='Start')
    ax.add_patch(Circle(goal, 2., fill=False, color='green', linestyle='--', label='Goal region (strict <2 m)'))
    ax.scatter(*goal, marker='*', c='green', s=80)
    for radius, style, label in [(HUMAN_RADIUS, '-', 'H05 human proxy'), (SEPARATION, '--', 'Required robot-center boundary')]:
        ax.add_patch(Circle(human[:2], radius, fill=False, color='#ca3c3c', linestyle=style, label=label))
    for key, marker, label in [('center_plane_passage', '^', 'First center-plane passage'), ('closest', 'x', 'Continuous closest approach')]:
        p = result['primary'][key]
        if p:
            ax.scatter(*(p.get('xy') or p.get('point_xy')), marker=marker, c='#b30059', s=65, label=label)
    contact = result['primary']['first_human_contact']
    if contact:
        # Contact marker at nearest saved dense position, documented on plot.
        t = result['primary']['closest']['sim_time']
        ax.annotate(f"First human contact t={contact['sim_time']:.3f}s", human[:2], xytext=(human[0]+2, human[1]-3), arrowprops={'arrowstyle': '->'})
    ax.set(title=result['qualification'], xlabel='World X (m)', ylabel='World Y (m)', aspect='equal')
    ax.grid(alpha=.2); ax.legend(fontsize=8, loc='lower left'); fig.tight_layout(); fig.savefig(out/'execution-xy.png', dpi=170); plt.close(fig)
    relevant = [p for p in predictions if p['relevant_pre_passage']]
    clear = result['first_clear']['request_id'] if result['first_clear'] else 1
    chosen = sorted(set([max(1, clear-1), clear] + [p['request_id'] for p in relevant[-4:]]))
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(xy[:, 0], xy[:, 1], color='.65', linewidth=2, label='Measured primary path')
    for p in predictions:
        if p['request_id'] not in chosen:
            continue
        w = np.array(p['world_xy']); color = '#c64040' if p['classification'] == CLASSES[0] else '#26756e'
        ax.plot(w[:, 0], w[:, 1], color=color, alpha=.75, label=f"C{p['request_id']} {p['classification'].replace('PREDICTED_COLLISION_OR_LOW_CLEARANCE','LOW_CLEARANCE').replace('COMPLETE_SAFE_BYPASS_PREDICTED','COMPLETE_SAFE')}")
        ax.text(*w[-1], f"C{p['request_id']}", fontsize=8)
        ax.scatter(*p['lookahead_world_xy_m'], s=18, color=color)
    ax.add_patch(Circle(human[:2], SEPARATION, fill=False, linestyle='--', color='black', label='H05 required center boundary'))
    ax.scatter(*human[:2], c='black', s=25)
    ax.axvline(human[0]-SEPARATION-1., color='black', linestyle=':', label='Complete bypass endpoint boundary')
    ax.set(title='Individual observation-anchored native XY curves; dots = lookahead', xlabel='World X (m)', ylabel='World Y (m)', aspect='equal')
    ax.grid(alpha=.2); ax.legend(fontsize=7); fig.tight_layout(); fig.savefig(out/'fresh-curves.png', dpi=170); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.run_dir.resolve())
    print(result['qualification'], result['failure_flags'])
