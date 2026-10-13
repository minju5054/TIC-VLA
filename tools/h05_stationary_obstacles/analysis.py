"""Saved-only single-obstacle outcome, with fixed-body and contact checks."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def csv_rows(path):
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))


def evaluate(source, comparison):
    from research.nova_evidence import load_nova
    from research.analyze_hospital_turn import nonfloor_contacts
    from research.cart_geometry import path_clearance
    from research.records import write_json
    protected = read(source/'raw-sha256.json')
    for p, expected in protected.items():
        if digest(source/p) != expected:
            raise ValueError('Changed raw evidence: '+p)
    if not (source/'inference-runtime.json').exists():
        raise ValueError('No inference was started; inspect pre-model gates and failure.json')
    completed = (source/'summary.json').exists() and read(source/'summary.json')['status'] == 'PASS'
    run = load_nova(source, require_complete=completed)
    cfg, events, rows = run['cfg'], run['events'], run['robot']
    scenario = 'cart' if 'stationary_cart' in cfg else 'human'
    if ('stationary_human' in cfg) == ('stationary_cart' in cfg):
        raise ValueError('Exactly one scenario obstacle required')
    dense = np.array([[float(r[k]) for k in ('sim_time', 'x', 'y')] for r in rows])
    t, xy = dense[:, 0], dense[:, 1:]
    if not np.isfinite(dense).all() or np.any(np.diff(t) <= 0):
        raise ValueError('Invalid dense states')
    for e in events:
        r = rows[e['observation']['tick']]
        if e['observation']['position'][:2] != [float(r['x']), float(r['y'])] or e['observation']['sim_time'] != float(r['sim_time']):
            raise ValueError('Observation and measured tick mismatch')
    actors = csv_rows(source/f'raw/{scenario}_state.csv')
    actor_xyz = np.array([[float(r[k]) for k in ('x', 'y', 'z')] for r in actors])
    if len(actors) != len(rows) or any(a['tick'] != r['tick'] or a['sim_time'] != r['sim_time'] for a, r in zip(actors, rows)):
        raise ValueError('Obstacle measurements must match every robot tick')
    motion = float(np.linalg.norm(actor_xyz-actor_xyz[0], axis=1).max())
    center = actor_xyz[0, :2]
    goal = np.asarray(cfg['scene']['goal'][:2]); gd = np.linalg.norm(xy-goal, axis=1)
    entries = np.flatnonzero(gd < 2.)
    first_goal = float(t[entries[0]]) if len(entries) else None
    radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']
    if scenario == 'cart':
        actor_prim = cfg['stationary_cart']['copy_prim']
        cart = cfg['stationary_cart']['cart']
        clearance = path_clearance(xy, cart, radius, t)
        extent = max(cart['footprint_half_m'])+radius
    else:
        actor_prim = cfg['stationary_human']['prim']
        extent = cfg['stationary_human']['radius_m']+radius
        clearance = {'minimum_sampled_clearance_m': float(np.linalg.norm(xy-center, axis=1).min()-extent)}
    cleared = np.flatnonzero(xy[:, 0] < center[0]-extent)
    contacts = [json.loads(line) for line in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    spans = nonfloor_contacts(contacts, cfg['robot']['prim'])
    actor_contacts = [e for e in contacts if e['contacts'] and any(e[k] == actor_prim or e[k].startswith(actor_prim+'/') for k in ('collider0', 'collider1'))]
    until = first_goal if first_goal is not None else float(t[-1])
    before_goal = [s for s in spans if s['start_sim_time'] <= until]
    goal_actor_contacts = [e for e in actor_contacts if (e['sim_time'] or 0) <= until]
    visibility = [read(source/'raw/visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    key = scenario+'_visible_pixel_count'
    seen = [e['request_id'] for e, v in zip(events, visibility) if v[key] > 0]
    clear_ids = [e['request_id'] for e, v in zip(events, visibility) if v['state'] in ('CLEAR', 'VISIBLE')]
    passed = bool(completed and motion <= 1e-6 and len(cleared) and first_goal is not None and seen and not before_goal and not goal_actor_contacts)
    result = {'status': 'PASS_THIS_SCENARIO' if passed else 'FAIL_THIS_SCENARIO', 'scenario': scenario,
        'run_id': source.name, 'runtime_complete': completed, 'predictions': len(events), 'dense_ticks': len(rows),
        'obstacle_center_world_xyz_m': actor_xyz[0].tolist(), 'maximum_obstacle_motion_m': motion,
        'obstacle_fixed': motion <= 1e-6, 'obstacle_measurement': 'Measured PhysX positions' if scenario == 'human' else 'Verified static USD collider transform at every tick; not a dynamic rigid-body measurement',
        'first_visible_request': seen[0] if seen else None, 'first_clear_request': clear_ids[0] if clear_ids else None,
        'first_clear_of_obstacle_sim_s': float(t[cleared[0]]) if len(cleared) else None,
        'closest_goal_distance_m': float(gd.min()), 'first_strict_2m_goal_entry_sim_s': first_goal,
        'final_goal_distance_m': float(gd[-1]), 'last_sim_time_s': float(t[-1]),
        'nonfloor_contact_spans_total': len(spans), 'nonfloor_contact_spans_through_goal_or_end': len(before_goal),
        'obstacle_contact_reports_total': len(actor_contacts), 'obstacle_contacts_through_goal_or_end': len(goal_actor_contacts),
        'first_nonfloor_contact_sim_s': min((s['start_sim_time'] for s in spans), default=None),
        'conservative_clearance_proxy': clearance,
        'criteria': 'Complete runtime, stationary single obstacle visible in actual model RGB, measured westward passage, strict goal entry, no reported obstacle/nonfloor contact through first goal entry.',
        'limitations': ['One run per distinct scene; not a general avoidance qualification or deterministic causal comparison.',
            'Conservative bounds/disk clearance is not exact shape clearance or penetration.',
            '48-prediction loop continues after goal entry; clocks and post-goal contacts reported separately.',
            'Isaac6 modified-copy renderer/custom wrapper plus Isaac5 model-only interpreter; not unchanged Isaac5 reproduction.'],
        'coordinates': 'Original Hospital world origin; X/Y metres, +Z up; no native yaw or SE(2) conversion.'}
    output = source/'derived/fixed-obstacle'
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'summary.json', result)
    write_json(output/'contacts.json', {'nonfloor_spans': spans, 'obstacle_reports': actor_contacts})
    plot_saved(source, comparison, output, run, result, t, xy, center, gd)
    if any(digest(source/p) != expected for p, expected in protected.items()):
        raise ValueError('Raw evidence changed during analysis')
    print(json.dumps(result, indent=2))
    return result


def plot_saved(source, comparison, output, run, result, t, xy, center, gd):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from PIL import Image, ImageDraw, ImageFont
    from research.cart_geometry import outline
    cfg, events = run['cfg'], run['events']
    old = csv_rows(comparison/'robot_state.csv')
    old_xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in old])
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(*old_xy.T, color='#9199a3', ls='--', lw=1.4, label='Previous H05 path (human at a different position)')
    ax.plot(*xy.T, color='#19749a', lw=2.2, label='Measured path in this scene')
    if result['scenario'] == 'cart':
        ax.add_patch(Polygon(outline(cfg['stationary_cart']['cart']), color='#df892a', label='Fixed cart bounds'))
    else:
        ax.add_patch(Circle(center, cfg['stationary_human']['radius_m'], color='#df892a', label='Fixed human collider radius'))
    c24 = read(comparison/'raw/requests/request_000024.json')['agent_pose_at_observation'][:2]
    ax.scatter(*c24, s=50, marker='+', color='#555', label='Previous C24 robot position')
    goal = cfg['scene']['goal'][:2]
    ax.add_patch(Circle(goal, 2, color='#e2c469', alpha=.2))
    ax.scatter(*goal, s=120, marker='*', color='#a47a00', label='Goal / 2 m region')
    ax.scatter(*xy[0], s=55, marker='s', color='#26723d', label='Start')
    ax.scatter(*xy[-1], s=65, marker='X', color='#b23a3a', label='Final')
    first_contact = result['first_nonfloor_contact_sim_s']
    if first_contact is not None:
        i = int(np.argmin(abs(t-first_contact)))
        ax.scatter(*xy[i], s=80, marker='x', color='#b23a3a', label=f'First nonfloor contact {first_contact:.2f} s')
    ax.set(xlabel='Hospital world X [m]', ylabel='Hospital world Y [m]', title=f'C24 {result["scenario"]}-only | {result["status"]}')
    ax.axis('equal'); ax.grid(alpha=.2); ax.legend(fontsize=8, loc='best')
    fig.text(.5, .01, 'Original Hospital origin, +Z up. Scene walls omitted. Prior H05 is descriptive, not a matched obstacle-free control.', ha='center', fontsize=9)
    fig.tight_layout(rect=[0,.04,1,1]); fig.savefig(output/'trajectory.png', dpi=170); plt.close(fig)
    obs_t = np.array([e['observation']['sim_time'] for e in events])
    near = int(np.argmin([np.linalg.norm(np.array(e['observation']['position'][:2])-center) for e in events]))
    indices = sorted(set([0, max(0, (result['first_clear_request'] or 1)-1), max(0,near-1), near, min(len(events)-1,near+1), len(events)-1]))
    fig, axes = plt.subplots(2, 3, figsize=(14, 6.8))
    for ax in axes.flat: ax.axis('off')
    for ax, i in zip(axes.flat, indices):
        with Image.open(source/events[i]['rgb_observation_reference']) as rgb: ax.imshow(rgb)
        ax.set_title(f'C{i+1} | sim {obs_t[i]:.3f} s')
    fig.suptitle(f'{result["scenario"].capitalize()}-only scene | actual saved RGB', fontsize=15)
    fig.tight_layout(rect=[0,0,1,.95]); fig.savefig(output/'rgb-contact-sheet.png', dpi=135); plt.close(fig)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    frames = []
    for e in events:
        with Image.open(source/e['rgb_observation_reference']) as original:
            rgb = original.convert('RGB'); rgb.thumbnail((960, 540), Image.Resampling.LANCZOS)
        canvas = Image.new('RGB', (960, 572), '#14212c'); canvas.paste(rgb, (0,32))
        ImageDraw.Draw(canvas).text((12,7), f'{result["scenario"]} only | C{e["request_id"]:02d} | sim {e["observation"]["sim_time"]:.3f} s | saved RGB', font=font, fill='white')
        frames.append(canvas.quantize(colors=192))
    durations = np.r_[np.diff(obs_t), max(.02, t[-1]-obs_t[-1])]
    frames[0].save(output/'rgb-observations.gif', save_all=True, append_images=frames[1:],
        duration=[max(20, round(1000*d)) for d in durations], loop=0, disposal=2, optimize=False)
