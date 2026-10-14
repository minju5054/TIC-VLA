"""Saved evidence only: visibility, signed passage side, contacts and goal entry."""
import csv
import json
from pathlib import Path
import numpy as np
from tools.exact_two_humans.analysis import read, digest
from research.records import write_json


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def passage(t, xy, center, yaw, radius):
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    left = np.array([-forward[1], forward[0]])
    rel = np.asarray(xy)-np.asarray(center[:2])
    along, lateral = rel@forward, rel@left
    crossings = np.flatnonzero((along[:-1] < 0) & (along[1:] >= 0))
    crossing = None
    if len(crossings):
        i = int(crossings[0]); u = -along[i]/(along[i+1]-along[i])
        crossing = {'sim_s': float(t[i]+u*(t[i+1]-t[i])),
                    'signed_left_m': float(lateral[i]+u*(lateral[i+1]-lateral[i])),
                    'world_xy': (xy[i]+u*(xy[i+1]-xy[i])).tolist()}
        crossing['side'] = 'left' if crossing['signed_left_m'] > 0 else 'right' if crossing['signed_left_m'] < 0 else 'centre'
    beyond = np.flatnonzero(along > radius)
    return {'first_centre_plane_crossing': crossing,
            'fully_passed_sim_s': float(t[beyond[0]]) if len(beyond) else None}


def evaluate(source, comparison):
    from research.nova_evidence import load_nova
    from research.analyze_hospital_turn import nonfloor_contacts
    hashes = read(source/'raw-sha256.json')
    if any(digest(source/p) != h for p, h in hashes.items()):
        raise ValueError('Raw evidence changed')
    run = load_nova(source, require_complete=True)
    cfg, events, robot = run['cfg'], run['events'], run['robot']
    t = np.array([float(r['sim_time']) for r in robot])
    xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in robot])
    wheel = rows(source/'raw/wheelchairs_state.csv')
    assets = read(source/'wheelchairs_asset.json')
    cart = read(source/'cart_asset.json'); assets['C24_cart'] = cart
    actors = {w['id']: [r for r in wheel if r['id'] == w['id']] for w in cfg['fixed_wheelchairs']}
    actors['C24_cart'] = rows(source/'raw/cart_state.csv')
    if len(wheel) != 2*len(robot):
        raise ValueError('Wheelchair row count mismatch')
    for actor in actors.values():
        if len(actor) != len(robot) or any(a['tick'] != r['tick'] or a['sim_time'] != r['sim_time'] for a, r in zip(actor, robot)):
            raise ValueError('Actor/robot tick mismatch')
    contacts = [json.loads(x) for x in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    spans = nonfloor_contacts(contacts, cfg['robot']['prim'])
    gd = np.linalg.norm(xy-np.asarray(cfg['scene']['goal'][:2]), axis=1)
    entry = np.flatnonzero(gd < 2.)
    goal_time = float(t[entry[0]]) if len(entry) else None
    end = goal_time if goal_time is not None else float(t[-1])
    visibility = [read(source/'raw/object_visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    objects = {}
    for name, asset in assets.items():
        actor = actors[name]
        xyz = np.array([[float(r[k]) for k in ('x', 'y', 'z')] for r in actor])
        cfg_actor = next((w for w in cfg['fixed_wheelchairs'] if w['id'] == name), cfg['adjacent_cart'])
        yaw = cfg_actor.get('reference_tangent_rad', cfg['fixed_wheelchairs'][1]['reference_tangent_rad'])
        key = name if name == 'C24_cart' else name+'_wheelchair'
        own = [e for e in contacts if e['contacts'] and any(e[k] == cfg_actor['prim'] or e[k].startswith(cfg_actor['prim']+'/') for k in ('collider0', 'collider1'))]
        seen = [e['request_id'] for e, v in zip(events, visibility) if v['objects'][key]['human_visible_pixel_count'] > 0]
        clear = [e['request_id'] for e, v in zip(events, visibility) if v['objects'][key]['state'] == 'VISIBLE']
        objects[name] = {**passage(t, xy, xyz[0], yaw, cfg['spatial_turn_gate']['robot_conservative_radius_m']+max(asset['footprint_half_m'])),
            'position_world_m': xyz[0].tolist(),
            'max_position_change_m': float(np.linalg.norm(xyz-xyz[0], axis=1).max()),
            'maximum_transform_error': max(float(r['matrix_max_error']) for r in actor),
            'first_visible_request': seen[0] if seen else None,
            'first_clear_request': clear[0] if clear else None,
            'contact_reports': len(own), 'first_contact_sim_s': min((e['sim_time'] for e in own), default=None),
            'contact_reports_through_goal_or_end': sum(e['sim_time'] <= end for e in own),
            'minimum_center_distance_m': float(np.linalg.norm(xy-xyz[0, :2], axis=1).min())}
    # A post-corner observation is downstream of the C10 passage plane and
    # faces within 45 degrees of the original C24 corridor tangent.
    c10_time = objects['C10']['fully_passed_sim_s']
    tangent = cfg['fixed_wheelchairs'][1]['reference_tangent_rad']
    joint = []
    for e, v in zip(events, visibility):
        yaw = e['observation']['pose'][2]
        post_corner = c10_time is not None and e['observation']['sim_time'] > c10_time and np.cos(yaw-tangent) > np.cos(np.pi/4)
        if post_corner and all(v['objects'][k]['state'] == 'VISIBLE' for k in ('C24_wheelchair', 'C24_cart')):
            joint.append({'request_id': e['request_id'], 'sim_s': e['observation']['sim_time'],
                          'pixels': {k: v['objects'][k]['human_visible_pixel_count'] for k in ('C24_wheelchair', 'C24_cart')}})
    crossing = objects['C24']['first_centre_plane_crossing']
    through = [s for s in spans if s['start_sim_time'] <= end]
    checks = {'goal_region_entered': goal_time is not None,
              'no_nonfloor_reports_through_goal_or_end': not through,
              'all_obstacles_passed': all(v['fully_passed_sim_s'] is not None for v in objects.values()),
              'all_obstacles_fixed': all(v['maximum_transform_error'] <= 1e-12 and v['max_position_change_m'] <= 1e-9 for v in objects.values()),
              'C24_passed_on_left': crossing is not None and crossing['side'] == 'left',
              'both_C24_objects_clear_after_corner_before_crossing': bool(crossing and any(j['sim_s'] < crossing['sim_s'] for j in joint))}
    result = {'status': 'PASS_THIS_SCENARIO' if all(checks.values()) else 'FAIL_THIS_SCENARIO',
              'checks': checks, 'run_id': source.name, 'predictions': len(events), 'robot_ticks': len(robot),
              'objects': objects, 'joint_clear_post_corner': joint,
              'first_goal_entry_sim_s': goal_time, 'closest_goal_distance_m': float(gd.min()),
              'final_goal_distance_m': float(gd[-1]), 'last_sim_time_s': float(t[-1]),
              'final_goal_region_occupied': bool(gd[-1] < 2),
              'max_applied_v_m_s': max(float(r['applied_v']) for r in robot),
              'nonfloor_contact_spans': len(spans), 'nonfloor_contact_spans_through_goal_or_end': len(through),
              'first_nonfloor_contact_sim_s': min((s['start_sim_time'] for s in spans), default=None),
              'coordinates': 'Hospital original origin, XY metres, +Z up. Signed passage side uses original C23->C25 tangent, left=(-sin yaw,cos yaw), approximately world -Y. Robot quaternion wxyz; static USD row matrices local->world.',
              'limitations': ['One modified Isaac6 trial, not official Isaac5 reproduction or a success rate.',
                              'Goal entry is not stopping; fixed 144-call loop continues.',
                              'First plane crossing side describes measured motion, not inferred intent.',
                              'Centre distances and enclosing bounds are not physical surface clearances.',
                              'Observation, inference, reasoning and command application timestamps remain distinct.']}
    output = source/'derived/c24-adjacent-cart-v05'; output.mkdir(parents=True, exist_ok=False)
    write_json(output/'summary.json', result); write_json(output/'contacts.json', {'spans': spans})
    plot(source, comparison, output, events, cfg, result, t, xy, assets)
    if any(digest(source/p) != h for p, h in hashes.items()):
        raise ValueError('Raw evidence changed during analysis')
    print(json.dumps(result, indent=2)); return result


def plot(source, comparison, out, events, cfg, result, t, xy, assets):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from research.cart_geometry import rotation
    from PIL import Image, ImageDraw, ImageFont
    old = rows(comparison/'robot_state.csv')
    oldxy = np.array([[float(r[k]) for k in ('x', 'y')] for r in old])
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(*oldxy.T, '--', color='#929aa5', label='Original path, no added obstacle')
    goal_time = result['first_goal_entry_sim_s']
    before = np.ones(len(t), bool) if goal_time is None else t <= goal_time
    ax.plot(*xy[before].T, color='#167594', lw=2.2, label='Measured path before first goal entry')
    if goal_time is not None:
        i = int(np.flatnonzero(before)[-1]); ax.plot(*xy[i:].T, '--', color='#768799', label='Continued motion after goal entry')
    for name, a in assets.items():
        half = np.asarray(a['footprint_half_m'])
        corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*half
        corners = corners@rotation(a['yaw_rad']).T+a['center_world_xyz'][:2]
        ax.add_patch(Polygon(corners, color='#c77c23' if name == 'C24_cart' else '#75409a', alpha=.7, label=name+' bounds'))
    c24 = np.asarray(assets['C24']['center_world_xyz'][:2]); yaw = cfg['fixed_wheelchairs'][1]['reference_tangent_rad']
    left = np.array([-np.sin(yaw), np.cos(yaw)])
    ax.annotate('', xy=c24+2*left, xytext=c24+left, arrowprops={'arrowstyle':'->'})
    ax.text(*(c24+2.2*left), 'Robot left', ha='center', fontsize=9)
    goal = cfg['scene']['goal'][:2]
    ax.add_patch(Circle(goal, 2, color='#e2c469', alpha=.2)); ax.scatter(*goal, s=120, marker='*', color='#a47a00', label='Goal / 2 m')
    ax.scatter(*xy[0], marker='s', color='#26723d', label='Start'); ax.scatter(*xy[-1], marker='X', color='#b23a3a', label='Final')
    ax.set(xlabel='Hospital world X [m]', ylabel='Hospital world Y [m]', title='C10/C24 wheelchairs + adjacent cart | 0.5 m/s | '+result['status'])
    ax.axis('equal'); ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.text(.5, .01, 'Original Hospital origin; +Z up. Bounds shown, walls omitted. Goal entry does not mean stopping.', ha='center', fontsize=9)
    fig.tight_layout(rect=[0,.04,1,1]); fig.savefig(out/'trajectory.png', dpi=170); plt.close(fig)
    joint = result['joint_clear_post_corner']; mid = joint[0]['request_id'] if joint else 30
    ids = [1,10,max(1,mid-2),mid,min(len(events),mid+8),len(events)]
    fig, axes = plt.subplots(2,3,figsize=(15,7))
    for ax, rid in zip(axes.flat, ids):
        e = events[rid-1]
        with Image.open(source/e['rgb_observation_reference']) as im: ax.imshow(im)
        ax.set_title(f'C{rid} | sim {e["observation"]["sim_time"]:.3f} s'); ax.axis('off')
    fig.tight_layout(); fig.savefig(out/'rgb-contact-sheet.png', dpi=140); plt.close(fig)
    frames = []; font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    for e in events:
        with Image.open(source/e['rgb_observation_reference']) as im:
            rgb = im.convert('RGB'); rgb.thumbnail((960,540), Image.Resampling.LANCZOS)
        canvas = Image.new('RGB', (960,572), '#14212c'); canvas.paste(rgb,(0,32))
        ImageDraw.Draw(canvas).text((12,7), f'C10/C24 wheelchairs + cart | 0.5 m/s | C{e["request_id"]:02d} | sim {e["observation"]["sim_time"]:.3f} s', font=font, fill='white')
        frames.append(canvas.quantize(colors=192))
    times = np.array([e['observation']['sim_time'] for e in events])
    duration = np.r_[np.diff(times), max(.02,t[-1]-times[-1])]
    frames[0].save(out/'rgb-observations.gif', save_all=True, append_images=frames[1:], duration=[max(20,round(1000*d)) for d in duration], loop=0, disposal=2, optimize=False)
