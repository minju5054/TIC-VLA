"""Saved-only E21 RGB, measured right-turn, contacts and goal-entry evidence."""
from pathlib import Path
import json
import numpy as np
from tools.e21_current_render.runner import read, write, digest
from research.nova_evidence import load_nova
from research.blind_corner import baseline_crossing
from research.analyze_hospital_turn import nonfloor_contacts
from research.nova_bright import bright_pass


def summarize(path):
    run = load_nova(path)
    cfg, events, robot = run['cfg'], run['events'], run['robot']
    xy = np.array([[float(r[k]) for k in ('x','y')] for r in robot])
    times = np.array([float(r['sim_time']) for r in robot])
    yaw = np.unwrap([float(r['yaw']) for r in robot])
    crossing, following = baseline_crossing(events, cfg['blind_corner']['baseline_gate'])
    last = following if following is not None else len(events)-1
    end = events[last]['application']['sim_time']
    entries = [json.loads(l) for l in (path/'raw/robot_contacts.jsonl').read_text().splitlines()]
    spans = nonfloor_contacts(entries, cfg['robot']['prim'])
    required_contacts = [s for s in spans if s['start_sim_time'] <= end]
    rows = []
    for e in events:
        name = Path(e['rgb_observation_reference']).name
        b = read(path/'raw/brightness'/(name+'.json'))
        diag = read(path/'raw/scene_observations'/(name+'.json'))
        if abs(b['observation_sim_time']-e['observation']['sim_time']) > 1e-8:
            raise ValueError('Brightness observation time mismatch')
        rows.append({'request_id': e['request_id'], 'observation_sim_time': e['observation']['sim_time'],
            'pose': e['agent_pose_at_observation'], 'rgb': e['rgb_observation_reference'],
            'pass': bright_pass(b, cfg['bright_validation']), 'camera_clear': not diag['camera_near_scene_collision_paths'],
            'near_black': diag['degraded_near_black'], 'stats': b})
    distance = np.linalg.norm(xy-np.asarray(cfg['scene']['goal'][:2]), axis=1)
    hits = np.flatnonzero(distance < 2.)
    contact_rows = [e for e in entries if e['sim_time'] is not None
        and not all(e[k].startswith(cfg['robot']['prim']+'/') for k in ('collider0','collider1'))
        and any(abs(c['normal'][2]) < .5 for c in e['contacts'])]
    impact = next((e for e in contact_rows if any(np.linalg.norm(c['impulse'])>1e-8 for c in e['contacts'])), None)
    penetration = next((e for e in contact_rows if any(c['separation']<0 for c in e['contacts'])), None)
    checks = {'finite_complete_predictions': len(events) == cfg['simulation']['predictions'],
        'right_turn_exit_region': crossing is not None, 'following_observation': following is not None,
        'required_window_rgb': all(r['pass'] and r['camera_clear'] and not r['near_black'] for r in rows[:last+1]),
        'required_window_no_nonfloor_contacts': not required_contacts}
    summary = {'run_id': path.name, 'runtime_pass': read(path/'summary.json')['status']=='PASS',
        'predictions': len(events), 'max_v_m_s': cfg['controller']['max_v'], 'checks': checks,
        'baseline_pass': all(checks.values()),
        'right_turn_pass': checks['right_turn_exit_region'] and checks['following_observation'] and not required_contacts,
        'crossing_request_id': crossing+1 if crossing is not None else None,
        'following_request_id': following+1 if following is not None else None,
        'crossing_observation': events[crossing]['observation'] if crossing is not None else None,
        'required_window_end_sim_s': end,
        'required_window_failed_rgb_ids': [r['request_id'] for r in rows[:last+1] if not r['pass']],
        'all_failed_rgb_ids': [r['request_id'] for r in rows if not r['pass']],
        'all_camera_proximity_ids': [r['request_id'] for r in rows if not r['camera_clear']],
        'all_near_black_ids': [r['request_id'] for r in rows if r['near_black']],
        'brightness_rules': cfg['bright_validation'], 'nonfloor_contact_spans': spans,
        'required_window_contact_spans': required_contacts, 'nonfloor_contact_report_count': len(contact_rows),
        'first_positive_impulse_contact': impact, 'first_negative_separation_contact': penetration,
        'closest_goal_distance_m': float(distance.min()), 'final_goal_distance_m': float(distance[-1]),
        'goal_entered_strict_2m': bool(len(hits)), 'first_goal_entry_sim_s': float(times[hits[0]]) if len(hits) else None,
        'collision_free_goal_entry': bool(len(hits)) and not any(s['start_sim_time']<=times[hits[0]] for s in spans),
        'final_position': xy[-1].tolist(), 'duration_sim_s': float(times[-1]),
        'total_measured_yaw_change_deg': float(np.degrees(yaw[-1]-yaw[0])),
        'observation_gaps_sim_s': np.diff([e['observation']['sim_time'] for e in events]).tolist(),
        'coordinate_rule': 'Original Hospital world origin, metres, +Z up; yaw CCW from world +X; quaternion wxyz. XY distances. Negative measured yaw is right rotation. No trajectory alignment.',
        'limitations': ['One simulated static run; no general navigation qualification',
            'Native output is observation-body forward/left offsets, with no native yaw',
            'Fixed prediction budget does not stop robot at goal; entry is not stopping',
            'Historical comparison is not an isolated renderer-causality experiment']}
    return run, summary, rows, xy, times


def evaluate(destination, baseline):
    from PIL import Image, ImageOps, ImageDraw
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out = destination/'derived/e21-current-render'
    if out.exists():
        raise FileExistsError(out)
    protected = {str(p): digest(p) for folder in (destination, baseline)
        for p in folder.rglob('*') if p.is_file()}
    run, summary, rows, xy, times = summarize(destination)
    old, previous, oldrows, oldxy, _ = summarize(baseline)
    out.mkdir(parents=True, exist_ok=False)
    write(out/'summary.json', summary)
    write(out/'brightness.json', rows)
    write(out/'historical-summary.json', previous)
    write(out/'trajectory-coordinates.json', {'current_world_xy': xy.tolist(), 'current_sim_times': times.tolist(),
        'historical_world_xy': oldxy.tolist(), 'coordinate_rule': summary['coordinate_rule']})
    fig, axes = plt.subplots(1, 2, figsize=(12, 8), gridspec_kw={'width_ratios':[1,1.3]})
    for ax in axes:
        ax.plot(*oldxy.T, '--', color='#8393a3', label='Historical E21')
        ax.plot(*xy.T, color='#126aa0', label='Current render E21')
        ax.scatter(*xy[0], marker='s', color='#16865f', label='Start')
        ax.scatter(*xy[-1], marker='X', color='#be363e', label='Final')
        ax.set(xlabel='Hospital world X [m]', ylabel='Hospital world Y [m]')
        ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.2)
    axes[0].scatter(*run['cfg']['scene']['goal'][:2], marker='*', s=130, color='#b49300', label='Goal')
    axes[0].set_title('Full measured path')
    gate=run['cfg']['blind_corner']['baseline_gate']
    axes[1].plot([gate['exit_x_max_m']]*2,[gate['exit_y_min_m'],gate['exit_y_max_m']],color='#16865f',label='Existing turn gate boundary')
    for rid in sorted(set([1, *(list(range(2,(summary['following_request_id'] or 12)+1,2))), *([summary['crossing_request_id']] if summary['crossing_request_id'] else [])])):
        r=rows[rid-1];x,y=r['pose'][:2]
        axes[1].scatter(x,y,s=15,color='#126aa0');axes[1].annotate(f'C{rid}',(x,y),xytext=(3,4),textcoords='offset points',fontsize=8)
    axes[1].set(xlim=(15,25),ylim=(-3,7),title='Room exit and right turn')
    axes[0].legend(fontsize=8);axes[1].legend(fontsize=8)
    fig.suptitle(f"E21 current render | right turn {'PASS' if summary['right_turn_pass'] else 'FAIL'} | RGB {'PASS' if summary['checks']['required_window_rgb'] else 'FAIL'}\nmax_v={summary['max_v_m_s']} m/s; no added obstacles")
    fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=180);plt.close(fig)
    chosen=sorted(set([1,2,4,6,8,10,12,16,24,36,len(rows),*([summary['crossing_request_id']] if summary['crossing_request_id'] else [])]))
    sheet=Image.new('RGB',(1440,308*((len(chosen)+2)//3)), 'white');draw=ImageDraw.Draw(sheet)
    for i,rid in enumerate(chosen):
        row=rows[rid-1];im=Image.open(destination/row['rgb']).convert('RGB');im.thumbnail((480,270))
        x=(i%3)*480;y=(i//3)*308;sheet.paste(im,(x,y))
        draw.text((x+8,y+273),f"C{rid} | t={row['observation_sim_time']:.2f}s | RGB {'PASS' if row['pass'] else 'FAIL'}",fill='black')
        draw.text((x+8,y+289),f"mean {row['stats']['mean']:.1f} | saturated {row['stats']['saturated_pixel_fraction']:.2%}",fill='black')
    sheet.save(out/'rgb-contact-sheet.png')
    frames=[];durations=[]
    for i,row in enumerate(rows):
        im=Image.open(destination/row['rgb']).convert('RGB');im.thumbnail((960,540))
        frame=ImageOps.expand(im,border=(0,32,0,0),fill='white');draw=ImageDraw.Draw(frame)
        draw.text((10,8),f"E21 | C{row['request_id']} | sim {row['observation_sim_time']:.2f}s | v cap {summary['max_v_m_s']} m/s",fill='black')
        frames.append(frame)
        dt=(rows[i+1]['observation_sim_time'] if i+1<len(rows) else times[-1])-row['observation_sim_time']
        durations.append(max(10,round(dt*1000)))
    frames[0].save(out/'rgb-observations.gif',save_all=True,append_images=frames[1:],duration=durations,loop=0)
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for label,values,color in [('Historical',oldrows,'#8393a3'),('Current',rows,'#126aa0')]:
        ids=[r['request_id'] for r in values]
        axes[0].plot(ids,[r['stats']['mean'] for r in values],label=label,color=color)
        axes[1].plot(ids,[100*r['stats']['saturated_pixel_fraction'] for r in values],color=color)
    axes[0].axhline(215,color='#bd3c3c',ls='--');axes[1].axhline(5,color='#bd3c3c',ls='--')
    axes[0].set(ylabel='Mean luminance [0..255]');axes[1].set(ylabel='Saturated pixels [%]',xlabel='Observation C# (different paths, not pose-matched)')
    axes[0].legend();fig.tight_layout();fig.savefig(out/'brightness-comparison.png',dpi=160);plt.close(fig)
    for name,expected in protected.items():
        if digest(name)!=expected:raise ValueError('Saved source changed: '+name)
    write(out/'validation.json', {'protected_source_files':len(protected),'all_unchanged':True,
        'source_sha256':protected,'analysis_source_sha256':digest(__file__),
        'rgb_frames':len(frames),'gif_duration_ms':sum(durations)})
    print(json.dumps({k:v for k,v in summary.items() if k not in ('nonfloor_contact_spans','observation_gaps_sim_s','crossing_observation')},indent=2))
    return summary
