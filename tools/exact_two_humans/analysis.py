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
    from research.records import write_json
    protected = read(source/'raw-sha256.json')
    for p, expected in protected.items():
        if digest(source/p) != expected: raise ValueError('Changed raw evidence: '+p)
    completed = (source/'summary.json').exists() and read(source/'summary.json')['status'] == 'PASS'
    run = load_nova(source, require_complete=completed)
    cfg, events, rows = run['cfg'], run['events'], run['robot']
    dense = np.array([[float(r[k]) for k in ('sim_time','x','y')] for r in rows])
    t, xy = dense[:,0], dense[:,1:]
    if not np.isfinite(dense).all() or np.any(np.diff(t)<=0): raise ValueError('Invalid dense states')
    for e in events:
        r = rows[e['observation']['tick']]
        if e['observation']['position'][:2] != [float(r['x']),float(r['y'])] or e['observation']['sim_time'] != float(r['sim_time']):
            raise ValueError('Observation and measured tick mismatch')
    actors = csv_rows(source/'raw/humans_state.csv')
    if len(actors) != 2*len(rows): raise ValueError('Two measured humans required at every tick')
    goal = np.asarray(cfg['scene']['goal'][:2]); gd=np.linalg.norm(xy-goal,axis=1)
    entries=np.flatnonzero(gd<2.);first_goal=float(t[entries[0]]) if len(entries) else None
    contacts=[json.loads(line) for line in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    spans=nonfloor_contacts(contacts,cfg['robot']['prim'])
    until=first_goal if first_goal is not None else float(t[-1])
    before_goal=[s for s in spans if s['start_sim_time']<=until]
    vis=[read(source/'raw/individual_visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    people={}; all_actor_contacts=[]
    for h in cfg['fixed_humans']:
        measured=[r for r in actors if r['id']==h['id']]
        if len(measured)!=len(rows) or any(a['tick']!=r['tick'] or a['sim_time']!=r['sim_time'] for a,r in zip(measured,rows)):
            raise ValueError('Actor/robot tick mismatch')
        xyz=np.array([[float(r[k]) for k in ('x','y','z')] for r in measured]);center=xyz[0,:2]
        if not np.isfinite(xyz).all():raise ValueError('Nonfinite human pose')
        motion=float(np.linalg.norm(xyz-xyz[0],axis=1).max())
        orientation=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in measured])
        turn=float(np.max(np.abs(orientation-orientation[0])))
        radius=cfg['spatial_turn_gate']['robot_conservative_radius_m']+h['radius_m']
        forward=np.array([np.cos(h['reference_tangent_rad']),np.sin(h['reference_tangent_rad'])])
        progress=(xy-center)@forward
        cleared=np.flatnonzero(progress>radius)
        first_clear=float(t[cleared[0]]) if len(cleared) else None
        actor_contacts=[e for e in contacts if e['contacts'] and any(e[k]==h['prim'] or e[k].startswith(h['prim']+'/') for k in ('collider0','collider1'))]
        all_actor_contacts.extend(actor_contacts)
        seen=[e['request_id'] for e,v in zip(events,vis) if v['people'][h['id']]['human_visible_pixel_count']>0]
        clear=[e['request_id'] for e,v in zip(events,vis) if v['people'][h['id']]['state']=='VISIBLE']
        gate=read(source/'pre-model-gate.json')['people'][h['id']]
        people[h['id']]={'authored_exact_reference_xy':h['position'][:2], 'measured_xy':center.tolist(),
            'maximum_position_motion_m':motion,'maximum_quaternion_component_change':turn,
            'measured_xy_error_m':float(np.linalg.norm(center-np.array(h['reference_observation']['pose'][:2]))),
            'grounded_sole_heights_m':gate['current_mesh_bounds']['sole_minima_world_z'], 'floor_z_m':gate['floor_z_m'],
            'first_visible_request':seen[0] if seen else None,'first_clear_request':clear[0] if clear else None,
            'passed_beyond_human_sim_s':first_clear,
            'minimum_sampled_center_distance_m':float(np.linalg.norm(xy-center,axis=1).min()),
            'minimum_sampled_conservative_surface_clearance_m':float(np.linalg.norm(xy-center,axis=1).min()-radius),
            'contact_reports_total':len(actor_contacts),
            'contact_reports_through_goal_or_end':sum((e['sim_time'] or 0)<=until for e in actor_contacts),
            'passed_and_stationary':bool(first_clear is not None and seen and motion<=1e-6 and turn<=1e-7)}
    passed=bool(completed and first_goal is not None and not before_goal and all(p['passed_and_stationary'] and p['contact_reports_through_goal_or_end']==0 for p in people.values()))
    result={'status':'PASS_THIS_SCENARIO' if passed else 'FAIL_THIS_SCENARIO','run_id':source.name,
        'scenario':'Two grounded stationary humans at exact original no-added-obstacle C10/C24',
        'runtime_complete':completed,'predictions':len(events),'robot_ticks':len(rows),'human_rows':len(actors),
        'people':people,'first_strict_2m_goal_entry_sim_s':first_goal,'closest_goal_distance_m':float(gd.min()),
        'final_goal_distance_m':float(gd[-1]),'last_sim_time_s':float(t[-1]),
        'nonfloor_contact_spans_total':len(spans),'nonfloor_contact_spans_through_goal_or_end':len(before_goal),
        'first_nonfloor_contact_sim_s':min((s['start_sim_time'] for s in spans),default=None),
        'criteria':'Complete runtime, both exact-position grounded stationary humans visible and passed in their original measured travel directions, goal distance<2m, no reported person/nonfloor contacts through first goal entry.',
        'limitations':['One bounded 48-prediction simulation, not general navigation or causal avoidance qualification.',
            'Original no-added-obstacle reference uses a different historical renderer/runtime wrapper; path comparison is descriptive.',
            'Enclosing-disk clearance is conservative, not exact penetration. Grounding changes visual Z and collider centre Z only.',
            'Fixed loop continues after goal entry. Native outputs have no yaw; simulation and wall times remain distinct.'],
        'coordinates':'Original Hospital origin; world XY metres, +Z up. Passage projects robot-minus-human XY onto saved adjacent-observation tangent; direction is world forward, no native SE(2) trajectory.'}
    output=source/'derived/exact-two-humans';output.mkdir(parents=True,exist_ok=False)
    write_json(output/'summary.json',result);write_json(output/'contacts.json',{'nonfloor_spans':spans,'human_reports':all_actor_contacts})
    plot_saved(source,comparison,output,run,result,t,xy,gd)
    if any(digest(source/p)!=h for p,h in protected.items()):raise ValueError('Raw evidence changed')
    print(json.dumps(result,indent=2));return result


def plot_saved(source, comparison, output, run, result, t, xy, gd):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from PIL import Image, ImageDraw, ImageFont
    cfg,events=run['cfg'],run['events']
    old=csv_rows(comparison/'robot_state.csv');old_xy=np.array([[float(r[k]) for k in ('x','y')] for r in old])
    fig,ax=plt.subplots(figsize=(12,6))
    ax.plot(*old_xy.T,color='#939ca5',ls='--',lw=1.5,label='Original path, no added obstacle')
    ax.plot(*xy.T,color='#167594',lw=2.2,label='Measured path with two fixed humans')
    for h in cfg['fixed_humans']:
        center=h['position'][:2]
        ax.add_patch(Circle(center,h['radius_m'],color='#df892a',label='Stationary human' if h['id']=='C10' else None))
        ax.annotate(h['id']+' exact',center,xytext=(9,-16),textcoords='offset points',fontsize=10)
    goal=cfg['scene']['goal'][:2];ax.add_patch(Circle(goal,2,color='#e2c469',alpha=.2))
    ax.scatter(*goal,s=120,marker='*',color='#a47a00',label='Goal / 2 m region')
    ax.scatter(*xy[0],s=55,marker='s',color='#26723d',label='Start');ax.scatter(*xy[-1],s=65,marker='X',color='#b23a3a',label='Final')
    if result['first_nonfloor_contact_sim_s'] is not None:
        i=int(np.argmin(abs(t-result['first_nonfloor_contact_sim_s'])))
        ax.scatter(*xy[i],s=80,marker='x',color='#b23a3a',label='First nonfloor contact')
    ax.set(xlabel='Hospital world X [m]',ylabel='Hospital world Y [m]',title='Exact original C10 + C24 humans | '+result['status'])
    ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.text(.5,.01,'Original Hospital origin, +Z up. Actual saved XY, no placement offsets. Historical path is descriptive; walls omitted.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.04,1,1]);fig.savefig(output/'trajectory.png',dpi=170);plt.close(fig)
    obs_t=np.array([e['observation']['sim_time'] for e in events])
    obs_xy=np.array([e['observation']['position'][:2] for e in events])
    chosen=[0,len(events)-1]
    for h in cfg['fixed_humans']:
        near=int(np.argmin(np.linalg.norm(obs_xy-h['position'][:2],axis=1)))
        first=result['people'][h['id']]['first_clear_request']
        chosen.extend([first-1 if first is not None else near,near])
    chosen=sorted(set(chosen))
    fig,axes=plt.subplots(2,3,figsize=(14,6.8))
    for ax in axes.flat:ax.axis('off')
    for ax,i in zip(axes.flat,chosen):
        with Image.open(source/events[i]['rgb_observation_reference']) as rgb:ax.imshow(rgb)
        ax.set_title(f'C{i+1} | sim {obs_t[i]:.3f} s')
    fig.suptitle('Two fixed grounded humans | actual saved RGB',fontsize=15)
    fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(output/'rgb-contact-sheet.png',dpi=135);plt.close(fig)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18);frames=[]
    for e in events:
        with Image.open(source/e['rgb_observation_reference']) as original:
            rgb=original.convert('RGB');rgb.thumbnail((960,540),Image.Resampling.LANCZOS)
        canvas=Image.new('RGB',(960,572),'#14212c');canvas.paste(rgb,(0,32))
        ImageDraw.Draw(canvas).text((12,7),f'Exact C10 + C24 humans | C{e["request_id"]:02d} | sim {e["observation"]["sim_time"]:.3f} s',font=font,fill='white')
        frames.append(canvas.quantize(colors=192))
    durations=np.r_[np.diff(obs_t),max(.02,t[-1]-obs_t[-1])]
    frames[0].save(output/'rgb-observations.gif',save_all=True,append_images=frames[1:],duration=[max(20,round(1000*d)) for d in durations],loop=0,disposal=2,optimize=False)
