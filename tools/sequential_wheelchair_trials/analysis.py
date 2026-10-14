"""Saved-only requested wheelchair trial: contacts, visibility, passage side and goal."""
import csv
import json
from pathlib import Path
import numpy as np
from tools.exact_two_humans.analysis import read, digest


def rows(path):
    with path.open() as stream:return list(csv.DictReader(stream))


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


def evaluate(source,comparison):
    from research.nova_evidence import load_nova
    from research.analyze_hospital_turn import nonfloor_contacts
    from research.records import write_json
    hashes=read(source/'raw-sha256.json')
    if any(digest(source/p)!=h for p,h in hashes.items()):raise ValueError('Changed raw evidence')
    run=load_nova(source,require_complete=True);cfg=run['cfg'];events=run['events'];robot=run['robot']
    t=np.array([float(r['sim_time']) for r in robot]);xy=np.array([[float(r[k]) for k in ('x','y')] for r in robot])
    wheel=rows(source/'raw/wheelchairs_state.csv')
    actors={w['id']:[r for r in wheel if r['id']==w['id']] for w in cfg['fixed_wheelchairs']}
    if len(wheel)!=len(cfg['fixed_wheelchairs'])*len(robot):raise ValueError('Expected each wheelchair at every robot tick')
    for actor in actors.values():
        if len(actor)!=len(robot) or any(a['tick']!=r['tick'] or a['sim_time']!=r['sim_time'] for a,r in zip(actor,robot)):raise ValueError('Actor tick mismatch')
    contacts=[json.loads(x) for x in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    spans=nonfloor_contacts(contacts,cfg['robot']['prim']);goal=np.array(cfg['scene']['goal'][:2]);gd=np.linalg.norm(xy-goal,axis=1)
    entry=np.flatnonzero(gd<2.);goal_time=float(t[entry[0]]) if len(entry) else None;end=goal_time if goal_time is not None else float(t[-1])
    vis=[read(source/'raw/object_visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    people={};assets=read(source/'wheelchairs_asset.json')
    for h in cfg['fixed_wheelchairs']:
        key=h['id']+'_wheelchair';actor=actors[h['id']];wa=assets[h['id']]
        xyz=np.array([[float(r[k]) for k in ('x','y','z')] for r in actor]);motion=float(np.linalg.norm(xyz-xyz[0],axis=1).max())
        radius=cfg['spatial_turn_gate']['robot_conservative_radius_m']+max(wa['footprint_half_m'])
        forward=np.array([np.cos(h['reference_tangent_rad']),np.sin(h['reference_tangent_rad'])]);passed=np.flatnonzero((xy-xyz[0,:2])@forward>radius)
        own=[e for e in contacts if e['contacts'] and any(e[k]==h['prim'] or e[k].startswith(h['prim']+'/') for k in ('collider0','collider1'))]
        seen=[e['request_id'] for e,v in zip(events,vis) if v['objects'][key]['human_visible_pixel_count']>0]
        clear=[e['request_id'] for e,v in zip(events,vis) if v['objects'][key]['state']=='VISIBLE']
        people[key]={'position_world_m':xyz[0].tolist(),'measured_xy_error_m':float(np.linalg.norm(xyz[0,:2]-h['position'][:2])),
            'max_position_change_m':motion,'first_visible_request':seen[0] if seen else None,'first_clear_request':clear[0] if clear else None,
            'passed_sim_s':float(t[passed[0]]) if len(passed) else None,'contact_reports':len(own),
            'contact_reports_through_goal_or_end':sum(e['sim_time']<=end for e in own),
            'first_contact_sim_s':min((e['sim_time'] for e in own),default=None),
            'minimum_center_distance_m':float(np.linalg.norm(xy-xyz[0,:2],axis=1).min()),
            'measurement':'Verified static USD collision-geometry transform',
            'maximum_transform_error':max(float(r['matrix_max_error']) for r in actor),
            **passage(t,xy,xyz[0],h['reference_tangent_rad'],radius)}
    through=[s for s in spans if s['start_sim_time']<=end]
    passed=bool(goal_time is not None and not through and all(p['passed_sim_s'] is not None and p['max_position_change_m']<=1e-6 and p['maximum_transform_error']<=1e-12 and p['first_visible_request'] is not None for p in people.values()))
    result={'status':'PASS_THIS_SCENARIO' if passed else 'FAIL_THIS_SCENARIO','run_id':source.name,'predictions':len(events),'robot_ticks':len(robot),
        'configured_max_v_m_s':cfg['controller']['max_v'],'max_recorded_target_v_m_s':max(float(r['target_v']) for r in robot),'max_recorded_applied_v_m_s':max(float(r['applied_v']) for r in robot),'objects':people,'first_goal_entry_sim_s':goal_time,'closest_goal_distance_m':float(gd.min()),'final_goal_distance_m':float(gd[-1]),
        'nonfloor_contact_spans':len(spans),'nonfloor_contact_spans_through_goal_or_end':len(through),'first_nonfloor_contact_sim_s':min((s['start_sim_time'] for s in spans),default=None),
        'last_sim_time_s':float(t[-1]),'coordinates':'Original Hospital origin, XY metres,+Z up. Original tangent world-XY passage plane; Wheelchair static USD matrix, not a dynamic PhysX pose. Observation/application/inference/reasoning clocks remain separate.',
        'limitations':['One modified Isaac6 run, not unchanged Isaac5 reproduction or general avoidance proof.','Historical overlays are descriptive; stochastic/asynchronous runs are not matched causal controls.','Actual mesh/collider retained; wheelchair orientation is long axis parallel to original adjacent-observation tangent.','Goal entry does not mean stop; fixed prediction loop continues.']}
    brightness=[]
    for e in events:
        name=Path(e['rgb_observation_reference']).name
        b=read(source/'raw/brightness'/(name+'.json'))
        brightness.append({'request_id':e['request_id'],**b})
    nonfloor=[e for e in contacts if e['sim_time'] is not None and not all(e[k].startswith(cfg['robot']['prim']+'/') for k in ('collider0','collider1')) and any(abs(c['normal'][2])<.5 for c in e['contacts'])]
    impulse=next((e for e in nonfloor if any(np.linalg.norm(c['impulse'])>1e-8 for c in e['contacts'])),None)
    c24=people.get('C24_wheelchair',{});cross=c24.get('first_centre_plane_crossing')
    left=bool(cross and cross['side']=='left' and c24['passed_sim_s'] is not None and c24['contact_reports_through_goal_or_end']==0)
    result.update(scenario=run['metadata']['scenario'],instruction=cfg['instruction'],
        C24_left_passage_measured=left if c24 else None,
        C24_left_avoidance_and_goal=bool(passed and left) if c24 else None,
        nonfloor_contact_report_count=len(nonfloor),first_positive_impulse_contact=impulse,
        failed_rgb_ids=[b['request_id'] for b in brightness if not b['pass']],
        side_convention='Original saved path tangent f=(cos yaw,sin yaw); left=(-sin yaw,cos yaw). Signed lateral at first measured centre-plane crossing; not inferred intent or native yaw.')
    output=source/'derived/sequential-wheelchairs';output.mkdir(parents=True,exist_ok=False)
    write_json(output/'summary.json',result);write_json(output/'brightness.json',brightness);write_json(output/'contacts.json',{'spans':spans})
    plot(source,comparison,output,events,cfg,result,t,xy,assets)
    if any(digest(source/p)!=h for p,h in hashes.items()):raise ValueError('Raw changed')
    print(json.dumps(result,indent=2));return result


def plot(source,comparison,out,events,cfg,result,t,xy,assets):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Polygon
    from research.cart_geometry import rotation
    from PIL import Image,ImageDraw,ImageFont
    old=rows(comparison/'robot_state.csv');oldxy=np.array([[float(r[k]) for k in ('x','y')] for r in old])
    fig,ax=plt.subplots(figsize=(12,6));ax.plot(*oldxy.T,'--',color='#929aa5',label='Original path, no added obstacle')
    ax.plot(*xy.T,color='#167594',lw=2.2,label='Current | max_v '+str(cfg['controller']['max_v'])+' m/s')
    for name,wa in assets.items():
        half=np.array(wa['footprint_half_m']);corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*half
        corners=corners@rotation(wa['yaw_rad']).T+wa['center_world_xyz'][:2]
        ax.add_patch(Polygon(corners,color='#75409a',alpha=.65,label='Fixed '+name+' wheelchair bounds'))
        ax.annotate(name+' exact',wa['center_world_xyz'][:2],xytext=(8,-17),textcoords='offset points')
    goal=cfg['scene']['goal'][:2];ax.add_patch(Circle(goal,2,color='#e2c469',alpha=.2));ax.scatter(*goal,s=120,marker='*',color='#a47a00',label='Goal / 2m')
    ax.scatter(*xy[0],s=60,marker='s',color='#26723d',label='Start');ax.scatter(*xy[-1],s=65,marker='X',color='#b23a3a',label='Final')
    ax.set(xlabel='Hospital world X [m]',ylabel='Hospital world Y [m]',title=source.name+'\n'+result['status']);ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.text(.5,.01,'Original Hospital origin, metres,+Z up. Exact saved XY; walls omitted. Wheelchair shape shown as enclosing footprint.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.04,1,1]);fig.savefig(out/'trajectory.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(15,7))
    for ax,rid in zip(axes.flat,(1,10,18,30,42,len(events))):
        e=events[rid-1]
        with Image.open(source/e['rgb_observation_reference']) as im:ax.imshow(im)
        ax.set_title(f'Observation C{rid} | sim {e["observation"]["sim_time"]:.3f} s');ax.axis('off')
    fig.tight_layout();fig.savefig(out/'rgb-contact-sheet.png',dpi=140);plt.close(fig)
    frames=[];font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for e in events:
        with Image.open(source/e['rgb_observation_reference']) as im:rgb=im.convert('RGB');rgb.thumbnail((960,540),Image.Resampling.LANCZOS)
        canvas=Image.new('RGB',(960,572),'#14212c');canvas.paste(rgb,(0,32));ImageDraw.Draw(canvas).text((12,7),f'{cfg["official_episode"]["episode_id"]} | v cap {cfg["controller"]["max_v"]} m/s | C{e["request_id"]:02d} | sim {e["observation"]["sim_time"]:.3f} s',font=font,fill='white');frames.append(canvas.quantize(colors=192))
    times=np.array([e['observation']['sim_time'] for e in events]);duration=np.r_[np.diff(times),max(.02,t[-1]-times[-1])]
    frames[0].save(out/'rgb-observations.gif',save_all=True,append_images=frames[1:],duration=[max(20,round(1000*d)) for d in duration],loop=0,disposal=2,optimize=False)
