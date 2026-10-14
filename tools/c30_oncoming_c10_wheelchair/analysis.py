"""Saved-only measured encounter, contact and goal-entry analysis."""
import csv
import json
from pathlib import Path
import numpy as np
from tools.exact_two_humans.analysis import read, digest


def rows(path):
    with path.open() as f:return list(csv.DictReader(f))


def evaluate(source,comparison):
    from research.nova_evidence import load_nova
    from research.analyze_hospital_turn import nonfloor_contacts
    from research.records import write_json
    hashes=read(source/'raw-sha256.json')
    for p,h in hashes.items():
        if digest(source/p)!=h:raise ValueError('Changed raw evidence: '+p)
    run=load_nova(source,require_complete=True);cfg=run['cfg'];events=run['events'];robot=run['robot']
    human=rows(source/'raw/human_state.csv');wheel=rows(source/'raw/wheelchairs_state.csv')
    for actor in (human,wheel):
        if len(actor)!=len(robot) or any(a['tick']!=b['tick'] or a['sim_time']!=b['sim_time'] for a,b in zip(actor,robot)):
            raise ValueError('Actor/robot tick mismatch')
    t=np.array([float(r['sim_time']) for r in robot]);xy=np.array([[float(r[k]) for k in ('x','y')] for r in robot])
    hxy=np.array([[float(r[k]) for k in ('x','y')] for r in human]);wxy=np.array([[float(r[k]) for k in ('x','y')] for r in wheel])
    delta=np.linalg.norm(xy-hxy,axis=1);ci=int(delta.argmin())
    hs=np.r_[0.,np.linalg.norm(np.diff(hxy,axis=0),axis=1)/np.diff(t)]
    c=cfg['encounter_check'];corner=((xy>=c['corner_world_xy_min'])&(xy<=c['corner_world_xy_max'])).all(axis=1)
    encounter=np.flatnonzero(corner&(hs>.1)&(delta<=c['maximum_robot_human_center_distance_m']))
    contacts=[json.loads(x) for x in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    spans=nonfloor_contacts(contacts,cfg['robot']['prim'])
    goal=cfg['scene']['goal'][:2];gd=np.linalg.norm(xy-goal,axis=1);entry=np.flatnonzero(gd<2.)
    goal_time=float(t[entry[0]]) if len(entry) else None;end=goal_time if goal_time is not None else float(t[-1])
    through=[s for s in spans if s['start_sim_time']<=end]
    def actor_contacts(prim):
        own=[e for e in contacts if e['contacts'] and any(e[k]==prim or e[k].startswith(prim+'/') for k in ('collider0','collider1'))]
        return {'reports':len(own),'first_sim_s':min((e['sim_time'] for e in own),default=None),
            'through_goal_or_end':sum(e['sim_time']<=end for e in own)}
    h=cfg['oncoming_human'];w=cfg['fixed_wheelchairs'][0];radius=cfg['spatial_turn_gate']['robot_conservative_radius_m']
    heading=np.array([np.cos(float(r['target_yaw'])-np.pi/2) for r in human]);heading_y=np.array([np.sin(float(r['target_yaw'])-np.pi/2) for r in human])
    progress=np.sum((xy-hxy)*np.column_stack([heading,heading_y]),axis=1)
    passed_h=np.flatnonzero(progress < -(radius+h['radius_m']))
    wa=read(source/'wheelchairs_asset.json')['C10'];forward=np.array([np.cos(w['reference_tangent_rad']),np.sin(w['reference_tangent_rad'])])
    passed_w=np.flatnonzero((xy-wxy[0])@forward>radius+max(wa['footprint_half_m']))
    visibility=[read(source/'raw/object_visibility'/f'rgb_{e["request_id"]:06d}.png.json')['objects'] for e in events]
    visible={}
    for key in ('C30_human','C10_wheelchair'):
        seen=[e['request_id'] for e,v in zip(events,visibility) if v[key]['human_visible_pixel_count']>0]
        clear=[e['request_id'] for e,v in zip(events,visibility) if v[key]['state']=='VISIBLE']
        visible[key]={'first_visible_request':seen[0] if seen else None,'first_clear_request':clear[0] if clear else None,'visible_observations':len(seen)}
    human_result={'start_measured_world_xy_m':hxy[0].tolist(),'final_measured_world_xy_m':hxy[-1].tolist(),
        'distance_travelled_m':float(np.linalg.norm(np.diff(hxy,axis=0),axis=1).sum()),
        'maximum_target_error_m':max(float(r['target_error_m']) for r in human),
        'maximum_measured_speed_m_s':float(hs.max()),'closest_robot_center_distance_m':float(delta[ci]),
        'closest_sim_s':float(t[ci]),'robot_at_closest':xy[ci].tolist(),'human_at_closest':hxy[ci].tolist(),
        'passed_sim_s':float(t[passed_h[0]]) if len(passed_h) else None,'contacts':actor_contacts(h['prim']),**visible['C30_human']}
    wheel_result={'position_world_xy_m':wxy[0].tolist(),'maximum_displacement_m':float(np.linalg.norm(wxy-wxy[0],axis=1).max()),
        'maximum_transform_error':max(float(r['matrix_max_error']) for r in wheel),
        'closest_robot_center_distance_m':float(np.linalg.norm(xy-wxy,axis=1).min()),
        'passed_sim_s':float(t[passed_w[0]]) if len(passed_w) else None,'contacts':actor_contacts(w['prim']),**visible['C10_wheelchair']}
    encounter_result={'observed':bool(len(encounter)), 'first_sim_s':float(t[encounter[0]]) if len(encounter) else None,
        'criterion':c,'planned_sim_s':h['planned_encounter_sim_s'],'planned_world_xyz':h['planned_encounter_world_xyz']}
    passed=bool(len(encounter) and len(passed_h) and len(passed_w) and goal_time is not None and not through
        and human_result['contacts']['through_goal_or_end']==0 and wheel_result['contacts']['through_goal_or_end']==0)
    result={'status':'PASS_ENCOUNTER_AND_GOAL_ENTRY' if passed else 'FAIL_OR_UNQUALIFIED_ENCOUNTER',
        'run_id':source.name,'predictions':len(events),'robot_ticks':len(robot),'last_sim_time_s':float(t[-1]),
        'human':human_result,'wheelchair':wheel_result,'corner_encounter':encounter_result,
        'first_goal_entry_sim_s':goal_time,'closest_goal_distance_m':float(gd.min()),'final_goal_distance_m':float(gd[-1]),
        'nonfloor_contact_spans':len(spans),'nonfloor_spans_through_goal_or_end':len(through),
        'first_nonfloor_contact_sim_s':min((s['start_sim_time'] for s in spans),default=None),
        'maximum_applied_v_m_s':max(float(r['applied_v']) for r in robot),
        'limitations':['One scripted kinematic human in authored rest pose, no gait or crowd avoidance policy.',
            'PASS concerns contact-free encounter and goal-region entry, not stopping or general avoidance qualification.',
            'Centre distances are not mesh clearance. Contacts include persistent/proximity callbacks.',
            'Original Hospital world origin, metres,+Z up. PhysX human translation/quaternion xyzw; static USD wheelchair matrix.',
            'Observation, inference, reasoning and application times remain separate; simulation time is not wall time.']}
    output=source/'derived/c30-oncoming-c10-wheelchair';output.mkdir(parents=True,exist_ok=False)
    write_json(output/'summary.json',result);write_json(output/'contacts.json',{'spans':spans})
    plot(source,comparison,output,events,cfg,result,t,xy,hxy,wa,ci)
    for p,h in hashes.items():
        if digest(source/p)!=h:raise ValueError('Raw evidence changed')
    print(json.dumps(result,indent=2));return result


def plot(source,comparison,out,events,cfg,result,t,xy,hxy,wa,closest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Polygon
    from research.cart_geometry import rotation
    from PIL import Image,ImageDraw,ImageFont
    old=np.array([[float(r[k]) for k in ('x','y')] for r in rows(comparison/'robot_state.csv')])
    fig,ax=plt.subplots(figsize=(13,7));ax.plot(*old.T,':',color='#b7bec5',label='Original no-added-obstacle path')
    ge=result['first_goal_entry_sim_s'];end=int(np.flatnonzero(t>=ge)[0]) if ge is not None else len(t)-1
    ax.plot(*xy[:end+1].T,color='#167594',lw=2.5,label='Robot before goal entry / end')
    if ge is not None:ax.plot(*xy[end:].T,'--',color='#8994a4',label='Robot after goal entry')
    ax.plot(*hxy.T,color='#d77b18',lw=2,label='Measured oncoming human path')
    ax.scatter(*hxy[0],marker='s',color='#d77b18');ax.annotate('Human starts at C30',hxy[0],xytext=(0,-24),textcoords='offset points',ha='center')
    j=min(len(hxy)-1,int(np.argmax(np.linalg.norm(hxy-hxy[0],axis=1)>3.)))
    ax.annotate('',xy=hxy[min(j+60,len(hxy)-1)],xytext=hxy[j],arrowprops={'arrowstyle':'->','color':'#d77b18','lw':2})
    half=np.array(wa['footprint_half_m']);q=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*half
    q=q@rotation(wa['yaw_rad']).T+wa['center_world_xyz'][:2];ax.add_patch(Polygon(q,color='#75409a',alpha=.6,label='Fixed C10 wheelchair'))
    ax.plot([xy[closest,0],hxy[closest,0]],[xy[closest,1],hxy[closest,1]],'o-',color='#bb4444',label=f'Closest encounter {t[closest]:.2f} s')
    goal=cfg['scene']['goal'][:2];ax.add_patch(Circle(goal,2,color='#e2c469',alpha=.2));ax.scatter(*goal,marker='*',s=150,c='#b38a00',label='Goal / 2 m radius')
    ax.scatter(*xy[0],marker='s',c='#26723d',s=65,label='Robot start');ax.scatter(*xy[-1],marker='X',c='#b23a3a',s=65,label='Robot final')
    ax.set(xlabel='Hospital world X [m]',ylabel='Hospital world Y [m]',title='C30 oncoming human + C10 wheelchair | robot max_v 0.8 m/s')
    ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=9,ncol=2)
    fig.text(.5,.02,'Original Hospital origin,+Z up, metres; walls omitted. Human translation is measured PhysX, no gait animation.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.05,1,1]);fig.savefig(out/'trajectory.png',dpi=160);plt.close(fig)
    event_i=min(range(len(events)),key=lambda i:abs(events[i]['observation']['sim_time']-t[closest]));start=max(0,min(event_i-3,len(events)-6))
    fig,axes=plt.subplots(2,3,figsize=(15,7))
    for ax,e in zip(axes.flat,events[start:start+6]):
        with Image.open(source/e['rgb_observation_reference']) as im:ax.imshow(im)
        ax.set_title(f'C{e["request_id"]} | {e["observation"]["sim_time"]:.3f} sim s');ax.axis('off')
    fig.suptitle('RGB around closest measured robot-human approach');fig.tight_layout();fig.savefig(out/'encounter-rgb.png',dpi=140);plt.close(fig)
    frames=[];font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for e in events:
        with Image.open(source/e['rgb_observation_reference']) as im:rgb=im.convert('RGB');rgb.thumbnail((960,540),Image.Resampling.LANCZOS)
        frame=Image.new('RGB',(960,572),'#14212c');frame.paste(rgb,(0,32));ImageDraw.Draw(frame).text((12,7),f'C30 oncoming + C10 wheelchair | C{e["request_id"]:02d} | sim {e["observation"]["sim_time"]:.3f} s',font=font,fill='white');frames.append(frame.quantize(colors=192))
    times=np.array([e['observation']['sim_time'] for e in events]);duration=np.r_[np.diff(times),max(.02,t[-1]-times[-1])]
    frames[0].save(out/'rgb-observations.gif',save_all=True,append_images=frames[1:],duration=[max(20,round(1000*d)) for d in duration],loop=0,disposal=2,optimize=False)
