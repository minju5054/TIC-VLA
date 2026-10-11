"""Saved-only late-application diagnostics using unchanged Hospital gates."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.late_handoff import load_late, eligible
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_no_cart_speed import analyze_run,RULES,INVENTORY
from research.hospital_late_handoff import RULES as HOLD_RULES
from research.hospital_spatial_gate import wall_inventory,bound_clearance
from research.route_geometry import temporal_overlap,boundary_seam,rotation
from research.analyze_handoffs import metrics as timing
from research.analyze_e16_handoff_audit import editable_overlap,overlap_metrics,serial
from research.records import write_json,provenance
from research.compare_e16_cart_shift import write_csv
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination


def outcome(complete,trigger,contact,passage,left,runtime=True):
    if not runtime:return 'F'
    if contact is not None and (passage is None or contact<=passage):return 'D'
    if not complete or not trigger:return 'E'
    if passage is not None and left:return 'B' if contact is not None else 'A'
    return 'C'


def analyze(path,historical=False):
    run=load_saved(path) if historical else load_late(path)
    rules=json.loads(HOLD_RULES.read_text());cfg=run['cfg'];g=cfg['spatial_turn_gate']
    inventory=json.loads(INVENTORY.read_text());walls=wall_inventory(inventory)
    base=analyze_run(run,json.loads(RULES.read_text()),inventory);nav=base['summary']
    if historical:
        entry=nav['corner_entry']['sim_time']
        e=next((e for e in run['events'][1:] if eligible(e['observation'],cfg,rules['trigger_band_south_m'],entry<=e['observation']['sim_time'])),None)
        hold=None;delay=0.;trial=None
    else:
        hold=run['hold'];e=run['events'][hold['request_id']-1] if hold and hold['applied'] else None
        trial=run['metadata']['freeze_receipt']['trial'];delay=trial['delay_s']
    first=nav['first_nonfloor_contact'];ct=first['sim_time'] if first else None;pt=nav['corner_passage_completion_sim_time']
    end=min(t for t in [ct,pt,float(run['times'][-1])] if t is not None)
    begin=np.flatnonzero(run['xy'][:,1]>=g['center_y_min_m']-rules['trigger_band_south_m'])
    mask=(run['times']>=run['times'][begin[0]])&(run['times']<=end) if len(begin) else np.zeros(len(run['times']),bool)
    indices=np.flatnonzero(mask);points=run['xy'][mask]
    clear=[bound_clearance(p,walls,g['robot_conservative_radius_m']) for p in points]
    speed=base['speed'];yaw=base['measured']['body_yaw_rate_measured'];valid=mask&(speed>1e-6)&(np.abs(yaw)>1e-6)
    curvature=yaw[valid]/speed[valid];radii=1/np.abs(curvature)
    result=outcome(run['complete'],e is not None or hold is not None,ct,pt,nav['left_turn_begins'])
    m=dict(run=str(path),historical=historical,trial=trial['trial'] if trial else None,requested_delay_s=delay,
        realized_delay_s=hold['realized_delay_sim_s'] if hold else 0.,realized_ticks=hold['realized_ticks'] if hold else 0,
        runtime_status='PASS',outcome=result,request_count=len(run['events']),triggered=e is not None or hold is not None,
        selected_fresh_id=e['request_id'] if e else hold['request_id'] if hold else None,
        selected_old_id=e['request_id']-1 if e else hold['old_request_id'] if hold else None,
        fresh_applied=bool(e),hold_translation_m=hold['hold_translation_xy_m'] if hold else 0.,
        hold_yaw_change_deg=hold['hold_yaw_change_deg'] if hold else 0.,
        corner_window_start_sim_time=float(run['times'][indices[0]]) if len(indices) else None,corner_window_end_sim_time=end,
        max_corner_y_m=float(points[:,1].max()) if len(points) else None,
        northward_overshoot_m=max(0.,float(points[:,1].max()-g['center_y_max_m'])) if len(points) else None,
        signed_north_gate_excess_m=float(points[:,1].max()-g['center_y_max_m']) if len(points) else None,
        corner_travel_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()) if len(points) else None,
        min_structure_proxy_clearance_m=min((c['wall_bound_clearance_m'] for c in clear),default=None),
        max_abs_measured_curvature_per_m=float(np.abs(curvature).max()) if len(curvature) else None,
        min_measured_turn_radius_m=float(radii.min()) if len(radii) else None,
        corner_passage_sim_time=pt,left_turn_begins=nav['left_turn_begins'],left_turn_completed=nav['left_turn_completed'],
        first_left_interval=nav['first_actual_left_interval'],west_hallway_passed=bool(pt is not None),
        first_contact=first,contact_sim_time=ct,goal_region_reached=nav['goal_region_reached'],
        goal_entry_sim_time=nav['first_goal_region_entry']['sim_time'] if nav['first_goal_region_entry'] else None,
        baseline_gate_pass=bool(result=='A' and nav['left_turn_completed'] and nav['goal_region_reached'] and (e is not None or hold is not None)))
    first_left=nav['first_actual_left_interval']
    left_row=next((r for r in run['robot'] if first_left and float(r['sim_time'])==first_left['start_sim_time']),None)
    m.update(left_start_sim_time=first_left['start_sim_time'] if first_left else None,
        left_start_x_m=float(left_row['x']) if left_row else None,left_start_y_m=float(left_row['y']) if left_row else None,
        left_start_yaw_rad=float(left_row['yaw']) if left_row else None)
    details={};selected={}
    if e is not None or hold is not None:
        if e is None:
            received=json.loads((path/'raw/late_handoff/received.json').read_text())
            e={**received['request'],**received['response'],**received['old'],**received,
                'observation':received['request']['observation'],'agent_pose_at_observation':received['agent_pose_at_observation']}
            c=np.load(path/'raw/late_handoff/received.npy',allow_pickle=False)
            from research.analyze_chunk_geometry import project_world
            fw=project_world(c,e['agent_pose_at_observation']);switch=None
        else:c=run['chunks'][e['request_id']-1];fw=run['worlds'][e['request_id']-1];switch=e['application']
        oi=e['request_id']-2;old=run['events'][oi];ow=run['worlds'][oi];ot=old['observation']['sim_time'];ft=e['observation']['sim_time']
        full=temporal_overlap(ow,fw,ot,ft,.1);over=overlap_metrics(full,e['agent_pose_at_observation'][2],1e-6)
        m.update(full_overlap_rmse_m=over['rmse_m'],full_mean_abs_lateral_m=over['mean_abs_lateral_m'],full_max_abs_lateral_m=over['max_abs_lateral_m'],
            delta_v=e['controller_command'][0]-old['controller_command'][0],delta_w=e['controller_command'][1]-old['controller_command'][1],
            fresh_v=e['controller_command'][0],fresh_w=e['controller_command'][1],old_v=old['controller_command'][0],old_w=old['controller_command'][1],
            fresh_lookahead_forward_m=float(c[e['lookahead_index'],0]),fresh_lookahead_left_m=float(c[e['lookahead_index'],1]),
            original_left_command=bool(e['controller_command'][1]>0 and c[e['lookahead_index'],1]>0),
            actual_inference_wall_s=(e['ready']['monotonic_ns']-e['inference_start']['monotonic_ns'])/1e9,
            genuine_pending_sim_s=e['response_detected']['sim_time']-ft)
        if switch:
            st=switch['sim_time'];seam=boundary_seam(fw,e['observation'],switch,run['times'],run['xy'])
            edit=editable_overlap(full,ow,fw,ot,ft,st,.1)
            m.update(boundary_position_m=seam['raw_boundary_position_gap_m'],boundary_heading_deg=seam['raw_robot_heading_to_fresh_tangent_gap_deg'],
                boundary_executed_heading_deg=seam['raw_executed_to_fresh_tangent_gap_deg'],
                remaining_fresh_horizon_s=max(0.,ft+3-st),editable_overlap_rmse_m=overlap_metrics(edit,e['agent_pose_at_observation'][2],1e-6)['rmse_m'],
                observation_to_switch_transport_m=timing(e)['robot_translation_xy_m'],switch_sim_time=st,
                switch_proxy_clearance_m=bound_clearance(switch['pose'][:2],walls,g['robot_conservative_radius_m'])['wall_bound_clearance_m'])
            from research.nova_dynamic_geometry import remaining_curve
            remaining_times,remaining_points=remaining_curve(fw,ft,st,.1)
            predicted_clearances=[bound_clearance(p,walls,g['robot_conservative_radius_m']) for p in remaining_points]
            m['fresh_remaining_min_structure_proxy_clearance_m']=min((v['wall_bound_clearance_m'] for v in predicted_clearances),default=None)
            details['fresh_remaining_structure_proxy']=dict(absolute_nominal_times=remaining_times,world_points=remaining_points,clearances=predicted_clearances,
                note='Pointwise circumscribed disk / AABB diagnostic; no swept collision or dynamic recoverability proof')
            details['seam']=seam;details['editable']=edit
        else:
            m.update({k:None for k in ['boundary_position_m','boundary_heading_deg','boundary_executed_heading_deg','remaining_fresh_horizon_s',
                'editable_overlap_rmse_m','observation_to_switch_transport_m','switch_sim_time','switch_proxy_clearance_m']})
        details.update(old_world=ow,fresh_world=fw,full=full)
        selected=dict(old_request_id=old['request_id'],fresh_request_id=e['request_id'],observation=e['observation'],
            inference_start=e['inference_start'],ready=e['ready'],response_detected=e['response_detected'],application=switch,
            hold=hold,old_command=old['controller_command'],fresh_original_command=e['controller_command'],
            native_lookahead=c[e['lookahead_index']].tolist(),original_observation_anchor=e['agent_pose_at_observation'])
    m['recoverability']='UNRESOLVED: no collision-free dynamic reachability or corrected path tested; AABB clearance and native geometry are proxies only'
    samples=[dict(sim_time=float(run['times'][i]),x=float(run['xy'][i,0]),y=float(run['xy'][i,1]),
        command_w=float(run['robot'][i]['target_w']),yaw_rate_rad_s=float(yaw[i]),xy_speed_m_s=float(speed[i]),
        curvature_per_m=float(yaw[i]/speed[i]) if speed[i]>1e-6 and abs(yaw[i])>1e-6 else None,
        radius_m=float(speed[i]/abs(yaw[i])) if speed[i]>1e-6 and abs(yaw[i])>1e-6 else None,
        clearance_proxy_m=clear[j]['wall_bound_clearance_m']) for j,i in enumerate(indices)]
    return run,dict(metrics=m,navigation=nav,selected=selected,geometry=serial(details),corner_samples=samples)


def figures(out,runs,results,inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    colors=plt.cm.tab10(np.arange(len(runs))%10);g=next(iter(runs.values()))['cfg']['spatial_turn_gate']
    def context(ax):
        for w in wall_inventory(inventory):
            lo=np.array(w['world_aabb_min'][:2]);hi=np.array(w['world_aabb_max'][:2]);ax.add_patch(Rectangle(lo,*(hi-lo),fc='#ededed',ec='#cccccc',lw=.4,zorder=0))
        ax.plot([g['exit_center_x_max_m']]*2,[g['center_y_min_m'],g['center_y_max_m']],c='green',lw=2,label='Original exit gate')
        for y in [g['center_y_min_m'],g['center_y_max_m'],g['center_y_min_m']-.75]:ax.axhline(y,color='gray',lw=.7,ls=':')
        ax.set(xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.15)
    def save(fig,name):
        fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=170,bbox_inches='tight');plt.close(fig)
    for zoom in [False,True]:
        fig,ax=plt.subplots(figsize=(14,8));context(ax)
        for color,(name,run) in zip(colors,runs.items()):
            d=results[name];ax.plot(*run['xy'].T,color=color,lw=1.5,label=name)
            s=d['selected']
            if s:
                for k,mark in [('observation','o'),('response_detected','s'),('application','X')]:
                    if s[k]:ax.scatter(*s[k]['pose'][:2],facecolors='none' if k=='response_detected' else [color],edgecolors=[color] if k!='application' else 'black',linewidths=1,marker=mark,s=85 if k=='response_detected' else 45,zorder=6)
                if zoom:
                    for k,ls in [('old_world','--'),('fresh_world',':')]:ax.plot(*np.asarray(d['geometry'][k]).T,c=color,ls=ls,alpha=.6,lw=.9)
            contact=d['metrics']['first_contact']
            if contact:ax.scatter(*np.asarray(contact['reported_points'])[:,:2].T,c=[color],marker='+',s=100,zorder=8)
        if zoom:ax.set(xlim=[-1,8.3],ylim=[7.6,13.4])
        else:
            pts=np.vstack([r['xy'] for r in runs.values()]);ax.set(xlim=[pts[:,0].min()-1,pts[:,0].max()+1],ylim=[pts[:,1].min()-1,pts[:,1].max()+1])
        ax.legend(fontsize=8,loc='best');ax.set_title('Hospital E16 measured paths | o observation, square detection, X application\nCorner only: dashed OLD, dotted FRESH predictions; no correction')
        save(fig,'corner_comparison' if zoom else 'world_xy')
    def delay_axis(ax):
        ax.set_xlim(-.05,1.05);ax.set_xticks([0,.25,.5,.75,1])
        tested={d['metrics']['requested_delay_s'] for d in results.values() if not d['metrics']['historical']}
        if tested=={0.}:
            ax.axvspan(.125,1.05,color='#eeeeee',zorder=0)
            ax.text(.62,.5,'0.25–1.00 s NOT RUN\nBaseline repeatability gate failed',transform=ax.transAxes,ha='center',va='center',fontsize=10,color='#666666')

    panels=[('overshoot',['northward_overshoot_m'],['Northward overshoot [m]']),
        ('passage',['corner_passage_sim_time'],['Passage simulation time [s]']),
        ('seam_transport',['boundary_position_m','boundary_heading_deg','hold_translation_m'],['Boundary position seam [m]','Robot / derived tangent seam [deg]','Additional OLD transport [m]'])]
    for stem,keys,labels in panels:
        fig,axes=plt.subplots(len(keys),1,figsize=(10,3.3*len(keys)),squeeze=False)
        for ax,k,label in zip(axes[:,0],keys,labels):
            for color,(name,d) in zip(colors,results.items()):
                m=d['metrics'];v=m.get(k)
                if v is not None:ax.scatter(m['requested_delay_s'],v,color=color,marker='x' if m['historical'] else 'o',s=65,label=name)
            ax.set(xlabel='Requested additional hold [s]',ylabel=label,xlim=[-.05,1.05]);ax.grid(alpha=.25);delay_axis(ax)
        axes[0,0].legend(fontsize=8,ncol=3);fig.suptitle('Unrun conditions are absent; undefined passage/seams are not zeros');save(fig,'delay_'+stem)
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for color,(name,d) in zip(colors,results.items()):
        rows=d['corner_samples']
        if rows:
            s=d['selected'];t0=s['observation']['sim_time'] if s else rows[0]['sim_time'];t=np.array([r['sim_time']-t0 for r in rows])
            for ax,key in zip(axes,['command_w','yaw_rate_rad_s']):ax.plot(t,[r[key] for r in rows],c=color,label=name)
    axes[0].set_ylabel('Applied command w [rad/s]');axes[1].set(ylabel='Measured yaw rate [rad/s]',xlabel='Simulation seconds after spatial trigger observation')
    axes[0].legend(fontsize=8,ncol=3);save(fig,'angular_command_yaw_rate')
    fig,axes=plt.subplots(int(np.ceil(len(runs)/2)),2,figsize=(13,4.5*np.ceil(len(runs)/2)),squeeze=False)
    for ax,(name,d) in zip(axes.flat,results.items()):
        context(ax);s=d['selected']
        if s:
            for k,ls,label in [('old_world','--','OLD native -> world'),('fresh_world',':','FRESH native -> world')]:ax.plot(*np.asarray(d['geometry'][k]).T,ls=ls,label=label)
            r=runs[name];t0=s['observation']['sim_time'];t1=(s['application'] or s['response_detected'])['sim_time']+1
            mask=(r['times']>=t0-.5)&(r['times']<=t1);ax.plot(*r['xy'][mask].T,c='black',label='Measured execution')
            for key,mark in [('observation','o'),('response_detected','s'),('application','X')]:
                if s[key]:ax.scatter(*s[key]['pose'][:2],marker=mark,s=60,label=key)
            contact=d['metrics']['first_contact']
            if contact:ax.scatter(*np.asarray(contact['reported_points'])[:,:2].T,marker='+',s=100,c='red')
        ax.set(xlim=[1,8],ylim=[7.8,13]);ax.set_title(name,fontsize=9);ax.legend(fontsize=6)
    for ax in list(axes.flat)[len(runs):]:ax.set_visible(False)
    save(fig,'selected_old_fresh_world')
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for color,(name,d) in zip(colors,results.items()):
        m=d['metrics'];axes[0].scatter(m['requested_delay_s'],int(m['contact_sim_time'] is not None),c=[color],label=name,s=65)
        axes[1].scatter(m['requested_delay_s'],int(m['goal_region_reached']),c=[color],s=65)
    axes[0].set(ylabel='Actual nonfloor contact',yticks=[0,1]);axes[1].set(xlabel='Requested additional hold [s]',ylabel='Goal-region entry',yticks=[0,1],xlim=[-.05,1.05]);axes[0].legend(fontsize=8,ncol=3)
    for ax,key in zip(axes,['contact_sim_time','goal_region_reached']):
        delay_axis(ax)
        for value in [0,1]:
            names=[name for name,d in results.items() if d['metrics']['requested_delay_s']==0 and
                int(d['metrics'][key] is not None if key=='contact_sim_time' else d['metrics'][key])==value]
            if names:ax.annotate(' / '.join(names),(0,value),xytext=(12,-16 if value else 12),textcoords='offset points',fontsize=8)
    fig.suptitle('Observed contact / goal outcomes; markers at identical values overlap')
    save(fig,'delay_contact_goal')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path);p.add_argument('--plan',type=Path);p.add_argument('--output-dir',type=Path);a=p.parse_args()
    if a.run:
        path=a.run.resolve();hashes=evidence_hashes([path]);run,d=analyze(path)
        verify_hashes(hashes);json.dumps(serial(d),allow_nan=False);write_json(path/'derived/late_metrics_v2.json',serial(d));print(json.dumps(d['metrics'],indent=2));return
    receipt=json.loads((a.plan/'receipt.json').read_text());paths=[Path(p) for p in receipt['reference_runs']]
    paths += [ROOT/'outputs'/t['run_id'] for t in receipt['trials'] if (ROOT/'outputs'/t['run_id']/'derived/late_metrics_v2.json').exists()]
    out=validate_destination(a.output_dir,paths);hashes=evidence_hashes(paths);runs={};results={}
    for i,path in enumerate(paths):
        run,d=analyze(path,historical=i<2);m=d['metrics'];label=['Historical A','Historical B'][i] if i<2 else f"{m['requested_delay_s']:.2f}s trial {m['trial']}"
        runs[label]=run;results[label]=d
    out.mkdir(parents=True);(out/'figures').mkdir();(out/'derived').mkdir()
    for i,(name,d) in enumerate(results.items()):
        write_json(out/'derived'/f'run_{i:02d}.json',serial(d));write_csv(out/'derived'/f'run_{i:02d}_corner.csv',d['corner_samples'])
    rows=[{k:v for k,v in d['metrics'].items() if not isinstance(v,(dict,list))} for d in results.values()]
    # Union accommodates a trial stopped before application / missing trigger.
    keys=list(dict.fromkeys(k for r in rows for k in r));rows=[{k:r.get(k) for k in keys} for r in rows]
    write_csv(out/'trial_metrics.csv',rows);write_json(out/'summary.json',{k:serial(d['metrics']) for k,d in results.items()})
    figures(out/'figures',runs,results,json.loads(INVENTORY.read_text()));verify_hashes(hashes)
    write_json(out/'metadata.json',dict(source_sha256=hashes,source_files_unchanged=True,analysis=provenance(ROOT),
        plan_sha256=__import__('hashlib').sha256((a.plan/'receipt.json').read_bytes()).hexdigest(),
        conventions='Native forward/left metres -> world XY using each original observation yaw, CCW +X, Z up, quaternion wxyz. Nominal +0.1..3s from source convention, not native timestamps. Tangents are derived, not native yaw.'))
    print(str(out))


if __name__=='__main__':main()
