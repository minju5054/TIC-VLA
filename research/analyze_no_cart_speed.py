"""Saved-only E16 no-cart speed comparison; original corner/turn gates retained."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_hospital_turn import characterize
from research.analyze_handoffs import metrics as latency,stats
from research.hospital_no_cart_speed import validate_speed,DIFF
from research.hospital_spatial_gate import spatial_gate,wall_inventory
from research.control import waypoint_command
from research.cart_bypass import plot_coordinates
from research.e16_c26_tools import precontact
from research.route_geometry import temporal_overlap,boundary_seam,rotation
from research.compare_e16_cart_shift import write_csv
from research.analyze_e16_c26_speed import contact_info,INVENTORY
from research.records import write_json,provenance
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination

RULES=ROOT/'configs/research/e16_no_cart_speed10_analysis.json'


def in_corner(xy,cfg):
    p=np.asarray(xy);g=cfg['spatial_turn_gate']
    return ((p[...,0]>=g['exit_center_x_max_m']) &
            (p[...,0]<=cfg['robot']['start_position'][0]+g['robot_conservative_radius_m']) &
            (p[...,1]>=g['center_y_min_m']) & (p[...,1]<=g['center_y_max_m']))


def classify(complete,entry,passage_time,first_contact,left_valid):
    if not complete:return 'EXECUTION FAILED'
    if first_contact is not None and (passage_time is None or first_contact<=passage_time):return 'STRUCTURAL CONTACT BEFORE CORNER PASSAGE'
    if passage_time is not None and first_contact is None and left_valid:return 'CORNER PASSED WITHOUT CONTACT'
    if entry is None and first_contact is None:return 'INCOMPLETE — CORNER NOT REACHED'
    return 'OTHER NAVIGATION FAILURE'


def event_at_mask(run,mask):
    ids=np.flatnonzero(mask)
    if not len(ids):return None
    i=int(ids[0]);r=run['robot'][i]
    return dict(tick=i,sim_time=float(r['sim_time']),pose=[float(r[k]) for k in ['x','y','yaw']])


def analyze_run(run,rules,inventory):
    cfg=run['cfg'];g=cfg['spatial_turn_gate'];events=run['events'];first=min((c['start_sim_time'] for c in run['contacts']),default=None)
    diags=[json.loads((run['path']/'raw/scene_observations'/(Path(e['rgb_observation_reference']).name+'.json')).read_text()) for e in events]
    raw_contacts=[json.loads(x) for x in (run['path']/'raw/robot_contacts.jsonl').read_text().splitlines()]
    geom=dict(run,robot=[dict(r,command_v=r['target_v'],command_w=r['target_w']) for r in run['robot']])
    _,_,intervals,turn,_=characterize(geom,diags,raw_contacts)
    gate=spatial_gate(events,g);entry=event_at_mask(run,in_corner(run['xy'],cfg))
    passage_time=events[gate['through_request_id']-1]['application']['sim_time'] if gate['crossing_plus_one_observation'] else None
    groups=turn['actual_left_interval_groups_by_fresh_id'];left_valid=bool(groups)
    trim=[w for w in inventory if '/Geo_M2_TrimDoor5/' in w['path']]
    if len(trim)!=1:raise ValueError('Unique original TrimDoor5 bound required')
    trim_plane=trim[0]['world_aabb_min'][0]-g['robot_conservative_radius_m']
    corridor=(run['xy'][:,1]>=g['center_y_min_m'])&(run['xy'][:,1]<=g['center_y_max_m'])
    trim_pass=event_at_mask(run,(run['xy'][:,0]<=trim_plane)&corridor&(run['times']>=(entry['sim_time'] if entry else np.inf)))
    goal=np.linalg.norm(run['xy']-cfg['scene']['goal'][:2],axis=1);goal_event=event_at_mask(run,goal<rules['goal_region_radius_m'])
    observations=[];pairs=[];details={};yf=None
    for i,(e,chunk) in enumerate(zip(events,run['chunks'])):
        command,yf,index=waypoint_command(chunk,yf,**cfg['controller'])
        np.testing.assert_allclose(command,e['controller_command'],rtol=0,atol=1e-12)
        np.testing.assert_allclose(yf,e['yaw_filter_state'],rtol=0,atol=1e-12)
        if index!=e['lookahead_index']:raise ValueError('Lookahead mismatch')
        if 'stationary_cart' not in cfg and e['human_at_observation'] is not None:raise ValueError('Unexpected actor snapshot')
        pose=e['agent_pose_at_observation'];tick=run['robot'][e['observation']['tick']]
        if pose!=[float(tick[k]) for k in ['x','y','yaw']]:raise ValueError('Observation pose/tick mismatch')
        seam=boundary_seam(run['worlds'][i],e['observation'],e['application'],run['times'],run['xy'])
        observations.append(dict(sim_time=e['observation']['sim_time'],application_sim_time=e['application']['sim_time'],
            x_world_m=pose[0],y_world_m=pose[1],yaw_rad=pose[2],lookahead_index=index,
            native_lookahead_forward_m=float(chunk[index,0]),native_lookahead_left_m=float(chunk[index,1]),
            controller_v=command[0],controller_w=command[1],measured_forward_m_s=float(tick['body_v_forward']),
            measured_yaw_rate_rad_s=float(tick['body_yaw_rate_measured']),**latency(e),
            **{k:seam[k] for k in ['raw_boundary_position_gap_m','raw_executed_to_fresh_tangent_gap_deg','raw_robot_heading_to_fresh_tangent_gap_deg']}))
        if not i:continue
        old=events[i-1];a=temporal_overlap(run['worlds'][i-1],run['worlds'][i],old['observation']['sim_time'],e['observation']['sim_time'],rules['native_waypoint_dt_s'])
        local=a['delta']@rotation(pose[2]) if a else None
        row=dict(old_request_id=i,fresh_request_id=i+1,fresh_observation_sim_time=e['observation']['sim_time'],
            application_sim_time=e['application']['sim_time'],x_world_m=pose[0],y_world_m=pose[1],
            aligned_overlap_rmse_m=a['rmse'] if a else None,
            mean_abs_lateral_revision_m=float(np.abs(local[:,1]).mean()) if a else None,
            max_abs_lateral_revision_m=float(np.abs(local[:,1]).max()) if a else None,
            mean_signed_lateral_revision_m=float(local[:,1].mean()) if a else None,
            corner_region=bool(in_corner(pose[:2],cfg)),delta_v=command[0]-old['controller_command'][0],delta_w=command[1]-old['controller_command'][1],
            **{k:observations[-1][k] for k in ['raw_boundary_position_gap_m','raw_executed_to_fresh_tangent_gap_deg','raw_robot_heading_to_fresh_tangent_gap_deg']})
        row['precontact']=precontact(row,first);pairs.append(row)
        details[str(i)]=dict(old_full_world=run['worlds'][i-1].tolist(),fresh_full_world=run['worlds'][i].tolist(),
            old_observation_pose=old['agent_pose_at_observation'],fresh_observation_pose=pose,
            alignment={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in a.items()} if a else None)
    pre=[r for r in pairs if r['precontact']];post=[r for r in pairs if not r['precontact']]
    selected=next((r for r in pre if r['corner_region']),pre[-1] if pre else None)
    measured={k:np.array([float(r[k]) for r in run['robot']]) for k in ['body_v_forward','body_yaw_rate_measured','vx_world','vy_world','yaw']}
    speed=np.hypot(measured['vx_world'],measured['vy_world']);before=run['times']<(first if first is not None else np.inf)
    contact=contact_info(run,run['contacts'])
    if contact:
        tick=int(np.argmin(np.abs(run['times']-contact['sim_time'])))
        if abs(run['times'][tick]-contact['sim_time'])>1e-8:raise ValueError('Contact tick not measured')
        contact.update(robot_tick=tick,robot_yaw_rad=float(measured['yaw'][tick]),
                       robot_pose=[*run['xy'][tick].tolist(),float(measured['yaw'][tick])])
    def pair_stats(rows):
        return dict(count=len(rows),**{k:stats([r[k] for r in rows if r[k] is not None]) for k in ['aligned_overlap_rmse_m','mean_abs_lateral_revision_m','max_abs_lateral_revision_m','raw_boundary_position_gap_m']}) if rows else dict(count=0)
    summary=dict(run=str(run['path']),complete=run['complete'],request_count=len(events),measured_ticks=len(run['robot']),
        classification=classify(run['complete'],entry,passage_time,first,left_valid),corner_entry=entry,spatial_gate=gate,
        corner_passage_completion_sim_time=passage_time,actual_left_motion_groups=groups,left_turn_begins=left_valid,
        first_actual_left_interval=next((r for r in intervals if r['qualifying_left_motion']),None),
        left_turn_completed=bool(passage_time is not None and left_valid and (first is None or first>passage_time)),
        trimdoor5_contact=any('/Root/Geo_M2_TrimDoor5/' in json.dumps(c) for c in run['contacts']),
        trimdoor5_west_disk_clear_plane_x_m=trim_plane,trimdoor5_region_passage=trim_pass,
        continues_hallway=bool(trim_pass and passage_time is not None),
        first_nonfloor_contact=contact,contact_span_count=len(run['contacts']),
        final_pose=[*run['xy'][-1].tolist(),float(measured['yaw'][-1])],final_yaw_deg=float(np.degrees(measured['yaw'][-1])),
        travel_distance_m=float(np.linalg.norm(np.diff(run['xy'],axis=0),axis=1).sum()),
        goal_region_reached=bool(goal_event),first_goal_region_entry=goal_event,minimum_goal_distance_m=float(goal.min()),final_goal_distance_m=float(goal[-1]),
        official_navigation_success='NOT EVALUATED by this research harness',
        precontact_speed_m_s=stats(speed[before]),precontact_body_forward_m_s=stats(measured['body_v_forward'][before]),
        precontact_abs_body_yaw_rate_rad_s=stats(np.abs(measured['body_yaw_rate_measured'][before])),
        all_speed_m_s=stats(speed),precontact_pairs=pair_stats(pre),postcontact_pairs_excluded=pair_stats(post),
        selected_pair=selected,selected_pair_outside_corner=bool(selected and not selected['corner_region']),
        latency_nonbootstrap={k:stats([r[k] for r in observations[1:]]) for k in ['action_wall_s','observation_to_switch_wall_s','pending_sim_s','robot_translation_xy_m','robot_yaw_change_deg']},
        bootstrap=observations[0],controller_commands_recomputed=len(events))
    return dict(summary=summary,observations=observations,pairs=pairs,details=details,measured=measured,speed=speed,turn_intervals=intervals)


def figures(out,runs,results,inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'font.size':10,'pdf.fonttype':42})
    colors={'reference':'#2479ad','speed10':'#de5c24','cart12':'#999999'}
    labels={'reference':'No-cart max_v=1.5','speed10':'No-cart max_v=1.0','cart12':'WITH-cart max_v=1.2 (context only)'}
    g=runs['reference']['cfg']['spatial_turn_gate'];goal=runs['reference']['cfg']['scene']['goal']
    walls=wall_inventory(inventory);trim=next(w for w in walls if '/Geo_M2_TrimDoor5/' in w['path'])
    allxy=np.vstack([*[r['xy'] for r in runs.values()],np.array(goal[:2])[None,:]])
    lo,hi=allxy.min(0)-1,allxy.max(0)+1
    full=[lo[0],hi[0],lo[1],hi[1]];zoom=[-1.5,8.5,7.5,13.5]
    def context(ax,bounds):
        for w in walls:
            a,b=np.array(w['world_aabb_min']),np.array(w['world_aabb_max']);ax.add_patch(Rectangle(a[:2],*(b-a)[:2],fc='#ededed',ec='#bfc5cb',lw=.4,zorder=0))
        a,b=np.array(trim['world_aabb_min']),np.array(trim['world_aabb_max']);ax.add_patch(Rectangle(a[:2],*(b-a)[:2],fc='#dc4677',ec='#dc4677',lw=2,label='Original TrimDoor5',zorder=2))
        ax.plot([g['exit_center_x_max_m']]*2,[g['center_y_min_m'],g['center_y_max_m']],c='#309769',lw=2,label='Original corner exit gate')
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.15)
    def save(fig,stem):
        fig.tight_layout();fig.savefig(out/(stem+'.png'),dpi=160,bbox_inches='tight');fig.savefig(out/(stem+'.pdf'),bbox_inches='tight');plt.close(fig)
    for stem,bounds in [('world_xy',full),('corner_trimdoor5_zoom',zoom)]:
        fig,ax=plt.subplots(figsize=(14,7));context(ax,bounds)
        for name,r in runs.items():
            ax.plot(*r['xy'].T,c=colors[name],lw=2 if name!='cart12' else 1.2,ls='--' if name=='cart12' else '-',label=labels[name])
            ax.scatter(*r['xy'][-1],c=colors[name],s=60,marker='X',zorder=5)
            c=results[name]['summary']['first_nonfloor_contact']
            if c:
                pts=np.array(c['reported_points']);ax.scatter(*pts[:,:2].T,marker='+',s=100,c=colors[name],zorder=6)
                ax.annotate(f"{name}: contact {c['sim_time']:.3f}s",pts[0,:2],xytext=(.04,.9 if name=='speed10' else .82),textcoords='axes fraction',fontsize=9,bbox=dict(fc='white',ec='none',alpha=.9),arrowprops=dict(arrowstyle='-',lw=.7))
        ax.scatter(*goal[:2],marker='*',s=120,c='#5e3f91',label='Original goal');ax.legend(fontsize=8,loc='lower left');ax.set_title('Hospital E16 | measured paths; added cart absent in both primary runs');save(fig,stem)
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    for name,r in runs.items():
        d=results[name];axes[0].plot(r['times'],d['speed'],c=colors[name],label=labels[name]);axes[1].plot(r['times'],d['measured']['body_v_forward'],c=colors[name])
    axes[0].set_ylabel('Measured world XY speed [m/s]');axes[1].set(ylabel='Measured body forward [m/s]',xlabel='Simulation time [s]');axes[0].legend(fontsize=9)
    for ax in axes:ax.grid(alpha=.2)
    save(fig,'linear_speed_time')
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    for name,r in runs.items():
        d=results[name];axes[0].plot(r['times'],np.degrees(np.unwrap(d['measured']['yaw'])),c=colors[name],label=labels[name]);axes[1].plot(r['times'],d['measured']['body_yaw_rate_measured'],c=colors[name])
    axes[0].set_ylabel('Measured unwrapped yaw [deg]');axes[1].set(ylabel='Measured body yaw rate [rad/s]',xlabel='Simulation time [s]');axes[0].legend(fontsize=9)
    for ax in axes:ax.grid(alpha=.2)
    save(fig,'yaw_angular_velocity_time')
    fig,axes=plt.subplots(1,2,figsize=(16,7))
    for ax,name in zip(axes,['reference','speed10']):
        r=runs[name];context(ax,full);ax.plot(*r['xy'].T,c=colors[name],lw=1.5,label='Measured path')
        xy=np.array([e['agent_pose_at_observation'][:2] for e in r['events']]);artist=ax.scatter(*xy.T,c=colors[name],s=20,zorder=4);np.testing.assert_array_equal(artist.get_offsets(),xy)
        last=None
        for i,p in enumerate(xy,1):
            if i%4==1 and (last is None or np.linalg.norm(p-last)>.4):ax.annotate(f'C{i}',p,xytext=(3,8),textcoords='offset points',fontsize=7);last=p
        offset=(20,-22) if xy[-1,0]<(lo[0]+hi[0])/2 else (-100,-22)
        ax.annotate(f'C{len(xy)} final observation',xy[-1],xytext=offset,textcoords='offset points',fontsize=7,arrowprops=dict(arrowstyle='-',lw=.4));ax.set_title(labels[name]+' | all observation markers');ax.legend(fontsize=7,loc='lower left')
    save(fig,'observation_positions')
    fig,axes=plt.subplots(1,2,figsize=(16,6))
    for ax,name in zip(axes,['reference','speed10']):
        r=runs[name];d=results[name];p=d['summary']['selected_pair'];context(ax,zoom)
        ax.plot(*r['xy'].T,c=colors[name],alpha=.45,label='Measured executed path')
        if p:
            v=d['details'][str(p['old_request_id'])]
            for key,col,ls in [('old_full_world','#c28c32','--'),('fresh_full_world','#7744b5','-')]:ax.plot(*np.array(v[key]).T,c=col,ls=ls,lw=2,label=key.replace('_',' '))
            for key,col in [('old_observation_pose','#c28c32'),('fresh_observation_pose','#7744b5')]:ax.scatter(*v[key][:2],marker='D',s=50,c=col)
            ax.set_title(f"{labels[name]} | C{p['old_request_id']} -> C{p['fresh_request_id']}\nRMSE {p['aligned_overlap_rmse_m']:.3f}m; lateral max {p['max_abs_lateral_revision_m']:.3f}m")
        else:ax.set_title(labels[name]+' | no pre-contact pair')
        ax.legend(fontsize=8,loc='lower left')
    fig.suptitle('First pre-contact FRESH in fixed corner entry region; full 30-point curves\nOwn observation anchors; nominal timing and geometric tangents are derived');save(fig,'precontact_old_fresh')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['reference-run','run-dir','context-run','output-dir']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();out=validate_destination(a.output_dir,[a.reference_run,a.run_dir,a.context_run]);out.mkdir(parents=True)
    hashes=evidence_hashes([a.reference_run,a.run_dir,a.context_run,INVENTORY,RULES,Path(__file__)])
    runs={k:load_saved(v) for k,v in [('reference',a.reference_run),('speed10',a.run_dir),('cart12',a.context_run)]}
    validate_speed(runs['reference']['cfg'],runs['speed10']['cfg'])
    runtimes=[json.loads((r['path']/'inference-runtime.json').read_text()) for r in runs.values()]
    if any(r['artifact_sha256']!=runtimes[0]['artifact_sha256'] for r in runtimes):raise ValueError('Checkpoint mismatch')
    lights=[json.loads((runs[k]['path']/'hospital_lighting_applied.json').read_text()) for k in ['reference','speed10']]
    if lights[0]!=lights[1]:raise ValueError('Original lighting/renderer/scene changed')
    sims=[json.loads((runs[k]['path']/'simulation-runtime.json').read_text()) for k in ['reference','speed10']]
    if sims[0]!=sims[1]:raise ValueError('Simulation/controller runtime changed')
    for name in ['no_cart_pre_inference.json','no_cart_post_run.json']:
        audit=json.loads((a.run_dir/name).read_text())
        if not all(audit[k] for k in ['no_world_e16cart','actor_factory_none','actor_instance_none','original_hospital_collision_inventory_unchanged']):raise ValueError('No-cart/scene audit failed')
    rules=json.loads(RULES.read_text());inventory=json.loads(INVENTORY.read_text());results={k:analyze_run(r,rules,inventory) for k,r in runs.items()}
    summary=dict(primary_config_difference=DIFF,rules=rules,runs={k:v['summary'] for k,v in results.items()})
    write_json(out/'summary.json',summary);write_json(out/'trajectory_coordinates.json',plot_coordinates(runs));bins=[]
    for name,d in results.items():
        for kind in ['observations','pairs','turn_intervals']:
            write_json(out/(name+'_'+kind+'.json'),d[kind]);write_csv(out/(name+'_'+kind+'.csv'),d[kind])
        write_json(out/(name+'_details.json'),d['details'])
        g=runs[name]['cfg']['spatial_turn_gate']
        for axis,edges in [('approach_y',range(2,10)),('corridor_x',range(-1,8))]:
            for low in edges:
                rows=[r for r in d['pairs'] if r['precontact'] and (low<=r['y_world_m']<low+1 and r['x_world_m']>=g['exit_center_x_max_m'] if axis=='approach_y' else low<=r['x_world_m']<low+1 and g['center_y_min_m']<=r['y_world_m']<=g['center_y_max_m'])]
                bins.append(dict(run=name,axis=axis,low_m=low,high_m=low+1,count=len(rows),median_rmse_m=float(np.median([r['aligned_overlap_rmse_m'] for r in rows])) if rows else None,max_lateral_m=max((r['max_abs_lateral_revision_m'] for r in rows),default=None)))
    write_csv(out/'physical_location_bins.csv',bins)
    replay=out/'saved_replay';replay.mkdir();r=runs['speed10'];d=results['speed10'];contact=d['summary']['first_nonfloor_contact']
    write_json(replay/'primary_pair.json',d['summary']['selected_pair'] or dict(old_request_id=None,fresh_request_id=None))
    write_json(replay/'request_metrics.json',d['observations'])
    write_json(replay/'summary.json',dict(first_contact=min(r['contacts'],key=lambda c:c['start_sim_time']) if contact else None,
        first_contact_nearest_robot_tick=dict(zip(['x','y','yaw'],contact['robot_pose'])) if contact else None,
        selection='Pre-frozen first precontact corner observation; inspection only, not a reveal event'))
    figures(out,runs,results,inventory)
    verify_hashes(hashes);write_json(out/'metadata.json',dict(source_files_unchanged=True,source_sha256=hashes,analysis_provenance=provenance(ROOT)))
    print(json.dumps({k:{f:r['summary'][f] for f in ['classification','request_count','trimdoor5_contact','final_pose','travel_distance_m','goal_region_reached']} for k,r in results.items()},indent=2))


if __name__=='__main__':main()
