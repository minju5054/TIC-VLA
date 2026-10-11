"""Saved-only two-speed C26 comparison; no simulator/model imports or calls."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_handoffs import metrics as latency, stats
from research.compare_e16_cart_shift import analyze_run, write_csv
from research.cart_bypass import travel_coordinates, forward_crossing, plot_coordinates
from research.cart_geometry import clearance, outline, path_clearance
from research.control import waypoint_command
from research.e16_c26_speed_ablation import validate_ablation, EXPECTED_DIFF
from research.e16_c26_tools import precontact
from research.hospital_spatial_gate import wall_inventory
from research.records import write_json, provenance
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination, state

INVENTORY = ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'
NO_CART = ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01'


def classify(complete, passage, first_cart, first_structure):
    if not complete:
        return 'EXECUTION FAILED'
    clear_time = passage['clear_region']['sim_time'] if passage['clear_region'] else float('inf')
    if first_structure is not None and first_structure <= min(first_cart if first_cart is not None else float('inf'), clear_time):
        return 'OTHER STRUCTURAL CONTACT BEFORE CART'
    if first_cart is not None and first_cart <= clear_time:
        return 'CONTACT BEFORE BYPASS'
    if passage['clears_before_any_contact'] and 'BYPASS' in passage['geometric_passage_side'] and 'NO CLEAR' not in passage['geometric_passage_side']:
        return 'COLLISION-FREE COMPLETE BYPASS'
    return 'INCOMPLETE APPROACH / DOES NOT REACH CART'


def read_visibility(run, frozen):
    from PIL import Image
    result = []
    for e in run['events']:
        name = Path(e['rgb_observation_reference']).name
        folder = run['path']/'raw/visibility'
        v = json.loads((folder/(name+'.json')).read_text())
        mask_path = folder/(name+'.mask.png')
        mask = np.asarray(Image.open(mask_path)) > 0
        if mask.shape != (1080, 1920) or int(mask.sum()) != v['cart_visible_pixel_count']:
            raise ValueError('Semantic mask/count mismatch')
        mapped = {k.replace('cart_', 'human_'): x for k, x in v.items() if k.startswith('cart_')}
        if state(mapped, frozen['visibility_rules']) != v['state']:
            raise ValueError('Frozen semantic criterion changed')
        result.append(dict(v, request_id=e['request_id'], sim_time=e['observation']['sim_time'],
                           rgb=str(run['path']/e['rgb_observation_reference']), mask=str(mask_path)))
    return result


def contact_info(run, spans):
    if not spans:
        return None
    c = min(spans, key=lambda s: s['start_sim_time'])
    t = c['start_sim_time']
    return dict(sim_time=t, robot_xy=[float(np.interp(t, run['times'], run['xy'][:,j])) for j in range(2)],
                reported_points=[p['position'] for p in c['first_report']['contacts']], colliders=c['colliders'])


def structural_curve_clearance(world, inventory, radius):
    # Conservative enabled collider AABBs intersecting body height, no floor.
    best = None
    for c in inventory:
        if not c['enabled'] or c['world_aabb_min'] is None:
            continue
        lo, hi = np.array(c['world_aabb_min']), np.array(c['world_aabb_max'])
        if not (lo[2] < .65 and hi[2] > .15):
            continue
        proxy = dict(center_world_xyz=((lo+hi)/2).tolist(), yaw_rad=0., footprint_half_m=((hi-lo)[:2]/2).tolist())
        d = path_clearance(world, proxy, radius)['minimum_clearance_m']
        if best is None or d < best['clearance_m']:
            best = dict(clearance_m=d, prim=c['path'])
    return best


def summarize(run, frozen, inventory):
    cart, radius = frozen['cart'], frozen['robot_radius_m']
    data = analyze_run(run, cart, radius)
    vis = read_visibility(run, frozen)
    first = min((c['start_sim_time'] for c in run['contacts']), default=None)
    cart_contact = contact_info(run, data['cart_contacts'])
    structural_spans = [c for c in run['contacts'] if any(p.startswith('/Root/') for p in c['colliders'])]
    structure = contact_info(run, structural_spans)
    extent = max(cart['footprint_half_m'])+radius
    # Same physical window for both runs, inherited reference approach extent.
    entry_plane = -extent-3.*1.5
    observations = []
    yaw_filter = None
    for e, chunk, v in zip(run['events'], run['chunks'], vis):
        command, yaw_filter, index = waypoint_command(chunk, yaw_filter, **run['cfg']['controller'])
        np.testing.assert_allclose(command, e['controller_command'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(yaw_filter, e['yaw_filter_state'], atol=1e-12, rtol=0)
        if index != e['lookahead_index']:
            raise ValueError('Lookahead recomputation mismatch')
        xy = e['agent_pose_at_observation'][:2]
        progress, left = travel_coordinates(xy, cart)
        observations.append(dict(sim_time=e['observation']['sim_time'],
            x_world_m=xy[0], y_world_m=xy[1], yaw_rad=e['agent_pose_at_observation'][2],
            cart_progress_m=float(progress), cart_left_m=float(left),
            cart_center_distance_m=float(np.linalg.norm(np.array(xy)-cart['center_world_xyz'][:2])),
            cart_clearance_proxy_m=float(clearance(xy, cart, radius)), visibility=v['state'],
            lookahead_index=index, native_lookahead_forward_m=float(chunk[index,0]), native_lookahead_left_m=float(chunk[index,1]),
            controller_v=command[0], controller_w=command[1], application_sim_time=e['application']['sim_time'], **latency(e)))
    for row in data['pairs']:
        i = row['old_request_id']
        obs = observations[i]
        row.update(precontact=precontact(row, first), cart_progress_m=obs['cart_progress_m'],
                   cart_center_distance_m=obs['cart_center_distance_m'], fresh_lookahead_index=obs['lookahead_index'],
                   fresh_lookahead_forward_m=obs['native_lookahead_forward_m'], fresh_lookahead_left_m=obs['native_lookahead_left_m'],
                   fresh_controller_v=obs['controller_v'], fresh_controller_w=obs['controller_w'])
        row['common_spatial_approach'] = entry_plane <= row['cart_progress_m'] <= extent
        d = data['details'][str(i)]
        d['old_full_world_xy'] = run['worlds'][i-1].tolist()
        aligned = d['alignment']
        if aligned:
            horizon = np.array(aligned['absolute_times'])-row['fresh_observation_sim_time']
            error = np.linalg.norm(aligned['delta'], axis=1)
            for label, mask in [('near', horizon <= 1.), ('far', horizon > 1.)]:
                row[label+'_rmse_m'] = float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None
        row['fresh_complete_downstream_prediction'] = bool(forward_crossing(run['worlds'][i], .1*np.arange(1,31), cart, extent))
        row['fresh_structural_proxy'] = None
        if row['precontact'] and row['fresh_clear'] and row['fresh_complete_downstream_prediction']:
            row['fresh_structural_proxy'] = structural_curve_clearance(run['worlds'][i], inventory, radius)
        row['valid_fresh_proxy_bypass'] = bool(row['precontact'] and row['fresh_clear'] and
            row['fresh_complete_downstream_prediction'] and row['fresh_structural_proxy'] and row['fresh_structural_proxy']['clearance_m']>0)
    pre = [r for r in data['pairs'] if r['precontact']]
    post = [r for r in data['pairs'] if not r['precontact']]
    approach = [r for r in pre if r['common_spatial_approach']]
    peak = max(approach, key=lambda r: (r['max_abs_lateral_revision_m'], -r['old_request_id'])) if approach else None
    clear = next((o for o in observations if o['visibility']=='CLEAR'), None)
    # If execution never enters the approach window, show first CLEAR only as
    # context. Do not promote it to the preregistered approach selection.
    fallback = next((r for r in pre if clear and r['fresh_request_id']==clear['request_id']), None) if peak is None else None
    visible_id = next((v['request_id'] for v in vis if v['visible']), None)
    arc = np.r_[0., np.linalg.norm(np.diff(run['xy'], axis=0), axis=1).cumsum()]
    measured = {k: np.array([float(r[k]) for r in run['robot']]) for k in ['body_v_forward','body_yaw_rate_measured','vx_world','vy_world','yaw']}
    speed = np.hypot(measured['vx_world'], measured['vy_world'])
    before = run['times'] < (first if first is not None else float('inf'))
    progress = travel_coordinates(run['xy'], cart)[:,0]
    matched = before & (progress >= entry_plane) & (progress <= -extent)
    def window_stats(mask):
        return {k: stats(v[mask]) for k,v in dict(world_xy_speed_m_s=speed, body_forward_m_s=measured['body_v_forward'],
            absolute_yaw_rate_rad_s=np.abs(measured['body_yaw_rate_measured'])).items()} if mask.any() else None
    def pair_stats(rows):
        return dict(count=len(rows), **{k:stats([r[k] for r in rows if r[k] is not None]) for k in
            ['aligned_overlap_rmse_m','max_abs_lateral_revision_m','mean_abs_lateral_revision_m','near_rmse_m','far_rmse_m']}) if rows else {'count':0}
    latency_summary = {k:stats([o[k] for o in observations[1:]]) for k in ['action_wall_s','observation_to_switch_wall_s','pending_sim_s','robot_translation_xy_m','robot_yaw_change_deg']}
    conflict_clear = [[r['old_request_id'],r['fresh_request_id']] for r in pre if r['old_predicts_collision'] and r['fresh_clear']]
    valid = [[r['old_request_id'],r['fresh_request_id']] for r in pre if r['valid_fresh_proxy_bypass']]
    summary = dict(run=str(run['path']), complete=run['complete'], request_count=len(observations),
        outcome=classify(run['complete'], data['outcome'], cart_contact['sim_time'] if cart_contact else None, structure['sim_time'] if structure else None),
        passage=data['outcome'], first_cart_contact=cart_contact, first_structural_contact=structure,
        cart_contact_spans=len(data['cart_contacts']), structural_contact_spans=len(structural_spans), total_contact_spans=len(run['contacts']),
        travel_distance_m=float(arc[-1]), final_xy=run['xy'][-1].tolist(), final_yaw_rad=float(measured['yaw'][-1]),
        minimum_center_distance_m=float(np.linalg.norm(run['xy']-cart['center_world_xyz'][:2],axis=1).min()),
        speeds_all=window_stats(np.ones(len(before), bool)), speeds_precontact=window_stats(before), speeds_common_approach=window_stats(matched),
        common_spatial_progress_interval_m=[entry_plane,extent], common_speed_interval_m=[entry_plane,-extent],
        first_visible=observations[visible_id-1] if visible_id else None, first_clear=clear,
        observations_before_first_contact=sum(o['sim_time'] < first for o in observations) if first else len(observations),
        clear_observations_before_contact=sum(o['sim_time'] < first and o['sim_time']>=clear['sim_time'] for o in observations) if first and clear else None,
        clear_to_first_cart_contact_s=cart_contact['sim_time']-clear['sim_time'] if cart_contact and clear else None,
        clear_to_first_cart_contact_travel_m=float(np.interp(cart_contact['sim_time'],run['times'],arc)-np.interp(clear['sim_time'],run['times'],arc)) if cart_contact and clear else None,
        clear_to_first_nonfloor_contact_s=first-clear['sim_time'] if first and clear else None,
        latency_nonbootstrap=latency_summary, bootstrap=observations[0],
        precontact_pairs=pair_stats(pre), postcontact_pairs_excluded=pair_stats(post), common_approach_precontact_pairs=pair_stats(approach),
        selected_precontact_pair=peak, first_clear_context_pair_outside_approach=fallback,
        precontact_old_conflict_fresh_clear_pairs=conflict_clear,
        valid_fresh_proxy_bypass_pairs=valid, valid_fresh_avoidance='YES' if valid else 'NO' if cart_contact else 'INSUFFICIENT',
        controller_commands_recomputed=len(observations))
    return dict(summary=summary, pairs=data['pairs'], details=data['details'], observations=observations, visibility=vis,
                speed=speed, measured=measured, progress=progress)


def make_figures(out, runs, results, frozen, inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'font.size':10,'pdf.fonttype':42})
    cart, radius = frozen['cart'], frozen['robot_radius_m']
    colors = {'reference':'#3175aa','slower':'#d65c24'}
    labels = {'reference':'Reference max_v=1.5','slower':'Reduced max_v=1.2'}
    def context(ax, bounds=None):
        for w in wall_inventory(inventory):
            lo,hi=np.array(w['world_aabb_min']),np.array(w['world_aabb_max'])
            ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],fc='#eeeeee',ec='#ccd0d4',lw=.3,zorder=0))
        ax.plot(*outline(cart).T,c='#943c83',lw=2,label='Fixed C26 cart')
        ax.plot(*outline(cart,radius).T,c='#943c83',lw=1,ls=':',label='Cart + robot disk proxy')
        if bounds:ax.set(xlim=bounds[:2],ylim=bounds[2:])
        ax.set_aspect('equal',adjustable='box');ax.set(xlabel='World X [m]',ylabel='World Y [m]');ax.grid(alpha=.15)
    def save(fig, stem):
        fig.tight_layout();fig.savefig(out/(stem+'.png'),dpi=170,bbox_inches='tight');fig.savefig(out/(stem+'.pdf'),bbox_inches='tight');plt.close(fig)
    allxy=np.vstack([*[r['xy'] for r in runs.values()],outline(cart,radius)]);lo=allxy.min(0)-1.;hi=allxy.max(0)+1.
    full=[lo[0],hi[0],lo[1],hi[1]];cx,cy=cart['center_world_xyz'][:2]
    for stem,bounds in [('actual_world_xy',full),('cart_contact_zoom',[cx-2,cx+9,cy-3,cy+2.5])]:
        fig,ax=plt.subplots(figsize=(12,7));context(ax,bounds)
        for name,run in runs.items():
            ax.plot(*run['xy'].T,c=colors[name],lw=2,label=labels[name])
            ax.scatter(*run['xy'][0],c=colors[name],marker='s',s=45)
            ax.scatter(*run['xy'][-1],c=colors[name],marker='X',s=65,label=name+' final')
            for category,mark in [('first_cart_contact','x'),('first_structural_contact','+')]:
                c=results[name]['summary'][category]
                if c:
                    for point in c['reported_points']:ax.scatter(*point[:2],c=colors[name],marker=mark,s=90)
                    label_pos=(.60,.88) if name=='slower' else (.08,.89) if category=='first_structural_contact' else (.12,.70)
                    ax.annotate(f"{name} {category.replace('first_','').replace('_',' ')}\n{c['sim_time']:.3f}s",c['reported_points'][0][:2],xytext=label_pos,textcoords='axes fraction',fontsize=8,bbox=dict(facecolor='white',edgecolor='none',alpha=.9),arrowprops=dict(arrowstyle='-',lw=.6))
        ax.legend(fontsize=8);ax.set_title('Measured execution | same fixed C26 cart, 48 requests each');save(fig,stem)
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for name,run in runs.items():
        r=results[name];t=run['times'];c=colors[name]
        axes[0].plot(t,r['speed'],c=c,label=labels[name]);axes[0].plot(t,r['measured']['body_v_forward'],c=c,ls=':',alpha=.7)
        axes[1].plot(t,r['measured']['body_yaw_rate_measured'],c=c)
        axes[2].plot(t,np.degrees(np.unwrap(r['measured']['yaw'])),c=c)
        for ax in axes:
            contact=r['summary']['first_cart_contact']
            if contact:ax.axvline(contact['sim_time'],c=c,ls='--',alpha=.6)
    for ax,y in zip(axes,['World XY speed [m/s]\n(dotted: body forward)','Measured body yaw rate [rad/s]','Measured unwrapped yaw [deg]']):ax.set_ylabel(y);ax.grid(alpha=.2)
    axes[0].legend();axes[-1].set_xlabel('Simulation time [s]');fig.suptitle('Measured motion; dashed vertical lines = first cart contact');save(fig,'actual_speed_time')
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for name,run in runs.items():
        for ax,y in zip(axes,[np.linalg.norm(run['xy']-cart['center_world_xyz'][:2],axis=1),clearance(run['xy'],cart,radius),results[name]['progress']]):ax.plot(run['times'],y,c=colors[name],label=labels[name])
    for ax,y in zip(axes,['Cart center distance [m]','Cart footprint clearance proxy [m]','Progress along frozen cart frame [m]']):ax.set_ylabel(y);ax.grid(alpha=.2)
    axes[1].axhline(0,c='black',lw=.7);axes[2].axhline(0,c='black',lw=.7);axes[0].legend();axes[-1].set_xlabel('Simulation time [s]');save(fig,'cart_distance_clearance_time')
    fig,axes=plt.subplots(1,2,figsize=(16,5.5))
    for ax,(name,run) in zip(axes,runs.items()):
        r=results[name];peak=r['summary']['selected_precontact_pair'] or r['summary']['first_clear_context_pair_outside_approach'];context(ax,[cx-2,cx+8,cy-3,cy+2])
        ax.plot(*run['xy'].T,c=colors[name],alpha=.5,label='Actual executed path')
        if peak:
            i=peak['old_request_id'];d=r['details'][str(i)]
            points=np.vstack([d['old_full_world_xy'],d['fresh_world_xy'],outline(cart,radius)])
            lo,hi=points.min(0)-.7,points.max(0)+.7
            ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
            ax.plot(*np.array(d['old_full_world_xy']).T,'--',c='#c29235',lw=2,label=f'OLD C{i} full (30)')
            ax.plot(*np.array(d['old_remaining_world_xy']).T,c='#c29235',lw=2.5,label='OLD remaining')
            ax.plot(*np.array(d['fresh_world_xy']).T,c='#7548b6',lw=2,label=f'FRESH C{i+1} full (30)')
            for e,col in [(run['events'][i-1],'#c29235'),(run['events'][i],'#7548b6')]:ax.scatter(*e['agent_pose_at_observation'][:2],marker='D',c=col,s=60)
            role='Approach peak' if r['summary']['selected_precontact_pair'] else 'First CLEAR context; NEVER reached approach'
            ax.set_title(f"{labels[name]} | C{i} -> C{i+1}\n{role}\nRMSE {peak['aligned_overlap_rmse_m']:.3f} m; lateral max {peak['max_abs_lateral_revision_m']:.3f} m")
        ax.legend(fontsize=8)
    fig.suptitle('Pre-contact OLD/FRESH inspection; unavailable approach data explicitly identified\nOwn observation anchors; nominal temporal alignment; no native yaw');save(fig,'precontact_old_fresh')
    fig,axes=plt.subplots(1,2,figsize=(16,7))
    for ax,(name,run) in zip(axes,runs.items()):
        context(ax,full);r=results[name];ax.plot(*run['xy'].T,c=colors[name],lw=1.3,label='Actual path')
        obs=np.array([e['agent_pose_at_observation'][:2] for e in run['events']]);artist=ax.scatter(*obs.T,s=18,c=colors[name],zorder=4)
        np.testing.assert_array_equal(artist.get_offsets(),obs)
        last_label=None
        for o in r['observations']:
            xy=np.array([o['x_world_m'],o['y_world_m']])
            if o['request_id']%3==1 and (last_label is None or np.linalg.norm(xy-last_label)>.45):
                ax.annotate('C'+str(o['request_id']),xy,xytext=(4,8),textcoords='offset points',fontsize=7);last_label=xy
        ax.annotate('C48 final',obs[-1],xytext=(-50,12),textcoords='offset points',fontsize=7,arrowprops=dict(arrowstyle='-',lw=.4))
        o=r['summary']['first_clear']
        if o:
            ax.scatter(o['x_world_m'],o['y_world_m'],marker='*',c='#a72472',s=180,zorder=5,label='First CLEAR')
            ax.annotate(f"C{o['request_id']} CLEAR\nt={o['sim_time']:.3f}s\ncart {o['cart_center_distance_m']:.2f}m",(o['x_world_m'],o['y_world_m']),xytext=(-100,-45),textcoords='offset points',fontsize=9,arrowprops=dict(arrowstyle='-'))
        ax.legend(fontsize=8);ax.set_title(labels[name]+' | all 48 observation markers')
    save(fig,'observation_locations_clear')


def viewer_adapter(out, run, result, reference, frozen, hashes):
    peak=result['summary']['selected_precontact_pair'] or result['summary']['first_clear_context_pair_outside_approach']
    if not peak:return
    i=peak['old_request_id'];d=result['details'][str(i)];s=result['summary'];cart=frozen['cart'];radius=frozen['robot_radius_m']
    v=[{**r,**{k.replace('cart_','human_'):x for k,x in r.items() if k.startswith('cart_')}} for r in result['visibility']]
    candidate=dict(id=1,position=cart['center_world_xyz'],visibility=v,strict_qualified=False,selection_rank=None,
        failures=['Speed diagnostic; original bypass gate FAIL'],old_request_id=i,fresh_request_id=i+1,
        first_visible_request_id=s['first_visible']['request_id'] if s['first_visible'] else None,
        first_clear_request_id=s['first_clear']['request_id'] if s['first_clear'] else None,
        old_full_world_xy=d['old_full_world_xy'],remaining_world_xy=d['old_remaining_world_xy'],fresh_world_xy=d['fresh_world_xy'],
        old_min_clearance_m=peak['old_remaining_cart_clearance_m'],robot_current_clearance_at_reveal_m=peak['current_cart_clearance_m'],conflict=d['old_conflict'],
        bypass={'pass':False,'centerline_xy':[],'clearance_m':None},cart_outline=outline(cart).tolist(),inflated_outline=outline(cart,radius).tolist(),
        approach_sim_time=s['passage']['interaction_start_s'],closest_sim_time=s['passage']['closest_approach']['sim_time'],
        structural_contact_sim_time=min((c['start_sim_time'] for c in run['contacts']),default=None))
    cfg=dict(baseline_run=str(run['path']),reference_baseline_run=str(NO_CART),baseline_analysis=str(out),
        collision_inventory=str(INVENTORY),cart_frozen=frozen,robot_radius_m=radius,visibility=frozen['visibility_rules'],cart_event_jumps=True,
        secondary_cart_runs=[str(reference['path'])])
    folder=out/'viewer_data';folder.mkdir()
    for name,obj in [('preregistered.json',{'config':cfg}),('candidates.json',[candidate]),('summary.json',{'obstacle_type':'cart','selected_candidate_id':None,'diagnostic_candidate_id':1}),('source_evidence_sha256.json',hashes)]:write_json(folder/name,obj)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['reference-run','run-dir','preflight-dir','output-dir']:p.add_argument('--'+name,required=True,type=Path)
    a=p.parse_args();out=validate_destination(a.output_dir,[a.reference_run,a.run_dir,a.preflight_dir]);out.mkdir(parents=True)
    frozen=json.loads((a.preflight_dir/'freeze.json').read_text());inventory=json.loads(INVENTORY.read_text())
    hashes=evidence_hashes([a.reference_run,a.run_dir,a.preflight_dir,NO_CART,INVENTORY,Path(__file__)])
    runs={'reference':load_saved(a.reference_run),'slower':load_saved(a.run_dir)}
    validate_ablation(runs['reference']['cfg'],runs['slower']['cfg'],frozen)
    runtimes=[json.loads((r['path']/'inference-runtime.json').read_text()) for r in runs.values()]
    if runtimes[0]['artifact_sha256']!=runtimes[1]['artifact_sha256']:raise ValueError('Checkpoint mismatch')
    results={name:summarize(r,frozen,inventory) for name,r in runs.items()}
    write_json(out/'summary.json',dict(exact_config_difference=EXPECTED_DIFF,runs={k:v['summary'] for k,v in results.items()},
        selection_rule='Largest max absolute lateral revision, precontact observation AND application, fixed reference spatial approach window; ties earliest request.',
        interpretation='Single runs, distinct closed-loop/model histories and driver versions; descriptive comparison, no causal speed or reconciliation claim.'))
    coordinates=plot_coordinates(runs);write_json(out/'trajectory_coordinates.json',coordinates)
    bins=[]
    for name,result in results.items():
        for kind in ['pairs','observations','visibility']:
            write_json(out/(name+'_'+kind+'.json'),result[kind]);write_csv(out/(name+'_'+kind+'.csv'),result[kind])
        write_json(out/(name+'_pair_details.json'),result['details'])
        # Fixed 1 m physical progress bins; descriptive counts, never paired by C index.
        for low in range(-10,1):
            rows=[r for r in result['pairs'] if r['precontact'] and low<=r['cart_progress_m']<low+1]
            bins.append(dict(run=name,progress_low_m=low,progress_high_m=low+1,count=len(rows),
                median_rmse_m=float(np.median([r['aligned_overlap_rmse_m'] for r in rows])) if rows else None,
                max_lateral_m=max((r['max_abs_lateral_revision_m'] for r in rows),default=None)))
    write_csv(out/'precontact_progress_bins.csv',bins)
    make_figures(out,runs,results,frozen,inventory)
    viewer_adapter(out,runs['slower'],results['slower'],runs['reference'],frozen,hashes)
    verify_hashes(hashes);write_json(out/'metadata.json',dict(source_files_unchanged=True,source_evidence_sha256=hashes,**provenance(ROOT)))
    print(json.dumps({name:{k:r['summary'][k] for k in ['outcome','request_count','travel_distance_m','first_cart_contact','valid_fresh_avoidance']} for name,r in results.items()},indent=2))


if __name__=='__main__':main()
