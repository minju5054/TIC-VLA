"""Saved-only exact-config E16 reproduction comparison; no model/Isaac execution."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import textwrap
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_no_cart_speed import analyze_run,RULES,INVENTORY
from research.analyze_nova_dynamic_handoff import load_saved
from research.hospital_no_cart_reproduction import validate_reference
from research.hospital_no_cart_speed import validate_speed
from research.hospital_spatial_gate import wall_inventory
from research.cart_bypass import plot_coordinates
from research.compare_e16_cart_shift import write_csv
from research.records import write_json,provenance
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination

REPRO_RULES=ROOT/'configs/research/e16_no_cart_15_reproduction_analysis.json'
KEYS=['reasoning_text','reasoning_new_start_step','reasoning_completion_poll_step',
      'reasoning_exact_worker_ready_wall_time_ns','kv_cache_available','delay_reference',
      'delay_seconds_input','sampled_image_request_ids','robot_state_input','previous_waypoints_text']
LABELS={'historical':'Historical no-cart 1.5','reproduction':'NEW no-cart 1.5','speed10':'Recent no-cart 1.0'}
COLORS={'historical':'#267bab','reproduction':'#b0448a','speed10':'#dd742d'}


def signed(value,epsilon):
    return 1 if value>epsilon else -1 if value<-epsilon else 0


def first_persistent_disagreement(bins,w_eps,y_eps):
    """Frozen descriptive sign/persistence rule; never a navigation criterion."""
    bad=[]
    for low in range(2,10):
        rows={r['run']:r for r in bins if r['low_y_m']==low}
        a,b=rows.get('historical'),rows.get('reproduction')
        if a and b and a['count'] and b['count'] and all(
                signed(a[key],eps)!=signed(b[key],eps) for key,eps in
                [('median_w',w_eps),('median_lookahead_left',y_eps)]):bad.append(low)
    return next((low for low in bad if low+1 in bad),None)


def compare_progress(runs,results):
    bins=[]
    for name,run in runs.items():
        cutoff=results[name]['summary']['first_nonfloor_contact']
        cutoff=cutoff['sim_time'] if cutoff else np.inf
        xgate=run['cfg']['spatial_turn_gate']['exit_center_x_max_m']
        for low in range(2,10):
            rows=[o for o in results[name]['observations'] if low<=o['y_world_m']<low+1 and
                  o['x_world_m']>=xgate and o['application_sim_time']<cutoff]
            selected=min(rows,key=lambda o:abs(o['y_world_m']-low-.5)) if rows else None
            bins.append(dict(run=name,low_y_m=low,high_y_m=low+1,count=len(rows),
                median_w=float(np.median([o['controller_w'] for o in rows])) if rows else None,
                median_lookahead_left=float(np.median([o['native_lookahead_left_m'] for o in rows])) if rows else None,
                selected_request_id=selected['request_id'] if selected else None,
                selected_world_xy=[selected['x_world_m'],selected['y_world_m']] if selected else None))
    tol=runs['historical']['cfg']['hospital_validation']
    low=first_persistent_disagreement(bins,tol['command_sign_tolerance_rad_s'],tol['position_sign_tolerance_m'])
    selected={}
    for name in runs:
        row=next((b for b in bins if b['run']==name and b['low_y_m']==low),None)
        fallback=results[name]['summary']['selected_pair']
        selected[name]=dict(request_id=row['selected_request_id'] if row and row['count'] else fallback['fresh_request_id'] if fallback else None,
                            fallback=not bool(row and row['count']))
    return bins,dict(first_sustained_steering_disagreement_y_bin=low,selected=selected,
        identified=low is not None,interpretation='Descriptive direction difference across two common 1m bins; not statistical significance or causal onset')


def enrich(run,result):
    records=[]
    for e,c,o in zip(run['events'],run['chunks'],result['observations']):
        records.append(dict(o,**{k:e[k] for k in KEYS},
            native_chunk=c.tolist(),native_file=str(run['path']/f"raw/requests/request_{e['request_id']:06d}.npy"),
            rgb=str(run['path']/e['rgb_observation_reference']),instruction=e['instruction'],
            reasoning_text_sha256=hashlib.sha256((e['reasoning_text'] or '').encode()).hexdigest()))
    return records


def figures(out,runs,results,records,divergence,inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from PIL import Image
    plt.rcParams.update({'font.size':10,'pdf.fonttype':42})
    walls=wall_inventory(inventory);cfg=runs['historical']['cfg'];g=cfg['spatial_turn_gate']
    goal=np.array(cfg['scene']['goal'][:2]);allxy=np.vstack([*[r['xy'] for r in runs.values()],goal[None,:]])
    lo,hi=allxy.min(0)-1,allxy.max(0)+1;full=[lo[0],hi[0],lo[1],hi[1]];zoom=[-2,9,7,14]
    def context(ax,bounds):
        for w in walls:
            a,b=np.array(w['world_aabb_min']),np.array(w['world_aabb_max']);trim='/Geo_M2_TrimDoor5/' in w['path']
            ax.add_patch(Rectangle(a[:2],*(b-a)[:2],fc='#dc4677' if trim else '#ededed',ec='#dc4677' if trim else '#bfc5cb',lw=2 if trim else .4,zorder=0))
        ax.plot([g['exit_center_x_max_m']]*2,[g['center_y_min_m'],g['center_y_max_m']],c='#309769',lw=1.5)
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.15)
    def save(fig,stem):
        fig.tight_layout();fig.savefig(out/(stem+'.png'),dpi=160,bbox_inches='tight');fig.savefig(out/(stem+'.pdf'),bbox_inches='tight');plt.close(fig)
    for stem,bounds in [('world_xy',full),('corner_zoom',zoom)]:
        fig,ax=plt.subplots(figsize=(14,8));context(ax,bounds)
        for name,r in runs.items():
            ax.plot(*r['xy'].T,c=COLORS[name],lw=2,label=LABELS[name]);ax.scatter(*r['xy'][-1],c=COLORS[name],marker='X',s=70,zorder=5)
            hit=results[name]['summary']['first_nonfloor_contact']
            if hit:ax.scatter(*np.array(hit['reported_points'])[:,:2].T,c=COLORS[name],marker='+',s=100,zorder=5)
        ax.scatter(*goal,marker='*',s=120,c='#604692',label='Original goal');ax.legend(fontsize=9,loc='lower left')
        ax.set_title('Measured Hospital E16 paths | X=end, +=reported contact; magenta=TrimDoor5, green=exit gate');save(fig,stem)
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for name,r in runs.items():
        d=results[name];axes[0].plot(r['times'],np.degrees(np.unwrap(d['measured']['yaw'])),c=COLORS[name],label=LABELS[name])
        axes[1].plot(r['times'],d['speed'],c=COLORS[name])
    axes[0].set_ylabel('Measured unwrapped yaw [deg]');axes[1].set(ylabel='Measured world XY speed [m/s]',xlabel='Simulation time [s]');axes[0].legend(fontsize=9)
    for ax in axes:ax.grid(alpha=.2)
    save(fig,'yaw_velocity_time')
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for name,r in runs.items():
        axes[0].step([float(x['sim_time']) for x in r['robot']],[float(x['target_w']) for x in r['robot']],where='post',c=COLORS[name],label=LABELS[name])
        axes[1].plot(r['times'],results[name]['measured']['body_yaw_rate_measured'],c=COLORS[name])
    axes[0].set_ylabel('Applied angular target [rad/s]');axes[1].set(ylabel='Measured yaw rate [rad/s]',xlabel='Simulation time [s]');axes[0].legend(fontsize=9)
    for ax in axes:ax.grid(alpha=.2)
    save(fig,'angular_command_response')
    fig,axes=plt.subplots(1,3,figsize=(18,7))
    for ax,(name,r) in zip(axes,runs.items()):
        context(ax,zoom);ax.plot(*r['xy'].T,c=COLORS[name],alpha=.6)
        xy=np.array([e['agent_pose_at_observation'][:2] for e in r['events']]);artist=ax.scatter(*xy.T,c=COLORS[name],s=25,zorder=3);np.testing.assert_array_equal(artist.get_offsets(),xy)
        last=None
        for i,p in enumerate(xy,1):
            if zoom[0]<p[0]<zoom[1] and zoom[2]<p[1]<zoom[3] and (last is None or np.linalg.norm(p-last)>.25):
                ax.annotate(f'C{i}',p,xytext=(5 if i%2 else -20,6 if i%2 else -12),textcoords='offset points',fontsize=7);last=p
        ax.set_title(LABELS[name]+'\nMeasured observation positions')
    save(fig,'corner_observations')
    fig,axes=plt.subplots(1,3,figsize=(18,7))
    for ax,(name,r) in zip(axes,runs.items()):
        d=results[name];p=d['summary']['selected_pair'];context(ax,zoom);ax.plot(*r['xy'].T,c=COLORS[name],alpha=.3,label='Measured path')
        if p:
            details=d['details'][str(p['old_request_id'])]
            for key,col,style in [('old_full_world','#bb8a29','--'),('fresh_full_world','#7139ae','-')]:ax.plot(*np.array(details[key]).T,c=col,ls=style,lw=2,label=key.replace('_',' '))
            for key,col in [('old_observation_pose','#bb8a29'),('fresh_observation_pose','#7139ae')]:ax.scatter(*details[key][:2],c=col,marker='D',s=40)
            ax.set_title(f"{LABELS[name]} | C{p['old_request_id']} -> C{p['fresh_request_id']}\nRMSE {p['aligned_overlap_rmse_m']:.3f}m, lateral max {p['max_abs_lateral_revision_m']:.3f}m")
        ax.legend(fontsize=8,loc='lower left')
    fig.suptitle('First pre-contact corner observation | full native predictions projected at OWN observation pose');save(fig,'corner_old_fresh')
    fig,axes=plt.subplots(3,3,figsize=(18,12),gridspec_kw={'height_ratios':[1,1,.9]})
    selected_curves=np.vstack([records[name][divergence['selected'][name]['request_id']-1]['native_chunk'] for name in runs]+[[[0.,0.]]])
    curve_lo=selected_curves.min(0)-.15;curve_hi=selected_curves.max(0)+.15
    for col,name in enumerate(runs):
        rid=divergence['selected'][name]['request_id'];r=records[name][rid-1];p=np.asarray(r['native_chunk']);j=r['lookahead_index']
        with Image.open(r['rgb']) as im:axes[0,col].imshow(im)
        axes[0,col].set_title(f"{LABELS[name]} | C{rid} | recorded RGB");axes[0,col].axis('off')
        ax=axes[1,col];ax.plot(*p.T,'.-',c=COLORS[name],label='Native 30 XY points');ax.scatter(*p[j],c='red',marker='X',s=80,label=f'Lookahead index {j} (0-based)')
        ax.scatter(0,0,c='black',marker='D',s=25,label='Observation origin (separate marker)');ax.set(xlabel='Body forward [m]',ylabel='Body left [m]',xlim=[curve_lo[0],curve_hi[0]],ylim=[curve_lo[1],curve_hi[1]]);ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2);ax.legend(fontsize=7)
        note=(f"obs {r['sim_time']:.3f}s; app {r['application_sim_time']:.3f}s\n"
              f"world ({r['x_world_m']:.3f}, {r['y_world_m']:.3f})m; yaw {np.degrees(r['yaw_rad']):.2f}deg\n"
              f"lookahead ({p[j,0]:.4f}, {p[j,1]:.4f})m\n"
              f"v/w = {r['controller_v']:.4f} / {r['controller_w']:.4f}\n"
              f"reasoning start/poll: {r['reasoning_new_start_step']} / {r['reasoning_completion_poll_step']}\n"
              f"image IDs {r['sampled_image_request_ids']}; delay {r['delay_seconds_input']:.3f}s\n"
              f"action {1000*r['action_wall_s']:.1f}ms; pending {r['pending_sim_s']:.3f}sim s\n"
              'Saved reasoning excerpt (not ground truth):\n'+textwrap.fill((r['reasoning_text'] or '')[:250].replace('\n',' '),60))
        axes[2,col].axis('off');axes[2,col].text(0,1,note,va='top',fontsize=8,family='monospace')
    low=divergence['first_sustained_steering_disagreement_y_bin']
    title=f'First sustained steering-direction disagreement: approach Y=[{low},{low+1})m' if low is not None else 'No sustained sign disagreement identified; first corner observations shown for inspection'
    fig.suptitle(title+'\nNumerical RGB/native/command differences already occur at C1; causal onset is not identified.\nSeparate observation frames; native cross-run differences are not frame-invariant revision metrics',fontsize=11);save(fig,'divergence_diagnostics')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['historical-run','run-dir','speed10-run','output-dir']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();sources=[a.historical_run,a.run_dir,a.speed10_run]
    out=validate_destination(a.output_dir,sources);out.mkdir(parents=True)
    hashes=evidence_hashes([*sources,INVENTORY,RULES,REPRO_RULES,Path(__file__),ROOT/'research/analyze_no_cart_speed.py'])
    runs={k:load_saved(v) for k,v in zip(LABELS,sources)}
    validate_reference(runs['historical']['cfg'],runs['reproduction']['cfg']);validate_speed(runs['historical']['cfg'],runs['speed10']['cfg'])
    expected=runs['historical']['cfg']['instruction']
    for r in runs.values():
        if any(e['instruction']!=expected for e in r['events']):raise ValueError('Instruction changed')
    for filename in ['hospital_lighting_applied.json','simulation-runtime.json']:
        manifests=[json.loads((r['path']/filename).read_text()) for r in runs.values()]
        if any(m!=manifests[0] for m in manifests):raise ValueError('Scene/lighting/controller runtime changed')
    runtimes=[json.loads((r['path']/'inference-runtime.json').read_text()) for r in runs.values()]
    if any(r['artifact_sha256']!=runtimes[0]['artifact_sha256'] for r in runtimes):raise ValueError('Weights changed')
    for filename in ['no_cart_pre_inference.json','no_cart_post_run.json']:
        audit=json.loads((a.run_dir/filename).read_text())
        if not all(audit[k] for k in ['no_world_e16cart','actor_factory_none','actor_instance_none','original_hospital_collision_inventory_unchanged']):raise ValueError('Actor/geometry changed')
    rules=json.loads(RULES.read_text());inventory=json.loads(INVENTORY.read_text())
    results={k:analyze_run(r,rules,inventory) for k,r in runs.items()};records={k:enrich(r,results[k]) for k,r in runs.items()}
    bins,divergence=compare_progress(runs,results)
    boot={k:dict(native_sha256=hashlib.sha256(r['chunks'][0].tobytes()).hexdigest(),
        rgb_sha256=hashlib.sha256((r['path']/r['events'][0]['rgb_observation_reference']).read_bytes()).hexdigest(),
        pose=r['events'][0]['agent_pose_at_observation'],reasoning=records[k][0]['reasoning_text_sha256']) for k,r in runs.items()}
    from PIL import Image
    with Image.open(records['historical'][0]['rgb']) as im:old_rgb=np.asarray(im,dtype=float)
    with Image.open(records['reproduction'][0]['rgb']) as im:new_rgb=np.asarray(im,dtype=float)
    bootstrap_diagnostics=dict(same_observation_pose=boot['historical']['pose']==boot['reproduction']['pose'],
        recorded_rgb_mae_0_255=float(np.abs(old_rgb-new_rgb).mean()),
        recorded_rgb_changed_pixel_fraction=float(np.any(old_rgb!=new_rgb,axis=2).mean()),
        delta_w_rad_s=records['reproduction'][0]['controller_w']-records['historical'][0]['controller_w'],
        note='Recorded PNG comparison, not preprocessed model tensor equality or proof of the cause of divergence')
    summary=dict(primary_config_difference=[],instruction_equality=True,runs={k:v['summary'] for k,v in results.items()},
        divergence=divergence,bootstrap_identity=boot,bootstrap_diagnostics=bootstrap_diagnostics,rules=json.loads(REPRO_RULES.read_text()))
    write_json(out/'summary.json',summary);write_json(out/'trajectory_coordinates.json',plot_coordinates(runs));write_csv(out/'physical_progress_bins.csv',bins)
    for name,d in results.items():
        for key in ['observations','pairs','turn_intervals']:
            write_json(out/(name+'_'+key+'.json'),d[key]);write_csv(out/(name+'_'+key+'.csv'),d[key])
        write_json(out/(name+'_details.json'),d['details']);write_json(out/(name+'_reasoning_native.json'),records[name])
    replay=out/'saved_replay';replay.mkdir();r=runs['reproduction'];d=results['reproduction'];hit=d['summary']['first_nonfloor_contact']
    write_json(replay/'primary_pair.json',d['summary']['selected_pair'] or dict(old_request_id=None,fresh_request_id=None))
    write_json(replay/'request_metrics.json',d['observations'])
    write_json(replay/'summary.json',dict(first_contact=min(r['contacts'],key=lambda c:c['start_sim_time']) if hit else None,
        first_contact_nearest_robot_tick=dict(zip(['x','y','yaw'],hit['robot_pose'])) if hit else None))
    figures(out,runs,results,records,divergence,inventory);verify_hashes(hashes)
    write_json(out/'metadata.json',dict(source_files_unchanged=True,source_sha256=hashes,analysis_provenance=provenance(ROOT)))
    print(json.dumps({k:{f:v['summary'][f] for f in ['classification','request_count','left_turn_completed','goal_region_reached','trimdoor5_contact']} for k,v in results.items()},indent=2))
    print('divergence',divergence)


if __name__=='__main__':main()
