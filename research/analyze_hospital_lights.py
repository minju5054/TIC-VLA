"""Saved-only spatial baseline gate, wall context and original E16 comparison."""
import argparse,csv,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.hospital_episode import digest
from research.analyze_nova_hospital import analyze as turn_analysis
from research.analyze_nova_dynamic_handoff import load_saved,request_metrics,pair_metrics
from research.analyze_nova_bright import SETTINGS
from research.analyze_nova_response import contacts
from research.hospital_spatial_gate import spatial_gate,wall_inventory,bound_clearance,first_contact
from research.nova_bright import bright_pass


def analyze(source,original,audit,output):
    source,original,audit,output=map(lambda p:Path(p).resolve(),[source,original,audit,output])
    if any(p==output or p in output.parents or output in p.parents for p in [source,original,audit]):raise ValueError('Separate output required')
    hashes={str(p):digest(p) for d in [source,original,audit] for p in d.rglob('*') if p.is_file()}
    output.mkdir(parents=True,exist_ok=False)
    turns={'v2':turn_analysis(source,output/'turn'),'original':turn_analysis(original,output/'original_turn')}
    run,base=load_saved(source),load_saved(original);cfg=run['cfg'];gate=cfg['spatial_turn_gate'];cross=spatial_gate(run['events'],gate)
    brightness=[{**json.loads((source/'raw/brightness'/(Path(e['rgb_observation_reference']).name+'.json')).read_text()),'request_id':e['request_id']} for e in run['events']]
    n=len(cross['required_request_ids']);bright=all(bright_pass(s,cfg['bright_validation']) for s in brightness[:n]);spans=contacts(source);first=first_contact(spans)
    end=run['events'][n-1]['application']['sim_time'];early=[s for s in spans if s['start_sim_time']<=end]
    camera=turns['v2']['camera_proximity_request_ids'];camera_clean=not any(i<=n for i in camera)
    passed=bool(len(run['events'])==48 and cross['crossing_plus_one_observation'] and bright and camera_clean
        and turns['v2']['stage_b_decision']=='NOVA CARTER E16 TURN EXECUTION VALIDATED' and not spans)
    # Any logged wall contact vetoes the next human experiment per user stop rule;
    # required spatial-window contacts remain separately reported.
    groups=turns['v2']['actual_left_interval_groups_by_fresh_id'];fresh=groups[0][0] if groups else (cross['crossing_request_id'] or 2);fresh=max(2,fresh)
    rows=request_metrics(run,SETTINGS);pair,details=pair_metrics(run,rows,SETTINGS,fresh-1,fresh)
    write_json(output/'primary_pair.json',pair);write_json(output/'request_metrics.json',rows);write_json(output/'details.json',details);write_json(output/'brightness.json',brightness)
    walls=wall_inventory(json.loads((audit/'collision_geometry.json').read_text()));comparison={};traces={}
    for name,data in [('v2',run),('original',base)]:
        trace=[]
        for r in data['robot']:
            trace.append({'tick':int(r['tick']),'sim_time':float(r['sim_time']),'x':float(r['x']),'y':float(r['y']),'yaw_rad':float(r['yaw']),
                'target_v':float(r['target_v']),'target_w':float(r['target_w']),**bound_clearance([float(r['x']),float(r['y'])],walls,gate['robot_conservative_radius_m'])})
        traces[name]=trace;write_json(output/(name+'_wall_clearance.json'),trace)
        rr=json.loads((output/('turn' if name=='v2' else 'original_turn')/'request_metrics.json').read_text())
        selected=[r for r in rr if gate['exit_center_x_max_m']<=r['robot_world_x']<=8 and gate['source_south_corner']['world_aabb_max'][1]-1<=r['robot_world_y']<=gate['source_north_wall']['world_aabb_min'][1]+.5]
        fields=['native_endpoint_left_m','lookahead_left_m','lookahead_bearing_deg','controller_w']
        comparison[name]={'turn_region_request_ids':[r['request_id'] for r in selected],
            'turn_region_statistics':{k:{'median':float(np.median([r[k] for r in selected])),'min':min(r[k] for r in selected),'max':max(r[k] for r in selected)} for k in fields} if selected else {},
            'actual_signed_yaw_change_deg':turns[name]['signed_yaw_change_deg'],'maximum_signed_yaw_change_deg':turns[name]['maximum_signed_yaw_change_deg'],
            'contact_span_count':len(contacts(data['path'])),'spatial_gate':spatial_gate(data['events'],gate)}
    pre=[r for r in traces['v2'] if first is None or r['sim_time']<first['start_sim_time']];closest=min(pre,key=lambda r:r['wall_bound_clearance_m']) if pre else None
    first_pose=min(traces['v2'],key=lambda r:abs(r['sim_time']-first['start_sim_time'])) if first else None
    summary={'decision':'BASELINE SPATIAL PREREQUISITE PASS' if passed else 'INSUFFICIENT EVIDENCE','baseline_spatial_gate_pass':passed,
        'source_run':str(source),'prediction_count':len(run['events']),'spatial_gate':cross,'bright_required_window_pass':bright,'camera_required_window_pass':camera_clean,
        'failed_brightness_ids':[r['request_id'] for r in brightness if not bright_pass(r,cfg['bright_validation'])],
        'first_contact':first,'first_contact_nearest_robot_tick':first_pose,'closest_approach_proxy_before_contact':closest,'required_window_contact_spans':early,
        'all_contact_span_count':len(spans),'turn_validation':turns['v2']['stage_b_decision'],'inspection_pair':[fresh-1,fresh],
        'comparison':comparison,'note':'Baseline inspection only; no human reveal. Two closed-loop runs are not identical-input or isolated lighting-causal trials.'}
    write_json(output/'summary.json',summary);write_json(output/'comparison.json',comparison)
    write_json(output/'metadata.json',{'source_files_sha256':hashes,**provenance(ROOT)})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(13,6))
    for name,color in [('original','gray'),('v2','tab:blue')]:
        t=traces[name];axes[0].plot([r['x'] for r in t],[r['y'] for r in t],label=name,color=color)
        axes[1].plot([r['sim_time'] for r in t],[r['wall_bound_clearance_m'] for r in t],label=name,color=color)
    axes[0].plot([gate['exit_center_x_max_m']]*2,[gate['center_y_min_m'],gate['center_y_max_m']],'g-',lw=4,label='Fixed gate')
    for wall in walls:
        lo,hi=wall['world_aabb_min'],wall['world_aabb_max']
        if lo[0]<9 and hi[0]>-2 and lo[1]<14 and hi[1]>6:axes[0].plot([lo[0],hi[0],hi[0],lo[0],lo[0]],[lo[1],lo[1],hi[1],hi[1],lo[1]],color='black',alpha=.18)
    if first_pose:axes[0].scatter(first_pose['x'],first_pose['y'],c='red',marker='x',s=90,label='First contact')
    axes[0].set(xlim=(-2,9),ylim=(6,14),xlabel='World X [m]',ylabel='World Y [m]',title='Same physical corner; separate closed-loop runs');axes[0].set_aspect('equal')
    axes[1].axhline(0,color='black',ls=':');axes[1].set(xlabel='Simulation time [s]',ylabel='Conservative disk / wall AABB clearance [m]',title='Descriptive proxy; negative is not contact proof')
    if first:axes[1].axvline(first['start_sim_time'],color='red',ls='--',label='V2 first contact')
    for ax in axes:ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(output/'spatial_gate_wall_comparison.png',dpi=170);plt.close(fig)
    if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Source modified')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['comparison','required_window_contact_spans']},indent=2));return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['run-dir','original-run-dir','audit-dir','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args();analyze(a.run_dir,a.original_run_dir,a.audit_dir,a.output_dir)
