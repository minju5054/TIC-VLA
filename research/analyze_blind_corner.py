"""Saved-only blind-turn gate and publication trajectory plot, never inference."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.blind_corner import baseline_crossing
from research.analyze_nova_dynamic_handoff import load_saved,request_metrics,pair_metrics
from research.analyze_nova_bright import SETTINGS
from research.nova_bright import bright_pass
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.records import write_json,provenance


def trajectory_plot(out,run,selected,summary,inventory_path=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon,Circle,Patch
    inventory_path=inventory_path or ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'
    inventory=json.loads(Path(inventory_path).read_text())
    human=np.array(selected['human_position']);radius=SETTINGS['robot_radius_m']+SETTINGS['human_radius_m']
    points=np.vstack([run['xy'],human[:2]]);lo=points.min(0)-1.;hi=points.max(0)+1.
    polygons=[]
    for c in inventory:
        a,b=c['world_aabb_min'],c['world_aabb_max']
        if a is None or b is None or not c['enabled'] or a[2]>.5 or b[2]<1.:continue
        if np.any(np.array(b[:2])<lo) or np.any(np.array(a[:2])>hi):continue
        polygons.append({'prim':c['path'],'xy':[[a[0],a[1]],[b[0],a[1]],[b[0],b[1]],[a[0],b[1]]]})
    coords={'executed_robot_world_xy':run['xy'].tolist(),'executed_sim_times':run['times'].tolist(),
            'proposed_human_world_xyz':human.tolist(),'human_executed':False,
            'combined_proxy_radius_m':radius,'wall_aabb_polygons':polygons,'decision':summary['decision'],
            'coordinate_rule':'World metres. No spatial alignment. Predictions, if present, use their own observation poses.'}
    plt.rcParams.update({'pdf.fonttype':42,'font.size':10})
    fig,(overview,ax)=plt.subplots(1,2,figsize=(14,7),gridspec_kw={'width_ratios':[1,3]})
    for panel in (overview,ax):
        for wall in polygons:panel.add_patch(Polygon(wall['xy'],closed=True,color='#737b85',alpha=.22,linewidth=0))
        panel.plot(*run['xy'].T,color='#146b9e',lw=2,label='Actual executed robot path',zorder=3)
        panel.set(xlabel='World X [m]',ylabel='World Y [m]')
        panel.set_aspect('equal',adjustable='box');panel.grid(alpha=.2)
    overview.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),title='Complete saved baseline')
    overview.scatter(*run['xy'][-1],marker='X',s=55,color='#146b9e')
    local=run['xy'][run['xy'][:,1]<=run['cfg']['blind_corner']['baseline_gate']['exit_y_max_m']]
    near=np.vstack([local,human[:2]]);near_lo=near.min(0)-.65;near_hi=near.max(0)+.65
    ax.scatter(*run['xy'][0],marker='s',s=65,color='#146b9e',label='Actual start')
    ax.scatter(*human[:2],marker='*',s=150,color='#b02782',label='Proposed human (not executed)')
    ax.add_patch(Circle(human[:2],radius,fill=False,color='#b02782',ls='--',lw=1.3,label='Combined conflict proxy'))
    gate=run['cfg']['blind_corner']['baseline_gate'];ax.plot([gate['exit_x_max_m']]*2,[gate['exit_y_min_m'],gate['exit_y_max_m']],color='#229a70',lw=2,label='Preregistered turn-exit boundary')
    corner=[20.82,-.06];ax.scatter(*corner,marker='D',s=45,c='black',label='Existing doorway blind corner')
    observations=[]
    for rid in sorted(set(filter(None,[summary.get('crossing_request_id'),summary.get('through_request_id')]))):
        pose=run['events'][rid-1]['observation']['pose']
        ax.scatter(*pose[:2],marker='o',s=28,c='#146b9e',edgecolors='white',zorder=4)
        ax.annotate(f'C{rid}',pose[:2],xytext=(-33,4),textcoords='offset points',fontsize=9)
        observations.append({'request_id':rid,'world_pose':pose})
    coords.update(tested_corner_world_xy=corner,gate_observations=observations,
                  local_plot_bounds=[near_lo.tolist(),near_hi.tolist()],failed_gates=summary.get('failed_gates',[]))
    failed=summary.get('failed_gates',[])
    ax.set(xlim=(near_lo[0],near_hi[0]),ylim=(near_lo[1],near_hi[1]),title='Tested doorway and actual turn')
    handles,labels=ax.get_legend_handles_labels();handles.append(Patch(color='#737b85',alpha=.22));labels.append('Hospital collider XY bounds (proxy)')
    ax.legend(handles,labels,loc='upper left',bbox_to_anchor=(1.02,1),fontsize=9)
    fig.suptitle(f"Official Hospital E21 | {run['path'].name}\n{summary['decision']}",fontsize=13)
    reason=summary.get('failed_gate_detail','Failed gate: '+', '.join(failed))
    fig.text(.34,.06,reason+'\nNo stationary-human preflight or human-response data.',fontsize=10,color='#912626',va='bottom')
    fig.tight_layout(rect=[0,.14,1,.9]);fig.savefig(out/'trajectory_result.png',dpi=220,bbox_inches='tight');fig.savefig(out/'trajectory_result.pdf',bbox_inches='tight');plt.close(fig)
    write_json(out/'trajectory_coordinates.json',coords)


def analyze_baseline(source,out):
    out=validate_destination(out,[source]);hashes=evidence_hashes([source]);run=load_saved(source)
    selection=Path(run['cfg']['blind_corner']['selection_dir'])/'selected_route.json'
    inventory=ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'
    hashes.update(evidence_hashes([selection,inventory,Path(__file__)]))
    chosen=json.loads(selection.read_text())
    crossing,following=baseline_crossing(run['events'],run['cfg']['blind_corner']['baseline_gate'])
    end_index=following if following is not None else len(run['events'])-1;end=run['events'][end_index]['application']['sim_time']
    brightness=[];camera_fail=[]
    for e in run['events'][:end_index+1]:
        name=Path(e['rgb_observation_reference']).name
        b=json.loads((run['path']/'raw/brightness'/(name+'.json')).read_text());brightness.append({'request_id':e['request_id'],'pass':bright_pass(b,run['cfg']['bright_validation']),**b})
        diag=json.loads((run['path']/'raw/scene_observations'/(name+'.json')).read_text())
        if diag['camera_near_scene_collision_paths']:camera_fail.append(e['request_id'])
    contacts=[c for c in run['contacts'] if c['start_sim_time']<=end]
    checks={'complete_finite_chunks':run['complete'] and len(run['events'])==48,'intended_turn_and_exit':crossing is not None,
            'one_observation_after_reveal_zone':following is not None,'useful_rgb':all(b['pass'] for b in brightness) and not camera_fail,'no_relevant_nonfloor_collision':not contacts}
    failed=[k for k,v in checks.items() if not v];fresh=max(2,(crossing+1) if crossing is not None else 2)
    rows=request_metrics(run,SETTINGS);pair,detail=pair_metrics(run,rows,SETTINGS,fresh-1,fresh)
    summary={'decision':'BASELINE PASS - STATIONARY PREFLIGHT REQUIRED' if not failed else 'BASELINE FAIL - STOP BEFORE HUMAN PREFLIGHT',
        'baseline_pass':not failed,'checks':checks,'failed_gates':failed,'crossing_request_id':crossing+1 if crossing is not None else None,
        'through_request_id':end_index+1,'failed_brightness_ids':[b['request_id'] for b in brightness if not b['pass']],
        'camera_proximity_ids':camera_fail,'required_window_contacts':contacts,'all_contact_count':len(run['contacts']),
        'first_contact':run['contacts'][0] if run['contacts'] else None,'human_model_run_executed':False,'prediction_count':len(run['events']),
        'inspection_pair':[fresh-1,fresh],'note':'Baseline inspection pair is not a human reveal or human response.'}
    rules=run['cfg']['bright_validation']
    mean_fail=[b['request_id'] for b in brightness if b['mean']>rules['max_mean']]
    saturation_fail=[b['request_id'] for b in brightness if b['saturated_pixel_fraction']>rules['max_saturated_fraction']]
    summary.update(brightness_rules=rules,excess_mean_ids=mean_fail,excess_saturation_ids=saturation_fail,
                   required_window_end_sim_time=end,
                   observation_times=[e['observation']['sim_time'] for e in run['events']])
    summary['failed_gate_detail']='Failed gate: '+', '.join(failed)
    if mean_fail:
        summary['failed_gate_detail']+=f"\nRGB mean > {rules['max_mean']:g}: C{mean_fail[0]}–C{mean_fail[-1]} ({len(mean_fail)} observations)."
    if saturation_fail:
        summary['failed_gate_detail']+=f" Saturated pixels > {rules['max_saturated_fraction']:.0%}: C{saturation_fail[0]}–C{saturation_fail[-1]}."
    if run['contacts']:
        index=int(np.argmin(abs(run['times']-run['contacts'][0]['start_sim_time'])))
        summary['first_contact_nearest_robot_tick']={'x':float(run['xy'][index,0]),'y':float(run['xy'][index,1]),'sim_time':float(run['times'][index])}
    out.mkdir(parents=True,exist_ok=False)
    write_json(out/'summary.json',summary);write_json(out/'primary_pair.json',pair);write_json(out/'request_metrics.json',rows);write_json(out/'brightness.json',brightness)
    write_json(out/'metadata.json',{'source_sha256':hashes,**provenance(ROOT)})
    trajectory_plot(out,run,chosen,summary,inventory);verify_hashes(hashes);print(json.dumps(summary,indent=2));return summary


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
    analyze_baseline(a.run_dir,a.output_dir)


if __name__=='__main__':main()
