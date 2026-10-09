"""Saved-only Nova E16 prediction, accepted-command response and actual turn."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.nova_evidence import load_nova
from research.nova_response import trace,response_metrics
from research.analyze_nova_response import contacts
from research.analyze_hospital_turn import characterize
from research.hospital_turn_plots import plot_all
from research.hospital_episode import digest
from research.records import write_json,provenance


def accepted_responses(run):
    rows=run['robot'];data=trace(rows,'nova');spans=contacts(run['path']);result=[]
    for e in run['events']:
        rid=e['request_id'];indices=[i for i,r in enumerate(rows) if r['active_control_source_request_id']==str(rid)]
        if not indices: continue
        a,b=indices[0],indices[-1]
        if indices!=list(range(a,b+1)) or int(rows[a-1]['tick'])!=e['application']['tick']:
            raise ValueError('Accepted interval is discontinuous or missing application boundary')
        m=response_metrics(data,a,b)
        hits=[s for s in spans if s['start_sim_time']<=data['time'][b] and (s['end_sim_time'] is None or s['end_sim_time']>=data['time'][a-1])]
        result.append({'request_id':rid,'target_w':e['controller_command'][1],
            'application_sim_time':e['application']['sim_time'],'end_sim_time':float(data['time'][b]),
            'hold_duration_s':m['duration_s'],**m,'nonfloor_contact_span_count':len(hits)})
    return result,data


def analyze(run_dir,output_dir):
    source,output=Path(run_dir).resolve(),Path(output_dir).resolve()
    if output.exists():raise FileExistsError(output)
    if source in output.parents or output in source.parents:raise ValueError('Separate output required')
    hashes={str(p):digest(p) for p in source.rglob('*') if p.is_file()}
    complete=(source/'summary.json').is_file()
    run=load_nova(source,require_complete=complete)
    if not complete and not (source/'failure.json').is_file():raise ValueError('Neither completed nor recorded failure')
    intervals,data=accepted_responses(run)
    # Compatibility view for the existing pure geometry characterization only.
    # The source CSV retains distinct target/applied/two-wheel fields unchanged.
    geometry_run={**run,'robot':[{**r,'command_v':r['target_v'],'command_w':r['target_w']} for r in run['robot']]}
    diags=[json.loads((source/'raw/scene_observations'/(Path(e['rgb_observation_reference']).name+'.json')).read_text()) for e in run['events']]
    raw_contacts=[json.loads(line) for line in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    metrics,worlds,observation_intervals,summary,motion=characterize(geometry_run,diags,raw_contacts)
    first=summary['phases']['first_stable_left_request']
    actual=bool(summary['actual_left_interval_groups_by_fresh_id'])
    summary['stage_b_decision']=('NOVA CARTER E16 TURN EXECUTION VALIDATED' if first and actual else
        'NOVA CARTER LEFT-TURN PREDICTION OBSERVED; EXECUTION NOT VALIDATED' if first else
        'NO VALID NOVA-CARTER LEFT-TURN PREDICTION OBSERVED')
    summary.pop('stage_2_decision')
    summary['combined_interpretation']=('POSITIVE EVIDENCE THAT JACKAL/CUSTOM DOWNSTREAM EXECUTION WAS A MAJOR CONFOUND'
        if first and actual else 'ACTUATION IS NOT THE ONLY LIMITATION')
    summary['original_run_status']='PASS' if complete else 'FAIL/INCOMPLETE'
    summary['run_failure']=None if complete else json.loads((source/'failure.json').read_text())
    summary['source_run']=str(source)
    summary['measured_max_abs_yaw_rate_rad_s']=float(np.max(np.abs(data['omega'])))
    summary['whole_run_response']=response_metrics(data,1,len(data['time'])-1)
    summary['accepted_command_responses']=intervals
    summary['continuous_action_ticks']=run['inside_action_ticks']
    summary['prediction_note']='Geometry rules reused; Stage B asks actual left execution of own positive prediction, without requiring an earlier forward-only prediction label'
    summary['first_degraded_request_id']=next((r['request_id'] for r in metrics if r['degraded_near_black']),None)
    if (source/'capture_failure.json').is_file():summary['rejected_capture']=json.loads((source/'capture_failure.json').read_text())
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'metadata.json',{'source_files_sha256':hashes,**provenance(ROOT),
        'all_research_code_sha256':{str(p.relative_to(ROOT)):digest(p) for p in (ROOT/'research').rglob('*.py')}})
    write_json(output/'summary.json',summary);write_json(output/'request_metrics.json',metrics);write_json(output/'request_response.json',intervals)
    for name,rows in [('request_metrics',metrics),('request_response',intervals)]:
        with (output/(name+'.csv')).open('x') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    derived=output/'derived';derived.mkdir()
    write_json(derived/'observation_intervals.json',observation_intervals)
    write_json(derived/'transforms.json',[{'request_id':e['request_id'],'observation_pose':e['agent_pose_at_observation'],
        'transform':'world = R(observation_yaw) @ native_forward_left + observation_xy; metres; no native yaw'} for e in run['events']])
    for e,world in zip(run['events'],worlds):
        with (derived/f"request_{e['request_id']:06d}_world.npy").open('xb') as f:np.save(f,world,allow_pickle=False)
    plot_all(output,run,metrics,worlds,summary,motion)
    response_plots(output,data,intervals)
    if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Raw evidence changed')
    return summary


def response_plots(output,data,intervals):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=data['time'];dt=np.diff(t)
    fig,axes=plt.subplots(2,1,figsize=(12,8))
    ax=axes[0];ax.step(t[1:],data['target_w'][1:],where='pre',label='target = applied w')
    ax.plot(t[1:],data['omega'][1:],label='measured physics body yaw rate',alpha=.75)
    ax.plot(t[1:],np.diff(data['yaw'])/dt,label='measured pose derivative',alpha=.6)
    ax.set(ylabel='rad/s',xlabel='Simulation time [s]');ax.legend();ax.grid(alpha=.3)
    ax=axes[1];ax.plot(t,np.r_[0,np.cumsum(data['target_w'][1:]*dt)],label='integral target/applied w')
    ax.plot(t,data['yaw']-data['yaw'][0],label='actual unwrapped yaw change')
    ax.set(ylabel='rad',xlabel='Simulation time [s]');ax.legend();ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(output/'figures/target_measured_yaw_response.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,4))
    ax.plot([r['request_id'] for r in intervals],[r['response_ratio_target'] for r in intervals],'.-')
    ax.axhline(1,color='gray',ls='--');ax.set(xlabel='Accepted request ID',ylabel='Measured delta yaw / integral target w',
        title='Own accepted-command interval response; descriptive ratio, not an ideal-source requirement')
    ax.grid(alpha=.3);fig.tight_layout();fig.savefig(output/'figures/request_response_ratio.png',dpi=170);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',required=True);p.add_argument('--output-dir',required=True)
    a=p.parse_args();s=analyze(a.run_dir,a.output_dir);print(s['stage_b_decision']);print(s['combined_interpretation'])


if __name__=='__main__':main()
