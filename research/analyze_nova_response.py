"""Saved-only exact-input Jackal vs Nova response comparison and Stage A gate."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.nova_response import extract_schedule,read_csv,trace,response_metrics
from research.hospital_episode import digest
from research.records import write_json,provenance
from research.analyze_hospital_turn import nonfloor_contacts


def contacts(run):
    path=Path(run);cfg=json.loads((path/"metadata.json").read_text())["config"]
    return nonfloor_contacts([json.loads(s) for s in (path/"raw/robot_contacts.jsonl").read_text().splitlines()],cfg["robot"]["prim"])


def analyze(source,a0,direct,slew,output):
    paths=list(map(lambda p:Path(p).resolve(),[source,a0,direct,slew]));source,a0,direct,slew=paths
    output=Path(output).resolve()
    if output.exists(): raise FileExistsError(output)
    if any(p in output.parents or output in p.parents for p in paths): raise ValueError("Output must be separate from sources")
    hashes={str(f):digest(f) for p in paths for f in p.rglob('*') if f.is_file()}
    schedule=extract_schedule(source)
    before,last=schedule["boundary_tick"],schedule["end_tick_inclusive"]
    initial=json.loads((a0/"summary.json").read_text());cfg=json.loads((a0/"metadata.json").read_text())["config"]
    rules=cfg["nova_validation"]; noise=initial["stationary_max_yaw_excursion_rad"]
    results=[];data_sets=[]
    for label,path,embodiment in [('Jackal saved',source,'jackal'),('Nova direct',direct,'nova'),('Nova official-slew',slew,'nova')]:
        rows=read_csv(path/'robot_state.csv');data=trace(rows,embodiment)
        if embodiment=='nova':
            if json.loads((path/'summary.json').read_text())["status"]!='PASS':raise ValueError("Replay did not complete")
            stored=json.loads((path/'replay_schedule.json').read_text())
            if stored!=schedule:raise ValueError("Replay used different source schedule")
            if len(rows)!=last+1:raise ValueError("Replay tick count mismatch")
            for row,cmd in zip(rows[1:],schedule['commands']):
                if (int(row['source_tick'])!=cmd['source_tick'] or float(row['target_v'])!=cmd['target_v']
                        or float(row['target_w'])!=cmd['target_w']):raise ValueError("Replay altered saved target schedule")
            np.testing.assert_allclose(data['time'],np.array([float(r['sim_time']) for r in read_csv(source/'robot_state.csv')[:last+1]]),atol=1e-7,rtol=0)
        m=response_metrics(data,before+1,last)
        hits=[c for c in contacts(path) if c['start_sim_time']<=data['time'][last] and (c['end_sim_time'] is None or c['end_sim_time']>=data['time'][before])]
        row={"variant":label,**m,"nonfloor_contact_span_count":len(hits),"above_stationary_noise":m['actual_delta_yaw_rad']>rules['noise_multiple']*noise+rules['numerical_zero_rad']}
        results.append(row);data_sets.append((label,data))
    a0_hits=contacts(a0)
    valid=bool(initial['a0_valid'] and not a0_hits and all(r['above_stationary_noise'] and r['pose_sign_agreement_fraction']>.5
                and r['nonfloor_contact_span_count']==0 for r in results[1:]))
    summary={"stage_a_decision":"NOVA CARTER ANGULAR RESPONSE VALIDATED" if valid else "NOVA CARTER ANGULAR RESPONSE NOT VALIDATED",
        "a0":initial,"a0_nonfloor_contact_spans":a0_hits,"stationary_noise_rad":noise,
        "noise_gate_rad":rules['noise_multiple']*noise+rules['numerical_zero_rad'],"comparison":results,
        "source_schedule":{k:v for k,v in schedule.items() if k!='commands'},
        "interpretation_limit":"Same command input history and scene/start; embodiment plus wheel conversion changes. Actual state histories diverge. Ratios are descriptive, not ideal tracking requirements."}
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'metadata.json',{"source_files_sha256":hashes,**provenance(ROOT)})
    write_json(output/'summary.json',summary);write_json(output/'response_metrics.json',results)
    with (output/'response_metrics.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figs=output/'figures';figs.mkdir()
    fig,axes=plt.subplots(3,2,figsize=(13,11))
    for axpair,(label,data) in zip(axes,data_sets):
        sl=slice(before,last+1);t=data['time'][sl]-data['time'][before];dt=np.diff(t)
        target=data['target_w'][before+1:last+1];applied=data['applied_w'][before+1:last+1]
        ax=axpair[0];ax.step(t[1:],target,where='pre',label='target w');ax.step(t[1:],applied,where='pre',label='applied w',ls='--')
        ax.plot(t[1:],data['omega'][before+1:last+1],label='physics body yaw rate')
        ax.plot(t[1:],np.diff(data['yaw'][sl])/dt,label='pose derivative',alpha=.7)
        ax.set(title=label,ylabel='rad/s',xlabel='Time since primary boundary [s]')
        ax=axpair[1];ax.plot(t,np.r_[0,np.cumsum(target*dt)],label='integral target')
        ax.plot(t,np.r_[0,np.cumsum(applied*dt)],label='integral applied',ls='--')
        ax.plot(t,data['yaw'][sl]-data['yaw'][before],label='measured yaw change')
        ax.set(title=label,ylabel='rad',xlabel='Time since primary boundary [s]')
        for ax in axpair:ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(figs/'commanded_measured_response.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5))
    for label,data in data_sets:
        ax.plot(data['time'][before:last+1]-data['time'][before],np.degrees(data['yaw'][before:last+1]-data['yaw'][before]),label=label)
    ax.set(xlabel='C4-C7 elapsed time [s]',ylabel='Actual yaw change [deg]',title='Identical saved target schedule; measured response')
    ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(figs/'embodiment_comparison.png',dpi=170);plt.close(fig)
    if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError("Source changed during analysis")
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['jackal-run','a0-run','direct-run','slew-run','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args();r=analyze(a.jackal_run,a.a0_run,a.direct_run,a.slew_run,a.output_dir)
    print(r['stage_a_decision']);print(json.dumps(r['comparison'],indent=2))


if __name__=='__main__':main()
