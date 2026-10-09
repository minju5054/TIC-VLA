"""Saved bright baseline gate, using unchanged Nova turn/execution tests."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_nova_hospital import analyze as analyze_turn
from research.analyze_nova_dynamic_handoff import load_saved,request_metrics,pair_metrics
from research.records import write_json
from research.nova_bright import bright_pass

SETTINGS={'robot_radius_m':.6069825421953869,'human_radius_m':.25,'native_waypoint_dt':.1,
          'minimum_tangent_displacement_m':1e-6,'numerical_position_tolerance_m':1e-6,'executed_tangent_window_s':.1}


def analyze(source,output):
    source,output=Path(source).resolve(),Path(output).resolve()
    if source==output or source in output.parents or output in source.parents:raise ValueError('Separate output required')
    output.mkdir(parents=True,exist_ok=False)
    turn=analyze_turn(source,output/'turn');run=load_saved(source)
    brightness=[{**json.loads((source/'raw/brightness'/(Path(e['rgb_observation_reference']).name+'.json')).read_text()),'request_id':e['request_id']} for e in run['events']]
    groups=turn['actual_left_interval_groups_by_fresh_id']
    primary=groups[0][:2] if groups else [None,None]
    # Actual interval groups are consecutive FRESH IDs; a pair ending at the
    # first qualifying interval supplies a baseline inspection window only.
    old,fresh=(primary[0]-1,primary[0]) if groups else (1,2)
    end=min(len(brightness),primary[-1]+1) if groups else len(brightness)
    clean=all(bright_pass(x,run['cfg']['bright_validation']) for x in brightness[:end])
    from research.analyze_nova_response import contacts
    early=[s for s in contacts(source) if s['start_sim_time']<=run['events'][end-1]['application']['sim_time']]
    passed=bool(len(run['events'])==48 and turn['stage_b_decision']=='NOVA CARTER E16 TURN EXECUTION VALIDATED' and clean and not early)
    rows=request_metrics(run,SETTINGS);pair,detail=pair_metrics(run,rows,SETTINGS,old,fresh)
    write_json(output/'primary_pair.json',pair);write_json(output/'request_metrics.json',rows)
    write_json(output/'details.json',detail);write_json(output/'brightness.json',brightness)
    summary={'bright_baseline_pass':passed,'decision':'BRIGHT BASELINE PASS' if passed else 'INSUFFICIENT EVIDENCE',
        'source_run':str(source),'prediction_count':len(run['events']),'turn_validation':turn['stage_b_decision'],
        'qualifying_actual_turn_groups':groups,'inspection_pair':[old,fresh],
        'gate_request_ids':list(range(1,end+1)),'bright_gate_pass':clean,'early_contact_spans':early,
        'failed_brightness_ids':[x['request_id'] for x in brightness if not bright_pass(x,run['cfg']['bright_validation'])],
        'minimum_mean':min(x['mean'] for x in brightness),'minimum_p95':min(x['p95'] for x in brightness),
        'note':'Inspection pair from measured turn gate, not a human reveal event; baseline failure stops before stationary-human run.'}
    write_json(output/'summary.json',summary);print(json.dumps(summary,indent=2));return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();analyze(a.run_dir,a.output_dir)
