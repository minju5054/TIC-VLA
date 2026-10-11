"""Frozen offline RAW / RIGID / SE(2) GRAPH comparison on exactly two saved E16 runs."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import scipy
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_e16_handoff_audit import analyze,CONFIG as AUDIT_CONFIG,INVENTORY,distribution
from research.e16_se2_metrics import build_pair,evaluate
from research.se2_graph import rigid_align,solve_graph
from research.records import write_json,provenance
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination

CONFIG=ROOT/'configs/research/e16_se2_graph_offline.json'
METHODS=['RAW','RIGID','GRAPH','GRAPH_NO_CORR']
METRICS=['boundary_position_m','boundary_pose_heading_deg','first_segment_robot_heading_deg',
    'executed_to_future_tangent_deg','post_connection_first_segment_robot_heading_deg',
    'relative_translation_rmse_m','relative_rotation_rmse_deg','segment_length_distortion_rmse_m',
    'shape_turn_distortion_rmse_deg','derived_curve_heading_change_rmse_deg','pose_to_curve_heading_rmse_deg',
    'fresh_xy_rmse_m','far_fresh_xy_rmse_m','endpoint_displacement_m','near_heading_total_variation_deg',
    'near_heading_max_step_deg','near_curvature_change_rmse_rad_per_m','old_correspondence_xy_rmse_m']


def serial(x):
    if isinstance(x,np.ndarray):return serial(x.tolist())
    if isinstance(x,np.generic):return serial(x.item())
    if isinstance(x,float) and not np.isfinite(x):return None
    if isinstance(x,dict):return {k:serial(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [serial(v) for v in x]
    return x


def csv_save(path,rows):
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for r in rows:writer.writerow({k:json.dumps(serial(v),ensure_ascii=False) if isinstance(v,(dict,list,tuple,np.ndarray)) else serial(v) for k,v in r.items()})


def load_sources(config):
    expected={'A':'outputs/nova-e16-hospital-lights-baseline-20261010-01','B':'outputs/nova-e16-no-cart-15-repro-20261011-01'}
    if config['source_runs']!=expected:raise ValueError('Only the two authorized source runs are allowed')
    runs={k:load_saved(ROOT/v) for k,v in expected.items()}
    if runs['A']['cfg']!=runs['B']['cfg']:raise ValueError('Original source configs differ')
    audit={k:analyze(r,json.loads(AUDIT_CONFIG.read_text()),json.loads(INVENTORY.read_text())) for k,r in runs.items()}
    return runs,audit


def compare_pair(run,old_id,config,phase=None):
    pair=build_pair(run,old_id,config,phase);outputs={};metrics={};solvers={}
    if pair['eligibility']=='ELIGIBLE':
        f,b,t=pair['reference'],pair['boundary'],pair['times'];c=pair['correspondence']
        outputs={'RAW':f.copy(),'RIGID':rigid_align(f,b)}
        for method,use_corr in [('GRAPH',True),('GRAPH_NO_CORR',False)]:
            answer=solve_graph(f,b,t,c,pair['join_direction'],config,use_corr)
            outputs[method]=answer.pop('poses');solvers[method]=answer
        for method,x in outputs.items():
            m=evaluate(f,x,t,b,pair['executed_direction'],config)
            if m is not None:
                ids=c['indices'];m['old_correspondence_xy_rmse_m']=float(np.sqrt(np.mean(np.sum((x[ids,:2]-c['old_xy'])**2,axis=1)))) if len(ids) else None
            metrics[method]=m
    return dict(inputs=pair,outputs=outputs,metrics=metrics,solvers=solvers)


def summary(results,config):
    output={};eps=config['evaluation']['numerical_comparison_epsilon']
    for label in ['A','B','ALL']:
        output[label]={}
        for subset in ['all','nonbootstrap','startup']:
            cases=[d for d in results.values() if (label=='ALL' or d['run']==label) and (subset=='all' or d['inputs']['bootstrap']==(subset=='startup'))]
            eligible=[d for d in cases if d['inputs']['eligibility']=='ELIGIBLE']
            stats={m:{key:distribution([d['metrics'].get(m,{}).get(key) for d in eligible if d['metrics'].get(m) is not None]) for key in METRICS} for m in METHODS}
            # Explicitly retain nonconverged outputs above; convergence-filtered
            # paired trade-offs below state their smaller valid sample counts.
            paired=[d for d in eligible if d['solvers']['GRAPH']['success']]
            trade={}
            for key in ['first_segment_robot_heading_deg','executed_to_future_tangent_deg','far_fresh_xy_rmse_m','relative_translation_rmse_m','near_curvature_change_rmse_rad_per_m']:
                delta=[d['metrics']['GRAPH'][key]-d['metrics']['RIGID'][key] for d in paired if d['metrics']['GRAPH'][key] is not None and d['metrics']['RIGID'][key] is not None]
                trade[key]=dict(distribution=distribution(delta),better=int(sum(v < -eps for v in delta)),worse=int(sum(v > eps for v in delta)),tie=int(sum(abs(v)<=eps for v in delta)))
            ablated=[d for d in paired if d['solvers']['GRAPH_NO_CORR']['success']]
            ablation={key:distribution([d['metrics']['GRAPH'][key]-d['metrics']['GRAPH_NO_CORR'][key] for d in ablated if d['metrics']['GRAPH'][key] is not None and d['metrics']['GRAPH_NO_CORR'][key] is not None]) for key in METRICS}
            dominates=[d for d in paired if all(d['metrics']['GRAPH'][k] is not None and d['metrics']['RIGID'][k] is not None for k in ['executed_to_future_tangent_deg','far_fresh_xy_rmse_m'])]
            both=sum(all(d['metrics']['GRAPH'][k]<d['metrics']['RIGID'][k]-eps for k in ['executed_to_future_tangent_deg','far_fresh_xy_rmse_m']) for d in dominates)
            output[label][subset]=dict(source_pair_count=len(cases),eligible_count=len(eligible),ineligible_count=len(cases)-len(eligible),
                exclusions=dict(Counter(d['inputs']['exclusion_reason'] for d in cases if d['inputs']['eligibility']!='ELIGIBLE')),
                method_statistics=stats,graph_minus_rigid=trade,graph_minus_no_correspondence=ablation,
                graph_better_heading_and_far=dict(count=both,valid_count=len(dominates),note='Two-metric comparison only; additional relative/shape deformation is not free.'),
                solver={m:dict(success=sum(d['solvers'][m]['success'] for d in eligible),failed=sum(not d['solvers'][m]['success'] for d in eligible),
                    runtime_ms=distribution([d['solvers'][m]['runtime_ms'] for d in eligible]),nfev=distribution([d['solvers'][m]['nfev'] for d in eligible]),
                    numerical_failures=sum(d['solvers'][m]['numerical_failure'] for d in eligible)) for m in ['GRAPH','GRAPH_NO_CORR']})
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['selected','all'],required=True)
    p.add_argument('--freeze',type=Path,required=True);p.add_argument('--selected-dir',type=Path);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    receipt=json.loads(a.freeze.read_text());verify_hashes(receipt['source_sha256']);verify_hashes(receipt['implementation_sha256'])
    cfg=json.loads(CONFIG.read_text());runs,audit=load_sources(cfg)
    if a.phase=='all':
        if a.selected_dir is None:raise ValueError('Review selected-pair output before the full batch')
        selected=json.loads((a.selected_dir/'metadata.json').read_text())
        if selected['phase']!='selected' or selected['config']!=cfg or selected['implementation_sha256']!=receipt['implementation_sha256']:
            raise ValueError('Selected formulation differs from frozen full batch')
    paths=[r['path'] for r in runs.values()];out=validate_destination(a.output_dir,paths)
    hashes=evidence_hashes([*paths,CONFIG,a.freeze,INVENTORY,AUDIT_CONFIG,*[ROOT/p for p in ['research/se2_graph.py','research/e16_se2_metrics.py','research/compare_e16_se2_graph.py','research/plot_e16_se2_graph.py']]])
    out.mkdir(parents=True);(out/'pairs').mkdir();(out/'figures').mkdir()
    specs=cfg['selected_pairs'] if a.phase=='selected' else [(k,i) for k in runs for i in range(1,48)]
    results={};rows=[];accounting=[]
    for k,i in specs:
        name=f'{k}_C{i:02d}_C{i+1:02d}';d=compare_pair(runs[k],i,cfg,audit[k]['rows'][i-1]['phase']);d['run']=k;d['pair_id']=name;results[name]=d
        write_json(out/'pairs'/f'{name}.json',serial(d))
        inp=d['inputs'];account=dict(pair_id=name,run=k,old_request_id=i,fresh_request_id=i+1,eligibility=inp['eligibility'],exclusion_reason=inp['exclusion_reason'],
            bootstrap=i==1,phase=inp['phase'],valid_se2_node_count=inp['valid_node_count'],correspondence_count=inp['correspondence_count'],
            join_disabled_reason=inp.get('join_disabled_reason'),graph_status=d['solvers'].get('GRAPH',{}).get('status'),graph_success=d['solvers'].get('GRAPH',{}).get('success'))
        accounting.append(account)
        for method in METHODS:
            m=d['metrics'].get(method);row=dict(**account,method=method,solver_success=d['solvers'].get(method,{}).get('success'),
                metric_status='VALID_OUTPUT' if m is not None else 'UNAVAILABLE',**(m if m is not None else dict.fromkeys(METRICS)))
            rows.append(row)
            if d['outputs'].get(method) is not None:
                xyz=d['outputs'][method];csv_save(out/'pairs'/f'{name}_{method}.csv',[dict(derived_node=j,nominal_sim_time=float(t),world_x_m=float(v[0]),world_y_m=float(v[1]),derived_pose_heading_rad=float(v[2]),native_yaw=None) for j,(t,v) in enumerate(zip(inp['times'],xyz))])
        print(name,inp['eligibility'],d['solvers'].get('GRAPH',{}).get('success'),flush=True)
    if len(results)!=(5 if a.phase=='selected' else 94):raise ValueError('Pair accounting incomplete')
    csv_save(out/'pair_accounting.csv',accounting);csv_save(out/'method_metrics.csv',rows);write_json(out/'method_metrics.json',serial(rows))
    write_json(out/'summary.json',serial(summary(results,cfg)))
    from research.plot_e16_se2_graph import figures
    figures(out/'figures',results,cfg,a.phase)
    verify_hashes(hashes)
    if evidence_hashes(paths)!={k:v for k,v in hashes.items() if any(s in Path(k).parents for s in paths)}:raise ValueError('Raw source inventory changed')
    write_json(out/'metadata.json',dict(phase=a.phase,config=cfg,source_sha256=hashes,implementation_sha256=receipt['implementation_sha256'],
        source_files_unchanged=True,analysis_provenance=provenance(ROOT),python=sys.version,numpy=np.__version__,scipy=scipy.__version__,
        source_provenance={k:dict(metadata=json.loads((r['path']/'metadata.json').read_text()),inference_runtime=json.loads((r['path']/'inference-runtime.json').read_text())) for k,r in runs.items()},
        command=sys.argv,model_calls=0,simulation_launches=0,independent_runs=2,source_requests=96,total_source_adjacent_pairs=94,evaluated_pair_count=len(results)))
    print(str(out),flush=True)


if __name__=='__main__':main()
