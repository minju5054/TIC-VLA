"""Saved-result review only: common-sample statistics and unsupported/short-segment diagnostics.

Does not rerun or change the frozen optimizer, weights, selection or source data.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json
from research.reveal_window import evidence_hashes,verify_hashes
from research.compare_e16_se2_graph import serial,csv_save,METHODS
from research.analyze_e16_handoff_audit import distribution


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--analysis-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    meta=json.loads((a.analysis_dir/'metadata.json').read_text());verify_hashes(meta['source_sha256'])
    hashes=evidence_hashes([a.analysis_dir,Path(__file__)]);out=a.output_dir;out.mkdir(parents=True)
    ds=[json.loads(f.read_text()) for f in sorted((a.analysis_dir/'pairs').glob('*.json'))]
    if len(ds)!=94:raise ValueError('Complete 94-pair comparison required')
    eligible=[d for d in ds if d['inputs']['eligibility']=='ELIGIBLE'];normal=[d for d in eligible if not d['inputs']['bootstrap']]
    common=[d for d in normal if all(d['metrics'][m]['executed_to_future_tangent_deg'] is not None for m in METHODS)]
    stats={m:{k:distribution([d['metrics'][m][k] for d in common]) for k in ['executed_to_future_tangent_deg','far_fresh_xy_rmse_m','relative_translation_rmse_m']} for m in METHODS}
    reasons=Counter(diag['reason'] for d in ds for diag in d['inputs'].get('tangent_diagnostics',[]) if not diag['valid'])
    flags=[]
    for d in ds:
        i=d['inputs'];m=d['metrics'].get('GRAPH');rigid=d['metrics'].get('RIGID');no=d['metrics'].get('GRAPH_NO_CORR')
        flags.append(dict(pair=d['pair_id'],eligibility=i['eligibility'],bootstrap=i['bootstrap'],exclusion_reason=i['exclusion_reason'],
            graph_first_segment_m=m['first_segment_length_m'] if m else None,graph_first_segment_valid=m['first_segment_valid'] if m else None,
            graph_worse_join_than_rigid=(m['executed_to_future_tangent_deg']>rigid['executed_to_future_tangent_deg']) if m and m['executed_to_future_tangent_deg'] is not None else None,
            graph_more_near_heading_variation_than_raw=(m['near_heading_max_step_deg']>d['metrics']['RAW']['near_heading_max_step_deg']) if m else None,
            correspondence_max_xy_effect_m=float(np.linalg.norm(np.array(d['outputs']['GRAPH'])[:,:2]-np.array(d['outputs']['GRAPH_NO_CORR'])[:,:2],axis=1).max()) if m and no else None))
    csv_save(out/'pair_limitations.csv',flags)
    write_json(out/'common_sample_statistics.json',serial(dict(common_nonbootstrap_pairs=[d['pair_id'] for d in common],statistics=stats,invalid_node_reason_counts=dict(reasons))))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # Display the omitted source geometry rather than inventing an optimized path.
    d=next(d for d in ds if d['pair_id']=='A_C13_C14');i=d['inputs'];ref=np.array(i['reference'],float);b=np.array(i['boundary']);old=np.array(i['old_remaining_xy'])
    invalid=[j for j,x in enumerate(i['tangent_diagnostics']) if not x['valid']]
    fig,axes=plt.subplots(1,2,figsize=(13,6));hist=np.array(i['executed_xy']);times=np.array(i['executed_times'])
    for ax in axes:
        ax.plot(*hist[times>=i['switch_sim_time']-1].T,c='#298541',lw=2.5,label='Measured committed execution')
        ax.plot(*ref[:,:2].T,'.-',c='#333333',label='Original FRESH XY (not corrected)')
        ax.plot(*old.T,':',c='#999999',label='OLD remaining prediction')
        ax.scatter(*b[:2],marker='*',s=110,c='#298541',label='Measured switch B')
        ax.scatter(*i['fresh_observation_pose'][:2],marker='o',facecolors='none',edgecolors='black',s=55,label='FRESH observation origin')
        ax.scatter(ref[invalid,0],ref[invalid,1],s=100,marker='x',c='red',label='Unsupported derived tangent')
        ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]');ax.grid(alpha=.2)
    j=invalid[0];v=i['tangent_diagnostics'][j];t=np.array(i['times']);q=np.r_[v['window_start'],t[(t>v['window_start'])&(t<v['window_end'])],v['window_end']]
    xy=np.column_stack([np.interp(q,t,ref[:,k]) for k in [0,1]])
    axes[1].plot(*xy.T,'o-',c='#b42630',lw=2,label='Tangent support window');lo=xy.min(0)-.06;hi=xy.max(0)+.06
    axes[1].set(xlim=[lo[0],hi[0]],ylim=[lo[1],hi[1]],title=f"Node {j}: chord {v['chord_m']:.4f}m / arc {v['arc_m']:.4f}m\nCoherence {v['coherence']:.4f} < frozen 0.25")
    axes[0].legend(fontsize=7);axes[1].legend(fontsize=7);fig.suptitle('A C13 -> C14: INELIGIBLE, XY data remains valid\nRIGID/GRAPH unavailable under the all-headings-valid formulation; no fabricated yaw',fontsize=12)
    fig.tight_layout();fig.savefig(out/'unsupported_selected_geometry.png',dpi=170,bbox_inches='tight');fig.savefig(out/'unsupported_selected_geometry.pdf',bbox_inches='tight');plt.close(fig)
    # One figure book is the primary PDF deliverable. Individual scientific PDFs
    # remain available; substitute the informative unsupported-case panel.
    files=[out/'unsupported_selected_geometry.pdf' if f.name=='A_C13_C14.pdf' else f for f in sorted((a.analysis_dir/'figures').glob('*.pdf'))]
    subprocess.run(['pdfunite',*[str(f) for f in files],str(out/'comparison_figures.pdf')],check=True)
    verify_hashes(hashes)
    write_json(out/'metadata.json',dict(numerical_analysis=str(a.analysis_dir.resolve()),source_sha256=hashes,numeric_results_unchanged=True,
        solver_runs=0,model_calls=0,simulation_launches=0,figure_book_pages=len(files),note='Supplemental saved-only review; frozen method unchanged.'))
    print(str(out))


if __name__=='__main__':main()
