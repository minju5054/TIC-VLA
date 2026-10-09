"""Saved-only brightness/contact audit figures; no sensor or model execution."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.hospital_episode import digest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['run-dir','analysis-dir','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args();source=Path(a.run_dir).resolve();analysis=Path(a.analysis_dir).resolve();out=Path(a.output_dir).resolve()
    if any(x==out or x in out.parents or out in x.parents for x in [source,analysis]):raise ValueError('Separate output required')
    hashes={str(p):digest(p) for folder in [source,analysis] for p in folder.rglob('*') if p.is_file()}
    out.mkdir(parents=True,exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    rows=json.loads((analysis/'brightness.json').read_text());summary=json.loads((analysis/'summary.json').read_text())
    x=np.array([r['request_id'] for r in rows]);t=np.array([r['observation_sim_time'] for r in rows])
    contacts=summary['early_contact_spans'];onset=min(s['start_sim_time'] for s in contacts) if contacts else None
    fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
    for key in ['mean','p05','p50','p95','p99']:axes[0].plot(x,[r[key] for r in rows],'.-',label=key)
    axes[0].set_ylabel('Luminance [0–255]');axes[0].legend(ncol=5);axes[0].axhline(100,color='gray',ls=':',label='p95 minimum')
    axes[1].plot(x,[r['std'] for r in rows],'.-',label='Luminance std');axes[1].axhline(15,color='red',ls='--',label='std minimum')
    axes[1].set_ylabel('Standard deviation');axes[1].legend()
    axes[2].plot(x,[r['dark_pixel_fraction'] for r in rows],'.-',label='Dark fraction (<15)')
    axes[2].plot(x,[r['saturated_pixel_fraction'] for r in rows],'.-',label='Any channel >=250 fraction')
    axes[2].axhline(.35,color='red',ls='--',label='Dark fraction maximum');axes[2].set(ylabel='Pixel fraction',xlabel='Request ID');axes[2].legend()
    for ax in axes:
        if onset is not None:ax.axvline(np.interp(onset,t,x),color='black',ls='--',label='First non-floor contact')
        ax.grid(alpha=.25)
    axes[0].set_title('Bright baseline: fixed profile; vertical black line = first non-floor contact')
    fig.tight_layout();fig.savefig(out/'brightness_contact_timeline.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(16,7))
    for rid,ax in zip([1,12,16,18,19,48],axes.flat):
        ax.imshow(Image.open(source/f'diagnostics/rgb_{rid:06d}.png'));ax.set_title(f"Original C{rid}: mean {rows[rid-1]['mean']:.1f}, p95 {rows[rid-1]['p95']:.1f}");ax.axis('off')
    fig.tight_layout();fig.savefig(out/'original_rgb_context.png',dpi=140);plt.close(fig)
    pair=json.loads((analysis/'primary_pair.json').read_text())
    detail=json.loads((analysis/'details.json').read_text())
    aligned=detail['alignment'];delta=np.array(detail['fresh_frame_delta'])
    fresh_id=pair['fresh_request_id'];old_id=pair['old_request_id']
    horizon=np.array(aligned['absolute_times'])-pair['fresh_context']['observation_sim_time']
    fig,ax=plt.subplots(figsize=(10,4))
    ax.plot(horizon,np.linalg.norm(aligned['delta'],axis=1),label='Euclidean revision')
    ax.plot(horizon,delta[:,0],label='Signed forward (FRESH frame)')
    ax.plot(horizon,delta[:,1],label='Signed left (FRESH frame)')
    ax.set(xlabel='Nominal seconds after FRESH observation',ylabel='Revision [m]',
        title=f'No-human inspection C{old_id} → C{fresh_id}: RMSE {pair["overlap_rmse_m"]:.3f} m')
    ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'inspection_overlap.png',dpi=170);plt.close(fig)
    from research.analyze_chunk_geometry import project_world
    from research.nova_response import read_csv
    events=[json.loads((source/f'raw/requests/request_{rid:06d}.json').read_text()) for rid in [old_id,fresh_id]]
    worlds=[project_world(np.load(source/f'raw/requests/request_{e["request_id"]:06d}.npy',allow_pickle=False),e['agent_pose_at_observation']) for e in events]
    robot=read_csv(source/'robot_state.csv');rt=np.array([float(r['sim_time']) for r in robot]);xy=np.array([[float(r['x']),float(r['y'])] for r in robot])
    seam=pair['raw_seam'];boundary=np.array(events[1]['application']['position'][:2]);expected=np.array(seam['fresh_expected_xy'])
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for w,color,label in zip(worlds,['tab:orange','tab:green'],['OLD own-observation projection','FRESH own-observation projection']):axes[0].plot(*w.T,'.-',color=color,label=label)
    axes[0].plot(*xy[(rt>=events[0]['observation']['sim_time'])&(rt<=events[1]['application']['sim_time'])].T,color='cyan',label='Measured robot segment')
    for e,color in zip(events,['tab:orange','tab:green']):axes[0].scatter(*e['agent_pose_at_observation'][:2],marker='x',s=70,color=color)
    axes[0].scatter(*boundary,color='black',marker='s',label='Switch B')
    start=seam['executed_window_start_sim_time'];stop=seam['executed_window_end_sim_time']
    window_times=np.r_[start,rt[(rt>start)&(rt<stop)],stop]
    executed=np.column_stack([np.interp(window_times,rt,xy[:,i]) for i in range(2)])
    axes[1].plot(*executed.T,'o-',color='cyan',label='Measured last 0.1 s (clipped)')
    axes[1].plot(*np.vstack([events[1]['agent_pose_at_observation'][:2],worlds[1][:3]]).T,'.-',color='tab:green',label='FRESH + derived observation anchor')
    axes[1].plot([boundary[0],expected[0]],[boundary[1],expected[1]],'r-',lw=3,label=f'RAW gap {pair["raw_seam"]["raw_boundary_position_gap_m"]:.4f} m')
    axes[1].scatter(*boundary,color='black',marker='s');axes[1].scatter(*expected,color='red',marker='x')
    for ax in axes:ax.set(xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend(fontsize=8)
    axes[0].set_title('Baseline turn inspection; no human/reveal')
    axes[1].set_title('RAW switch; no transport correction')
    fig.tight_layout();fig.savefig(out/'inspection_world_raw_seam.png',dpi=170);plt.close(fig)
    if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Source mutated')
    write_json(out/'metadata.json',{'source_sha256':hashes,'original_rgb_unmodified':True,**provenance(ROOT)})


if __name__=='__main__':main()
