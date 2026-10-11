"""Saved-only bounded preflight report and existing-viewer adapter; never inference."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.analyze_nova_dynamic_handoff import load_saved
from research.cart_geometry import outline
from research.hospital_spatial_gate import wall_inventory


def report(source,out):
    protocol=json.loads((source/'protocol.json').read_text());summary=json.loads((source/'summary.json').read_text())
    candidates=json.loads((source/'derived/candidates.json').read_text())
    if summary['qualified_count']!=0:raise ValueError('This stop report is only for a failed Stage A')
    hashes=evidence_hashes([source]);hashes.update(protocol['source_evidence_sha256']);verify_hashes(hashes)
    out=validate_destination(out,[source,protocol['baseline_run']]);out.mkdir(parents=True);(out/'figures').mkdir();(out/'derived').mkdir()
    run=load_saved(Path(protocol['baseline_run']));radius=protocol['robot_radius_m']
    diagnostic=candidates[-1]  # Largest fixed candidate ID, not a replacement selected pair.
    selected=diagnostic['visibility'];cid=diagnostic['candidate_id'];cart=diagnostic['cart']
    fid=selected['first_clear_request_id'];oid=selected['old_request_id'];temporal=selected['temporal_at_first_clear']
    rows=[]
    for c in candidates:
        v=c['visibility'];t=v['temporal_at_first_clear']
        rows.append({'candidate_id':c['candidate_id'],'station_request_id':c['station_request_id'],
            'left_offset_m':c['left_offset_m'],'x_world_m':c['cart']['center_world_xyz'][0],
            'y_world_m':c['cart']['center_world_xyz'][1],'z_world_m':c['cart']['center_world_xyz'][2],
            'yaw_rad':c['cart']['yaw_rad'],'physical_pass':c['physical_pass'],
            'bypass_clearance_m':c['bypass']['cart_clearance_m'],
            'C1_state':v['rows'][0]['state'],'C1_pixels':v['rows'][0]['cart_visible_pixel_count'],
            'first_visible_request_id':v['first_visible_request_id'],'first_clear_request_id':v['first_clear_request_id'],
            'old_clearance_at_first_clear_m':t['old_remaining_clearance_m'] if t else None,
            'temporal_pass_fresh_ids':','.join(map(str,c['temporal_pass_pair_ids'])),
            'qualified':c['qualified'],'failures':','.join(c['failures'])})
    with (out/'derived/candidate_table.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    write_json(out/'derived/candidate_table.json',rows)
    write_json(out/'derived/plot_coordinates.json',{'baseline_world_xy':run['xy'].tolist(),'baseline_times':run['times'].tolist(),
        'observation_poses':[e['agent_pose_at_observation'] for e in run['events']],
        'diagnostic_candidate_id':cid,'diagnostic_only':True,'cart':cart,
        'first_clear_baseline_old_world':run['worlds'][oid-1].tolist(),
        'first_clear_baseline_fresh_world':run['worlds'][fid-1].tolist(),
        'old_remaining_world':temporal['remaining_world_xy'],'baseline_switch':run['events'][fid-1]['application'],
        'bypass_world_xy':diagnostic['bypass']['world_xy'],'obstacle_conditioned_curves':None})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.colors import ListedColormap
    from PIL import Image
    walls=wall_inventory(json.loads((ROOT/protocol['config']['collision_inventory']).read_text()))
    def context(ax,bounds):
        for w in walls:
            lo=np.array(w['world_aabb_min']);hi=np.array(w['world_aabb_max'])
            if hi[0]>=bounds[0] and lo[0]<=bounds[1] and hi[1]>=bounds[2] and lo[1]<=bounds[3]:
                ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],fc='.90',ec='.80',lw=.5,zorder=0))
        ax.plot(*run['xy'].T,c='tab:blue',lw=1.7,label='Actual NO-CART baseline')
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.15)
    def save(fig,name):
        fig.savefig(out/'figures'/f'{name}.png',dpi=180,bbox_inches='tight');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(16,6),gridspec_kw={'width_ratios':[1.3,1]})
    context(axes[0],[-22,9,0,14]);context(axes[1],[0,8,8.5,12.9])
    axes[0].scatter(*run['xy'][0],c='green',s=55,marker='s',label='Start');axes[0].scatter(*run['xy'][-1],c='red',s=55,marker='X',label='Final')
    for c in candidates:
        xy=c['cart']['center_world_xyz'][:2]
        for ax in axes: ax.scatter(*xy,s=25,c='purple')
        axes[1].annotate(str(c['candidate_id']),xy,xytext=(5,3),textcoords='offset points',fontsize=9)
    axes[0].legend(loc='lower left');axes[0].set_title('Original E16 actual path; no new navigation')
    axes[1].set_title('12 frozen candidate centres (IDs, not requests)')
    fig.suptitle('Bounded stationary-cart preflight | 12 physical passes, 0 strict reveal passes',fontsize=14)
    fig.tight_layout();save(fig,'baseline_and_candidates')
    fig,axes=plt.subplots(3,4,figsize=(16,14))
    for index,(ax,c) in enumerate(zip(axes.flat,candidates)):
        xy=c['cart']['center_world_xyz'][:2];context(ax,[xy[0]-2.7,xy[0]+2.7,xy[1]-2.5,xy[1]+2.5])
        ax.plot(*outline(c['cart']).T,c='purple',label='Cart original bounds')
        ax.plot(*outline(c['cart'],radius).T,'r--',lw=.8,label='Inflated cart')
        ax.plot(*np.array(c['bypass']['world_xy']).T,c='green',lw=1,label='Connected bypass proxy')
        ax.set_title(f"Candidate {c['candidate_id']} | {c['visibility']['rows'][0]['state']} at C1",fontsize=10)
        if index<8: ax.set_xlabel('')
    handles,labels=axes.flat[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=4)
    fig.suptitle('Independent placements; sampled static query and exact cart clearance\nGreen curves are model-free feasibility paths, not VLA predictions or execution')
    fig.subplots_adjust(left=.055,right=.99,top=.90,bottom=.085,hspace=.38,wspace=.30)
    save(fig,'physical_bypass_candidates')
    fig,ax=plt.subplots(figsize=(14,6));values=np.full((12,20),3.)
    for i,c in enumerate(candidates):
        for r in c['visibility']['rows']:values[i,r['request_id']-1]={'HIDDEN':0,'MARGINAL':1,'CLEAR':2}[r['state']]
    ax.imshow(values,aspect='auto',cmap=ListedColormap(['#d7e7f4','#ffbc5e','#c26bd1','#eeeeee']),vmin=0,vmax=3)
    for i,c in enumerate(candidates):
        ax.scatter(np.array(c['temporal_pass_pair_ids'])-1,[i]*len(c['temporal_pass_pair_ids']),facecolors='none',edgecolors='black',s=100)
        for r in c['visibility']['rows']:ax.text(r['request_id']-1,i,r['state'][0],ha='center',va='center',fontsize=9)
    ax.set(xticks=np.arange(20),xticklabels=np.arange(1,21),yticks=np.arange(12),yticklabels=np.arange(1,13),xlabel='Saved baseline request C#',ylabel='Frozen candidate ID')
    ax.set_title('H=zero pixels; M=MARGINAL; C=first CLEAR; grey=unmeasured after first CLEAR\nOpen circles: later baseline OLD-conflict/positive-response pairs, NOT replacement reveals')
    fig.tight_layout();save(fig,'visibility_and_temporal_opportunity')
    sequence_ids=sorted(set([1,selected['first_visible_request_id']-1,selected['first_visible_request_id'],fid-1,fid]))
    fig,axes=plt.subplots(2,3,figsize=(16,7))
    for ax,rid in zip(axes.flat,sequence_ids):
        row=selected['rows'][rid-1];folder=source/'raw/visibility'/f'candidate_{cid:02d}'
        rgb=np.asarray(Image.open(folder/f'C{rid:02d}.png'));ax.imshow(rgb);box=row['cart_bbox_if_visible']
        if box:ax.add_patch(Rectangle((box[0],box[1]),box[2]-box[0],box[3]-box[1],fill=False,ec='magenta',lw=1.5))
        ax.set_title(f"C{rid}: {row['state']} | {row['cart_visible_pixel_count']} pixels");ax.axis('off')
    for ax in list(axes.flat)[len(sequence_ids):]:ax.axis('off')
    fig.suptitle(f'Diagnostic candidate {cid}: real preflight Hawk RGB, saved NO-CART robot poses\nMagenta box derived from semantic mask; no obstacle-conditioned model call')
    fig.tight_layout(rect=[0,0,1,.91]);save(fig,'diagnostic_visibility_sequence')
    fig,axes=plt.subplots(1,2,figsize=(15,7))
    for ax in axes:
        context(ax,[0,8.5,4.5,12.8]);ax.plot(*outline(cart).T,c='purple',lw=2,label='Stationary cart')
        ax.plot(*outline(cart,radius).T,'r--',label='Inflated cart')
    for ax,fresh in zip(axes,[fid,diagnostic['temporal_pass_pair_ids'][0]]):
        old=fresh-1;metric=diagnostic['temporal_pairs'][fresh-2]
        ax.plot(*run['worlds'][old-1].T,c='tab:orange',lw=2,label=f'Baseline OLD C{old}')
        ax.plot(*run['worlds'][fresh-1].T,c='tab:purple',lw=2,label=f'Baseline FRESH C{fresh}')
        for rid in [old,fresh]:
            xy=run['events'][rid-1]['agent_pose_at_observation'][:2];ax.scatter(*xy,c='black',s=22);ax.annotate(f'C{rid}',xy,xytext=(6,-12),textcoords='offset points')
        ax.set_title(f"{'Actual first CLEAR preflight' if fresh==fid else 'Later proxy conflict, NOT first reveal'}\nC{old}→C{fresh}; OLD clearance {metric['old_remaining_clearance_m']:+.3f} m")
        ax.legend(loc='lower left',fontsize=8)
    fig.suptitle('Original saved no-cart predictions only | No cart-conditioned FRESH avoidance evidence')
    fig.tight_layout(rect=[0,0,1,.94]);save(fig,'first_clear_vs_later_conflict')
    viewer=out/'viewer_data';viewer.mkdir();vis=[]
    for v in selected['rows']:
        rid=v['request_id'];folder=source/'raw/visibility'/f'candidate_{cid:02d}'
        vis.append({**v,**{k.replace('cart_','human_'):value for k,value in v.items() if k.startswith('cart_')},
            'rgb':str(folder/f'C{rid:02d}.png'),'mask':str(folder/f'C{rid:02d}.mask.png'),
            'baseline_rgb':str(run['path']/run['events'][rid-1]['rgb_observation_reference'])})
    candidate={'id':cid,'position':cart['center_world_xyz'],'visibility':vis,'strict_qualified':False,'selection_rank':None,
        'failures':diagnostic['failures'],'old_request_id':oid,'fresh_request_id':fid,
        'first_visible_request_id':selected['first_visible_request_id'],'first_clear_request_id':fid,
        'old_full_world_xy':run['worlds'][oid-1].tolist(),'remaining_world_xy':temporal['remaining_world_xy'],
        'fresh_world_xy':run['worlds'][fid-1].tolist(),'old_min_clearance_m':temporal['old_remaining_clearance_m'],
        'robot_current_clearance_at_reveal_m':temporal['current_robot_clearance_m'],'conflict':None,
        'reveal_lead_s':None,'switch_to_conflict_margin_s':None,'bypass':{'pass':True,
            'centerline_xy':diagnostic['bypass']['world_xy'],'clearance_m':diagnostic['bypass']['cart_clearance_m']},
        'cart_outline':outline(cart).tolist(),'inflated_outline':outline(cart,radius).tolist()}
    cfg={'baseline_run':str(run['path']),'reference_baseline_run':str(run['path']),'baseline_analysis':str(out),
         'collision_inventory':str(ROOT/protocol['config']['collision_inventory']),'cart_frozen':diagnostic['frozen_asset'],
         'robot_radius_m':radius,'visibility':protocol['visibility_rules'],
         'preflight_title':f'CANDIDATE {cid} PREFLIGHT ONLY | NO CART-CONDITIONED RUN'}
    for name,value in [('preregistered.json',{'config':cfg}),('candidates.json',[candidate]),
        ('summary.json',{'obstacle_type':'cart','preflight_only':True,'selected_candidate_id':None,'diagnostic_candidate_id':cid}),
        ('source_evidence_sha256.json',hashes)]:write_json(viewer/name,value)
    verify_hashes(hashes)
    write_json(out/'metadata.json',{'source_evidence_sha256':hashes,'figure_count':5,'model_calls':0,
        'physics_reexecuted':False,'diagnostic_selection':'Largest preregistered ID; never a qualifying scenario or substituted actual pair',
        'command_argv':sys.argv,**provenance(ROOT)})
    print(out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-dir',required=True);p.add_argument('--output-dir',required=True)
    a=p.parse_args();report(Path(a.source_dir).resolve(),Path(a.output_dir).resolve())
