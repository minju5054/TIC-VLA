"""Saved-only stopped-preflight report; never fabricates a human model response."""
import argparse
import csv
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.e16_c19_manual import check_frozen
from research.plot_e16_observation_path import load_observations,draw_map,FIELDS
from research.hospital_spatial_gate import wall_inventory
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.records import write_json,provenance


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--preflight-dir',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
    source=Path(a.preflight_dir).resolve();frozen,run=check_frozen(source)
    out=validate_destination(a.output_dir,[source,run['path']]);out.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((source/'preregistered.json').read_text())['config']
    physical=json.loads((source/'physical.json').read_text())
    candidate=json.loads((source/'candidates.json').read_text())[0] if (source/'candidates.json').exists() else None
    if candidate and candidate['strict_qualified']:raise ValueError('Passing preflight requires the authorized human-run analysis first')
    hashes=evidence_hashes([source,run['path'],Path(cfg['collision_inventory']),Path(__file__)])
    rows,dense=load_observations(run['path']);visibility=candidate['visibility'] if candidate else []
    first=candidate.get('fresh_request_id') if candidate else None
    failure=('C19 HUMAN PLACEMENT PHYSICALLY INVALID' if not physical['pass'] else
             'First CLEAR is C1: no preceding OLD and no three prior HIDDEN observations' if first==1 else
             'Strict first-CLEAR gate failed: '+', '.join(candidate['failures']))
    write_json(out/'summary.json',{'decision':'STOP BEFORE HUMAN INFERENCE','failed_gate':failure,
        'physical_pass':physical['pass'],'first_clear_request_id':first,'human_model_run_executed':False})
    with (out/'observation_poses_visibility.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=FIELDS+['visibility','human_pixels']);w.writeheader()
        for r in rows:
            v=visibility[r['request_id']-1] if visibility else {}
            w.writerow({**r,'visibility':v.get('state','UNMEASURED'),'human_pixels':v.get('human_visible_pixel_count')})
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Patch
    from matplotlib.lines import Line2D
    colors={'HIDDEN':'#23805b','MARGINAL':'#e1a11a','CLEAR':'#c84237','UNMEASURED':'#777777'}
    walls=wall_inventory(json.loads(Path(cfg['collision_inventory']).read_text()))
    human=np.array(frozen['human']['position']);radius=cfg['robot_radius_m']+cfg['human_radius_m'];figures={}
    for zoom in [False,True]:
        fig,markers,info=draw_map(rows,dense,walls,zoom);ax=fig.axes[0]
        markers.set_facecolors([colors[visibility[i-1]['state']] if visibility else colors['UNMEASURED'] for i in info['request_ids']])
        expected=np.array([[rows[i-1]['x_world_m'],rows[i-1]['y_world_m']] for i in info['request_ids']])
        np.testing.assert_array_equal(markers.get_offsets(),expected)
        ax.scatter(*human[:2],marker='*',s=180,c='#ac2580',edgecolors='white',lw=.7,zorder=10)
        proxy=Circle(human[:2],radius,fill=False,color='#ac2580',ls='--',lw=1.4,zorder=3);ax.add_patch(proxy)
        np.testing.assert_array_equal(proxy.center,human[:2])
        yaw=frozen['human']['yaw'];ax.annotate('',human[:2]+.55*np.array([np.cos(yaw),np.sin(yaw)]),human[:2],
            arrowprops=dict(arrowstyle='->',color='#ac2580',lw=1.5),zorder=9)
        if zoom:ax.annotate('Frozen human\nC19 + 0.40 m world Y',human[:2],xytext=(0,45),textcoords='offset points',
            ha='center',fontsize=10,color='#8e1b68',bbox=dict(fc='white',ec='none',pad=2),arrowprops=dict(arrowstyle='-',color='#8e1b68'))
        for legend in fig.legends:legend.remove()
        handles=[Line2D([],[],color='#216e9d',label='No-human baseline path')]
        handles += [Line2D([],[],marker='o',ls='',color=color,label=state+' observation') for state,color in colors.items() if state in {v['state'] for v in visibility} or not visibility and state=='UNMEASURED']
        handles += [Line2D([],[],marker='*',ls='',color='#ac2580',ms=10,label='Frozen stationary human'),
                    Line2D([],[],color='#ac2580',ls='--',label='Combined conflict proxy'),Patch(fc='#e1e5e9',label='Structural collider XY bounds')]
        fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.90),ncol=3,frameon=False,fontsize=10)
        fig.suptitle('E16 | C19 + 0.40 m world Y | STRICT PREFLIGHT FAIL'+(' | turn zoom' if zoom else ''),fontsize=16,y=.98)
        for t in fig.texts:
            if t.get_position()[1]==.93:t.set_text('Semantic visibility at saved baseline observations; C# markers remain exact observation poses')
            if t.get_position()[1]==.03:t.set_text(failure+'\nHuman model run NOT EXECUTED; no FRESH response or revision is inferred.');t.set_color('#923329')
        stem='trajectory_turn_zoom' if zoom else 'trajectory_result'
        fig.savefig(out/(stem+'.png'),dpi=240);fig.savefig(out/(stem+'.pdf'));plt.close(fig);figures[stem]=info
    write_json(out/'trajectory_coordinates.json',{'observations':rows,'dense_baseline_world_xy':run['xy'].tolist(),
        'dense_baseline_sim_times':run['times'].tolist(),'dense_source':str(run['path']/'robot_state.csv'),
        'frozen_human':frozen['human'],'combined_proxy_radius_m':radius,'visibility':visibility,'figures':figures,
        'coordinate_rule':'All measured coordinates unchanged in Hospital world metres; human-run path and FRESH response absent.'})
    if visibility:
        from PIL import Image
        ids=sorted(set([i for i in range(max(1,first-3),min(48,first+1)+1)])) if first else [1,48]
        fig,axes=plt.subplots(1,len(ids),figsize=(7*len(ids),4.8),squeeze=False)
        for ax,rid in zip(axes[0],ids):
            v=visibility[rid-1];ax.imshow(Image.open(source/f'candidate_001/C{rid:02d}.overlay.png'));ax.axis('off')
            ax.set_title(f"C{rid} | {v['state']} | {v['human_visible_pixel_count']} pixels"+(' | FIRST CLEAR' if rid==first else ''),fontsize=13)
        fig.suptitle('Frozen human at saved Hawk poses; magenta = semantic human mask',fontsize=15)
        fig.text(.5,.03,'First CLEAR is C1: earlier observations and OLD do not exist.' if first==1 else failure,ha='center',fontsize=12)
        fig.tight_layout(rect=[0,.06,1,.93]);fig.savefig(out/'first_reveal_sheet.png',dpi=180);plt.close(fig)
    # Thin data adapter for the existing saved-only viewer. Raw renders stay in
    # the preflight directory; no extra sensor capture is needed by the GUI.
    viewer=out/'viewer_data';viewer.mkdir()
    result=candidate or {**physical,'strict_qualified':False,'selection_rank':None,'visibility':None,'failures':['E'],'remaining_world_xy':[]}
    if candidate:
        result=json.loads(json.dumps(candidate))
        for v in result['visibility']:
            rid=v['request_id'];v.update(rgb=str(source/f'candidate_001/C{rid:02d}.png'),mask=str(source/f'candidate_001/C{rid:02d}.mask.png'))
    for name,value in [('preregistered.json',{'config':cfg}),('human_asset.json',json.loads((source/'human_asset.json').read_text())),
        ('candidates.json',[result]),('summary.json',{'selected_candidate_id':None,'diagnostic_candidate_id':1,'single_manual_placement':True}),
        ('source_evidence_sha256.json',hashes)]:write_json(viewer/name,value)
    verify_hashes(hashes)
    write_json(out/'metadata.json',{**provenance(ROOT),'source_evidence_sha256':hashes,'plot_coordinates_exact':True,
        'source_files_unchanged':True,'model_calls':0,'navigation_physics_reexecuted':False,'failed_gate':failure})
    print(json.dumps({'output_dir':str(out),'failed_gate':failure}))


if __name__=='__main__':main()
