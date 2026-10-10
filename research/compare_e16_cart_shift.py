"""Saved-only comparison of the baseline, exact-C19 cart and one +0.40 world-Y cart."""
import argparse,csv,json,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_e16_cart import pair
from research.cart_bypass import outcome,predicted_side,plot_coordinates
from research.cart_geometry import outline
from research.hospital_cart import COPY
from research.hospital_spatial_gate import wall_inventory
from research.plot_e16_observation_path import draw_map,FIELDS
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.records import write_json,provenance


def write_csv(path,rows):
    if rows:
        with Path(path).open('x') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def analyze_run(run,cart,radius,present=True):
    cart_contacts=[s for s in run['contacts'] if COPY in json.dumps(s)]
    structural=[s for s in run['contacts'] if any(p.startswith('/Root/') for p in s['colliders'])]
    first=min(structural,key=lambda c:c['start_sim_time']) if structural else None
    o=outcome(run['xy'],run['times'],cart,radius,run['contacts'],run['cfg']['controller']['max_v'])
    o['clears_before_structural_contact']=bool(o['clear_region'] and (first is None or o['clear_region']['sim_time']<first['start_sim_time']))
    rows=[];details={}
    for old in range(1,len(run['events'])):
        r,d=pair(run,old,cart,radius);fresh=run['events'][old];previous=run['events'][old-1]
        sides=[predicted_side(d['old_remaining_world_xy'],d['old_remaining_times'],cart,radius) if d['old_remaining_world_xy'] else {'side':'UNKNOWN / DOES NOT REACH','crossing':None},
            predicted_side(d['fresh_world_xy'],fresh['observation']['sim_time']+.1*np.arange(1,31),cart,radius)]
        t=r['fresh_observation_sim_time'];window=o['interaction_start_s'] is not None and o['interaction_start_s']<=t<=o['interaction_end_s']
        r.update(old_observation_x=previous['agent_pose_at_observation'][0],old_observation_y=previous['agent_pose_at_observation'][1],
            fresh_observation_x=fresh['agent_pose_at_observation'][0],fresh_observation_y=fresh['agent_pose_at_observation'][1],
            interaction_window=bool(window),old_predicted_side=sides[0]['side'],fresh_predicted_side=sides[1]['side'],
            structural_contact_already_reported=bool(first and first['start_sim_time']<=t),
            old_conflict_fresh_improved=bool(present and r['old_predicts_collision'] and r['fresh_cart_clearance_m']>r['old_remaining_cart_clearance_m']+1e-9),
            fresh_clear=bool(r['fresh_cart_clearance_m']>0))
        r['predicted_side_change']=set([r['old_predicted_side'],r['fresh_predicted_side']])=={'NORTH/RIGHT BYPASS','SOUTH/LEFT BYPASS'}
        d['predicted_side_details']=sides;rows.append(r);details[str(old)]=d
    window=[r for r in rows if r['interaction_window'] and r['max_abs_lateral_revision_m'] is not None]
    chosen=max(window,key=lambda r:(r['max_abs_lateral_revision_m'],-r['old_request_id'])) if window else None
    improvement=[r['old_request_id'] for r in rows if r['old_conflict_fresh_improved']]
    changes=[r['old_request_id'] for r in rows if r['interaction_window'] and r['predicted_side_change']]
    comparison={'actual_bypass_side':o['actual_bypass_side'] if present else 'N/A - no cart',
        'geometric_passage_side':o['geometric_passage_side'] if present else 'N/A - no cart',
        'cart_contact_spans':len(cart_contacts) if present else None,'minimum_cart_clearance_proxy_m':o['closest_approach']['clearance_m'] if present else None,
        'first_structural_contact_prim':next((p for p in first['colliders'] if p.startswith('/Root/')),None) if first else None,
        'first_structural_contact_time_s':first['start_sim_time'] if first else None,
        'DoorFrame52_contact':any('/Root/Geo_M_DoorFrame52/' in json.dumps(c) for c in structural),
        'clears_cart_region_before_structural_contact':o['clears_before_structural_contact'] if present else None,
        'distance_after_clear_or_matched_reference_plane_m':o['distance_travelled_after_clearing_m'],
        'forward_progress_after_clear_or_matched_reference_plane_m':o['signed_forward_progress_after_clearing_m'],
        'final_x_world_m':float(run['xy'][-1,0]),'final_y_world_m':float(run['xy'][-1,1]),
        'max_lateral_revision_in_interaction_m':chosen['max_abs_lateral_revision_m'] if chosen else None,
        'max_abs_delta_w_in_interaction':max((abs(r['delta_w']) for r in window),default=None),
        'max_lateral_revision_all_pairs_m':max((r['max_abs_lateral_revision_m'] for r in rows if r['max_abs_lateral_revision_m'] is not None),default=None),
        'max_abs_delta_w_all_pairs':max((abs(r['delta_w']) for r in rows),default=None)}
    return {'outcome':o,'comparison':comparison,'pairs':rows,'details':details,'selected_old_id':chosen['old_request_id'] if chosen else None,
        'old_conflict_fresh_improved_ids':improvement,'first_predicted_side_change_old_id':changes[0] if changes else None,
        'first_structural_contact':first,'cart_contacts':cart_contacts}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['run-dir','preflight-dir','rules','output-dir']:p.add_argument('--'+name,required=True)
    a=p.parse_args();pre=Path(a.preflight_dir).resolve();out=validate_destination(a.output_dir,[pre,a.run_dir]);out.mkdir(parents=True)
    # Existing cart analysis preserves first visibility/CLEAR independently.
    subprocess.run([sys.executable,str(ROOT/'research/analyze_e16_cart.py'),'--run-dir',a.run_dir,'--preflight-dir',str(pre),'--output-dir',str(out/'first_reveal')],check=True,stdout=subprocess.DEVNULL)
    runs={'no_cart':load_saved(ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01'),
        'exact_C19':load_saved(ROOT/'outputs/nova-e16-cart-20261010-01'),'plus_Y':load_saved(a.run_dir)}
    frozen=json.loads((pre/'freeze.json').read_text());oldf=json.loads((ROOT/'outputs/e16-cart-preflight-20261010-01/freeze.json').read_text())
    cart,oldcart=frozen['cart'],oldf['cart'];radius=frozen['robot_radius_m'];rules=json.loads(Path(a.rules).read_text())
    hashes=evidence_hashes([*(r['path'] for r in runs.values()),pre,ROOT/'outputs/e16-cart-preflight-20261010-01',Path(a.rules),Path(__file__),ROOT/'research/cart_bypass.py',ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'])
    results={name:analyze_run(r,oldcart if name=='exact_C19' else cart,radius,name!='no_cart') for name,r in runs.items()}
    new=results['plus_Y'];old=results['exact_C19'];selected=new['selected_old_id'];coords=plot_coordinates(runs)
    if old['outcome']['actual_bypass_side']!='NORTH/RIGHT BYPASS':raise ValueError('Previous known side not reproduced; inspect geometry')
    decision={'SOUTH/LEFT BYPASS':'CART +Y SHIFT PRODUCED SOUTH/LEFT BYPASS','NORTH/RIGHT BYPASS':'CART +Y SHIFT RETAINED NORTH/RIGHT BYPASS'}.get(new['outcome']['actual_bypass_side'],'NO CLEAR CART BYPASS')
    if not runs['plus_Y']['complete']:decision='INSUFFICIENT EVIDENCE'
    summary={'decision':decision,'adjacent_pair_decision':'OLD CONFLICT -> FRESH CLEAR/IMPROVED' if new['old_conflict_fresh_improved_ids'] else 'NO RECONCILIATION-RELEVANT ADJACENT PAIR',
        'selected_inspection_pair':[selected,selected+1] if selected else None,'selection_rule':rules['inspection_pair'],
        'outcome':new['outcome'],'comparison':[{**{'run':name},**r['comparison']} for name,r in results.items()],
        'first_reveal':json.loads((out/'first_reveal/summary.json').read_text()),
        'first_predicted_side_change_old_id':new['first_predicted_side_change_old_id'],'old_conflict_fresh_improved_ids':new['old_conflict_fresh_improved_ids'],
        'selected_metrics':new['pairs'][selected-1] if selected else None,
        'first_structural_contact':new['first_structural_contact'],'interpretation':'Single intervention, separate closed-loop histories; no isolated causal mechanism or graph/reconciliation improvement validation.'}
    write_json(out/'summary.json',summary);write_json(out/'rules.json',rules);write_csv(out/'comparison.csv',summary['comparison'])
    for name,r in results.items():
        write_json(out/(name+'_pair_metrics.json'),r['pairs']);write_csv(out/(name+'_pair_metrics.csv'),r['pairs'])
        write_json(out/(name+'_pair_details.json'),r['details'])
    write_csv(out/'interaction_pair_metrics.csv',[r for r in new['pairs'] if r['interaction_window']])
    write_csv(out/'observation_poses.csv',[dict(zip(FIELDS,[r['request_id'],r['sim_time'],*r['pose'],float(np.degrees(r['pose'][2]))])) for r in coords['plus_Y']['observations']])
    footprints={'previous':outline(oldcart).tolist(),'new':outline(cart).tolist(),'inflated_new':outline(cart,radius).tolist()}
    coords.update(footprints=footprints,selected_pair=summary['selected_metrics'],selected_details=new['details'].get(str(selected)),outcome=new['outcome'])
    plot_comparison(out,runs,results,cart,oldcart,coords,summary)
    write_json(out/'trajectory_coordinates.json',coords)
    # Reuse the proven saved-data GUI; replace only its descriptive inspection pair.
    original=out/'first_reveal/viewer_data';viewer=out/'viewer_data';viewer.mkdir()
    cfg=json.loads((original/'preregistered.json').read_text())['config'];cfg.update(baseline_analysis=str(out),previous_cart_run=str(runs['exact_C19']['path']),previous_cart_outline=footprints['previous'])
    c=json.loads((original/'candidates.json').read_text())[0];d=new['details'].get(str(selected));r=summary['selected_metrics']
    if selected:
        c.update(old_request_id=selected,fresh_request_id=selected+1,remaining_world_xy=d['old_remaining_world_xy'],fresh_world_xy=d['fresh_world_xy'],
            old_min_clearance_m=r['old_remaining_cart_clearance_m'],robot_current_clearance_at_reveal_m=r['current_cart_clearance_m'],conflict=d['old_conflict'])
    c.update(strict_qualified=False,failures=[decision],measured_bypass_side=new['outcome']['actual_bypass_side'],
        approach_sim_time=new['outcome']['interaction_start_s'],closest_sim_time=new['outcome']['closest_approach']['sim_time'],
        structural_contact_sim_time=new['comparison']['first_structural_contact_time_s'])
    combined={**hashes,**json.loads((original/'source_evidence_sha256.json').read_text())}
    for name,value in [('preregistered.json',{'config':cfg}),('candidates.json',[c]),('summary.json',{'obstacle_type':'cart','selected_candidate_id':None,'diagnostic_candidate_id':1}),('source_evidence_sha256.json',combined)]:write_json(viewer/name,value)
    verify_hashes(hashes);write_json(out/'metadata.json',{'source_evidence_sha256':hashes,'source_files_unchanged':True,**provenance(ROOT)})
    print(json.dumps({k:v for k,v in summary.items() if k!='first_reveal'},indent=2))


def plot_comparison(out,runs,results,cart,oldcart,coords,summary):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    walls=wall_inventory(json.loads((ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json').read_text()))
    rows=[dict(zip(FIELDS,[r['request_id'],r['sim_time'],*r['pose'],float(np.degrees(r['pose'][2]))])) for r in coords['plus_Y']['observations']]
    dense=np.array([[float(r[k]) for k in ['tick','sim_time','x','y','yaw']] for r in runs['no_cart']['robot']])
    selected=results['plus_Y']['selected_old_id'];detail=results['plus_Y']['details'].get(str(selected));figure_info={}
    for stem,zoom in [('trajectory_result',False),('trajectory_turn_zoom',True),('trajectory_revision',True)]:
        # Every C is marked. Labels are spaced along measured motion, with selected
        # and final C always retained; this cannot move or downsample path points.
        labelled=[];last=None
        for r in rows:
            xy=np.array([r['x_world_m'],r['y_world_m']])
            if last is None or np.linalg.norm(xy-last)>.4 or r['request_id'] in [selected,selected+1 if selected else None,len(rows)]:labelled.append(r);last=xy
        fig,marker,info=draw_map(labelled,dense,walls,zoom);ax=fig.axes[0]
        marker.set_color('#239461');allpoints=np.array([[r['x_world_m'],r['y_world_m']] for r in rows])
        artist=ax.scatter(*allpoints.T,s=15,c='#239461',edgecolors='white',lw=.4,zorder=5);np.testing.assert_array_equal(artist.get_offsets(),allpoints)
        ax.plot(*runs['exact_C19']['xy'].T,color='#b78329',lw=2,zorder=3)
        ax.plot(*runs['plus_Y']['xy'].T,color='#239461',lw=2,zorder=3)
        for key,color,style in [('previous','#b78329',':'),('new','#b32680','-'),('inflated_new','#b32680','--')]:
            ax.plot(*np.array(coords['footprints'][key]).T,color=color,ls=style,lw=1.7,zorder=4)
        for text in ax.texts:
            if text.get_text()=='Final':text.set_text('No-cart final')
            offsets={f'C{selected}':(-70,-18),f'C{selected+1}' if selected else '':(-75,24),f'C{len(rows)}':(35,10)}
            if text.get_text() in offsets:text.set_position(offsets[text.get_text()])
        df=next(w for w in walls if '/Geo_M_DoorFrame52/' in w['path']);dfxy=(np.array(df['world_aabb_min'][:2])+df['world_aabb_max'][:2])/2
        annotations=[('DoorFrame52',dfxy,(-110,55),'#8a3449'),('New cart',cart['center_world_xyz'][:2],(-110,-55),'#b32680')]
        for key,label,color,offset in [('exact_C19','Previous north/right','#b78329',(-125,24)),('plus_Y','New '+summary['outcome']['actual_bypass_side'].lower(),'#239461',(-80,-65))]:
            cross=results[key]['outcome']['centre_crossing'];point=cross['world_xy'] if cross else results[key]['outcome']['closest_approach']['world_xy']
            annotations.append((label,point,offset,color))
            contact=results[key]['first_structural_contact']
            if contact:
                t=contact['start_sim_time'];xy=np.array([np.interp(t,runs[key]['times'],runs[key]['xy'][:,j]) for j in range(2)])
                ax.scatter(*xy,marker='X',s=90,c=color,edgecolors='black',lw=.5,zorder=8)
                annotations.append(('First contact '+('previous' if key=='exact_C19' else 'new'),xy,(65,70 if key=='exact_C19' else -85),color))
        for label,xy,offset,color in annotations:
            ax.annotate(label,xy,xytext=offset,textcoords='offset points',fontsize=9,color=color,bbox=dict(fc='white',ec='none',alpha=.9),arrowprops=dict(arrowstyle='-',color=color,lw=.65),zorder=10)
        if detail:
            ax.plot(*np.array(detail['old_remaining_world_xy']).T,color='#e26416',lw=2.5,zorder=6);ax.plot(*np.array(detail['fresh_world_xy']).T,color='#6b4db6',lw=2.5,zorder=6)
            for e,color,symbol in [(runs['plus_Y']['events'][selected-1],'#e26416','s'),(runs['plus_Y']['events'][selected],'#6b4db6','D')]:ax.scatter(*e['observation']['position'][:2],s=95,facecolors='none',edgecolors=color,marker=symbol,zorder=9)
            app=runs['plus_Y']['events'][selected]['application'];ax.scatter(*app['position'][:2],c='black',marker='x',s=95,zorder=10)
        for legend in fig.legends:legend.remove()
        handles=[Line2D([],[],color=c,label=l,ls=ls) for c,l,ls in [('#216e9d','No-cart reference','-'),('#b78329','Previous exact-C19','-'),('#239461','New C19 +0.40 world Y','-'),('#b78329','Previous footprint',':'),('#b32680','New footprint','-'),('#b32680','New inflated region','--'),('#e26416','OLD remaining','-'),('#6b4db6','FRESH','-')]]
        handles+=[Line2D([],[],marker='x',c='black',ls='',label='Application'),Patch(fc='#e1e5e9',label='Structural bounds')]
        fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.90),ncol=5,frameon=False,fontsize=9)
        fig.suptitle('Hospital E16 | Cart +0.40 m world Y'+(' | OLD/FRESH inspection' if stem.endswith('revision') else ' | turn' if zoom else ''),y=.98,fontsize=16)
        metric=summary['selected_metrics']
        for t in fig.texts:
            if t.get_position()[1]==.93:t.set_text((f'Inspection C{selected} -> C{selected+1} | max lateral {metric["max_abs_lateral_revision_m"]:.3f} m | RMSE {metric["aligned_overlap_rmse_m"]:.3f} m' if selected else 'No adjacent inspection pair')+(' | POST-CONTACT' if metric and metric['structural_contact_already_reported'] else '')+' | reveal separate')
            if t.get_position()[1]==.03:t.set_text(summary['decision']+'\nMeasured paths/observation markers; nominal waypoint timing and tangents are derived. No correction.');t.set_color('#46535e')
        if stem.endswith('revision') and detail:
            points=np.vstack([detail['old_remaining_world_xy'],detail['fresh_world_xy'],coords['footprints']['inflated_new'],runs['plus_Y']['events'][selected]['application']['position'][:2]])
            lo,hi=points.min(0)-.7,points.max(0)+.7;ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
        if not zoom and detail:
            # Keep the full predicted curves and callouts below the legend.
            top=max(np.max(detail['old_remaining_world_xy'],axis=0)[1],np.max(detail['fresh_world_xy'],axis=0)[1])+2.
            ax.set_ylim(ax.get_ylim()[0],max(ax.get_ylim()[1],top))
        ax.set_aspect('equal',adjustable='box');fig.savefig(out/(stem+'.png'),dpi=220);fig.savefig(out/(stem+'.pdf'));plt.close(fig);figure_info[stem]=info
    coords['figures']=figure_info


if __name__=='__main__':main()
