"""Saved-only cart footprint clearance, nominal revision, figures and replay adapter."""
import argparse,csv,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_handoffs import metrics as latency
from research.route_geometry import temporal_overlap,boundary_seam,rotation
from research.nova_dynamic_geometry import remaining_curve
from research.cart_geometry import path_clearance,clearance,outline
from research.hospital_cart import COPY
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.records import write_json,provenance
from research.plot_e16_observation_path import draw_map,FIELDS
from research.hospital_spatial_gate import wall_inventory


def pair(run,old_id,cart,radius):
    old,fresh=run['events'][old_id-1:old_id+1];ow,fw=run['worlds'][old_id-1:old_id+1]
    ot,ft=old['observation']['sim_time'],fresh['observation']['sim_time']
    rt,rp=remaining_curve(ow,ot,ft,.1);oc=path_clearance(rp,cart,radius,rt) if rp is not None else {'minimum_clearance_m':None,'first_conflict':None}
    fc=path_clearance(fw,cart,radius,ft+.1*np.arange(1,31));aligned=temporal_overlap(ow,fw,ot,ft,.1,1e-6)
    seam=boundary_seam(fw,fresh['observation'],fresh['application'],run['times'],run['xy'])
    l=latency(fresh);l['cart_translation_m']=l.pop('human_translation_m')
    v,w=np.array(fresh['controller_command'])-old['controller_command'];norm=np.linalg.norm(aligned['delta'],axis=1) if aligned else None
    local=aligned['delta']@rotation(fresh['agent_pose_at_observation'][2]) if aligned else None
    row={'old_request_id':old_id,'fresh_request_id':old_id+1,'old_observation_sim_time':ot,'fresh_observation_sim_time':ft,
        'old_remaining_cart_clearance_m':oc['minimum_clearance_m'],'old_predicts_collision':bool(oc['minimum_clearance_m'] is not None and oc['minimum_clearance_m']<=0),
        'fresh_cart_clearance_m':fc['minimum_clearance_m'],'current_cart_clearance_m':float(clearance(fresh['observation']['position'][:2],cart,radius)),
        'predicted_conflict_sim_time':oc['first_conflict']['sim_time'] if oc['first_conflict'] else None,
        'application_sim_time':fresh['application']['sim_time'],
        'aligned_overlap_rmse_m':aligned['rmse'] if aligned else None,'aligned_overlap_max_m':float(norm.max()) if aligned else None,
        'mean_abs_lateral_revision_m':float(np.abs(local[:,1]).mean()) if aligned else None,'max_abs_lateral_revision_m':float(np.abs(local[:,1]).max()) if aligned else None,
        'mean_signed_lateral_revision_m':float(local[:,1].mean()) if aligned else None,
        'endpoint_revision_m':float(norm[-1]) if aligned else None,'mean_abs_tangent_change_deg':aligned['tangent_mean_deg'] if aligned else None,
        'max_abs_tangent_change_deg':aligned['tangent_max_deg'] if aligned else None,'delta_v':float(v),'delta_w':float(w),
        'observation_to_application_sim_s':fresh['application']['sim_time']-ft,**l,
        **{k:seam[k] for k in ['raw_boundary_position_gap_m','raw_executed_to_fresh_tangent_gap_deg','raw_robot_heading_to_fresh_tangent_gap_deg']}}
    details={'old_remaining_world_xy':rp.tolist() if rp is not None else [],'old_remaining_times':rt.tolist() if rt is not None else [],
        'old_conflict':oc['first_conflict'],'fresh_world_xy':fw.tolist(),'raw_seam':seam,
        'alignment':{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in aligned.items()} if aligned else None}
    return row,details


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['run-dir','preflight-dir','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args();source=Path(a.preflight_dir).resolve();run=load_saved(a.run_dir)
    baseline=load_saved(ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01')
    frozen=json.loads((source/'freeze.json').read_text());cart=frozen['cart'];radius=frozen['robot_radius_m']
    out=validate_destination(a.output_dir,[source,run['path'],baseline['path']]);out.mkdir(parents=True)
    hashes=evidence_hashes([source,run['path'],baseline['path'],Path(__file__),ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'])
    visibility=[];poses=[]
    from PIL import Image
    for e in run['events']:
        rid=e['request_id'];name=Path(e['rgb_observation_reference']).name
        v=json.loads((run['path']/'raw/visibility'/(name+'.json')).read_text());v.update(request_id=rid,sim_time=e['observation']['sim_time'])
        mask_path=run['path']/'raw/visibility'/(name+'.mask.png');mask=np.asarray(Image.open(mask_path))>0
        if int(mask.sum())!=v['cart_visible_pixel_count']:raise ValueError('Saved mask count differs')
        rgb=np.asarray(Image.open(run['path']/e['rgb_observation_reference'])).copy();rgb[mask]=(rgb[mask]*.45+np.array([255,20,150])*.55).astype('uint8')
        (out/'overlays').mkdir(exist_ok=True);Image.fromarray(rgb).save(out/f'overlays/C{rid:02d}.png')
        v.update(rgb=str(run['path']/e['rgb_observation_reference']),mask=str(mask_path));visibility.append(v)
        obs=e['observation'];r=dict(zip(FIELDS,[rid,obs['sim_time'],*obs['pose'],float(np.degrees(obs['pose'][2]))]));poses.append(r)
        row=run['robot'][obs['tick']]
        assert obs['pose']==[float(row[k]) for k in ['x','y','yaw']]
    first_visible=next((v['request_id'] for v in visibility if v['visible']),None)
    first_clear=next((v['request_id'] for v in visibility if v['state']=='CLEAR'),None)
    pairs=[];details={}
    for old in range(1,len(run['events'])):
        row,d=pair(run,old,cart,radius);pairs.append(row);details[str(old)]=d
    primary_old=first_clear-1 if first_clear and first_clear>1 else None
    secondary=None
    if first_clear==1:
        secondary=next((r['old_request_id'] for r in pairs if r['old_predicts_collision'] and r['fresh_cart_clearance_m']>0),None)
    selected=primary_old or secondary;metric=pairs[selected-1] if selected else None;detail=details[str(selected)] if selected else None
    prior=visibility[max(0,(first_clear or 1)-4):(first_clear or 1)-1]
    hidden_history=len(prior)==3 and all(v['state']=='HIDDEN' for v in prior)
    actual=path_clearance(run['xy'],cart,radius,run['times']);cart_contacts=[c for c in run['contacts'] if COPY in json.dumps(c)]
    qualifies=bool(primary_old and hidden_history and metric['old_predicts_collision'] and metric['fresh_cart_clearance_m']>0 and
        metric['aligned_overlap_rmse_m'] is not None and metric['aligned_overlap_rmse_m']>1e-6 and metric['current_cart_clearance_m']>0 and
        metric['predicted_conflict_sim_time']>metric['application_sim_time'] and not cart_contacts)
    decision=('INSUFFICIENT EVIDENCE' if not run['complete'] or first_clear is None else
        'CART REVEAL TIMING NOT RECONCILIATION-RELEVANT' if not hidden_history else
        'RECONCILIATION-RELEVANT CART OLD/FRESH REVISION OBSERVED' if qualifies else 'CART VISIBLE BUT NO QUALIFYING FRESH REVISION')
    summary={'decision':decision,'run':str(run['path']),'complete':run['complete'],'first_visible_request_id':first_visible,'first_clear_request_id':first_clear,
        'visible_at_C1':visibility[0]['visible'],'three_prior_hidden':hidden_history,'primary_pair':[primary_old,first_clear] if primary_old else None,
        'secondary_pair':[secondary,secondary+1] if secondary else None,'selected_metrics':metric,'actual_cart_clearance':actual,
        'cart_contact_spans':cart_contacts,'all_nonfloor_contact_span_count':len(run['contacts']),
        'first_nonfloor_contact':run['contacts'][0] if run['contacts'] else None,
        'conflict_to_clear_pairs_descriptive_only':[[r['old_request_id'],r['fresh_request_id']] for r in pairs if r['old_predicts_collision'] and r['fresh_cart_clearance_m']>0],
        'interpretation':'Enclosing oriented visual/collision bounds + conservative robot disk; negative clearance is a proxy, contact is logged separately. One run, no causal navigation or reconciliation improvement claim.'}
    write_json(out/'summary.json',summary);write_json(out/'pair_metrics.json',pairs);write_json(out/'pair_details.json',details);write_json(out/'visibility.json',visibility)
    for name,rows in [('pair_metrics',pairs),('observation_poses',poses)]:
        if rows:
            with (out/(name+'.csv')).open('x') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    walls=wall_inventory(json.loads((ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json').read_text()))
    dense=np.array([[float(r[k]) for k in ['tick','sim_time','x','y','yaw']] for r in baseline['robot']])
    footprints={'cart':outline(cart).tolist(),'inflated':outline(cart,radius).tolist()};figinfo={}
    # The late run can cluster at a wall. Keep every measured marker, but label
    # the approach, primary event and final C instead of stacking 28 labels.
    labelled=[r for r in poses if r['request_id']<=21 or r['request_id'] in [selected,selected+1 if selected else None,len(poses)]]
    for zoom in [False,True]:
        fig,markers,info=draw_map(labelled,dense,walls,zoom);ax=fig.axes[0];markers.set_color('#20875b')
        all_points=np.array([[r['x_world_m'],r['y_world_m']] for r in poses])
        plotted=ax.scatter(*all_points.T,s=12,c='#20875b',edgecolors='white',linewidths=.3,zorder=5)
        np.testing.assert_array_equal(plotted.get_offsets(),all_points)
        for label in ax.texts:
            if label.get_text()=='C21':label.set_position((-45,-25))
            if label.get_text()==f'C{len(poses)}':label.set_position((-45,30))
        ax.plot(*run['xy'].T,color='#20875b',lw=2,zorder=3)
        for label in ax.texts:
            if label.get_text()=='Final':label.set_text('Baseline final')
        ax.scatter(*run['xy'][-1],marker='X',s=90,c='#20875b',zorder=7)
        ax.annotate('Cart-run final',run['xy'][-1],xytext=(-90,8),textcoords='offset points',color='#20875b',fontsize=9)
        for points,color,style in [(footprints['cart'],'#ac2580','-'),(footprints['inflated'],'#ac2580','--')]:ax.plot(*np.array(points).T,color=color,ls=style,lw=1.8,zorder=5)
        ax.scatter(*cart['center_world_xyz'][:2],marker='+',c='#ac2580',s=80,zorder=7)
        if selected:
            ax.plot(*np.array(detail['old_remaining_world_xy']).T,color='#e07419',lw=2.2,zorder=4)
            ax.plot(*np.array(detail['fresh_world_xy']).T,color='#6853ad',lw=2,zorder=4)
            for e,label,color,marker in [(run['events'][selected-1],'OLD','#e07419','s'),(run['events'][selected],'FRESH','#6853ad','D')]:
                ax.scatter(*e['observation']['position'][:2],facecolors='none',edgecolors=color,marker=marker,s=115,zorder=8)
            app=run['events'][selected]['application'];ax.scatter(*app['position'][:2],marker='x',s=70,c='black',zorder=9)
        for legend in fig.legends:legend.remove()
        handles=[Line2D([],[],color=c,label=l) for c,l in [('#216e9d','No-cart reference'),('#20875b','Actual cart run'),('#e07419','OLD remaining'),('#6853ad','FRESH'),('#ac2580','Cart footprint')]]
        handles += [Line2D([],[],color='#ac2580',ls='--',label='Inflated conflict region'),Line2D([],[],color='black',marker='x',ls='',label='Application'),Patch(fc='#e1e5e9',label='Structural bounds')]
        fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.90),ncol=4,frameon=False,fontsize=9)
        location='C26' if frozen.get('intervention')=='EXACT_C26' else 'C19'
        fig.suptitle('Hospital E16 | Fixed original cart at exact '+location+(' | turn zoom' if zoom else ''),y=.98,fontsize=16)
        for t in fig.texts:
            if t.get_position()[1]==.93:t.set_text(f'Actual first visible C{first_visible}; first CLEAR C{first_clear}; '+(f'pair C{selected} -> C{selected+1}' if selected else 'no adjacent primary pair'))
            if t.get_position()[1]==.03:t.set_text(decision+'\nAll C observation markers retained; labels C1–C21 and final. Tangents/timing are derived.');t.set_color('#923329')
        name='trajectory_turn_zoom' if zoom else 'trajectory_result';fig.savefig(out/(name+'.png'),dpi=220);fig.savefig(out/(name+'.pdf'));plt.close(fig);figinfo[name]=info
    if detail and detail['alignment']:
        align=detail['alignment'];t=np.array(align['absolute_times'])-metric['fresh_observation_sim_time'];delta=np.array(align['delta']);local=delta@rotation(run['events'][selected]['observation']['pose'][2])
        fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True);axes[0].plot(t,np.linalg.norm(delta,axis=1));axes[0].set_ylabel('Aligned displacement [m]')
        axes[1].plot(t,local[:,1],label='Signed left');axes[1].plot(t,local[:,0],label='Signed forward');axes[1].set(xlabel='Nominal horizon after FRESH [s]',ylabel='FRESH-frame revision [m]');axes[1].legend()
        fig.suptitle(f'Observed C{selected} -> C{selected+1} geometry | RMSE {metric["aligned_overlap_rmse_m"]:.4f} m\n'+decision,fontsize=11)
        for ax in axes:ax.grid(alpha=.25)
        fig.tight_layout();fig.savefig(out/'trajectory_revision.png',dpi=220);fig.savefig(out/'trajectory_revision.pdf');plt.close(fig)
    write_json(out/'trajectory_coordinates.json',{'cart':cart,'footprints':footprints,'reference_xy':baseline['xy'].tolist(),'actual_xy':run['xy'].tolist(),
        'actual_sim_times':run['times'].tolist(),'observations':poses,'selected_pair':metric,'selected_details':detail,'figures':figinfo})
    viewer=out/'viewer_data';viewer.mkdir();physical=json.loads((source/'physical.json').read_text());vb=physical['bypass']
    vc=[]
    for v in visibility:vc.append({**v,**{k.replace('cart_','human_'):val for k,val in v.items() if k.startswith('cart_')}})
    candidate={'id':1,'position':cart['center_world_xyz'],'visibility':vc,'strict_qualified':qualifies,'selection_rank':None,'failures':[] if qualifies else [decision],
        'old_request_id':selected,'fresh_request_id':selected+1 if selected else (first_clear or 1),'first_visible_request_id':first_visible,'first_clear_request_id':first_clear,
        'old_full_world_xy':run['worlds'][selected-1].tolist() if selected else [],
        'remaining_world_xy':detail['old_remaining_world_xy'] if detail else [],'fresh_world_xy':detail['fresh_world_xy'] if detail else [],
        'old_min_clearance_m':metric['old_remaining_cart_clearance_m'] if metric else None,
        'robot_current_clearance_at_reveal_m':metric['current_cart_clearance_m'] if metric else None,'conflict':detail['old_conflict'] if detail else None,
        'bypass':{'pass':bool(vb),'centerline_xy':vb['world_xy'] if vb else [],'clearance_m':vb['clearance_m'] if vb else None},'cart_outline':footprints['cart'],'inflated_outline':footprints['inflated']}
    cfg={'baseline_run':str(run['path']),'reference_baseline_run':str(baseline['path']),'baseline_analysis':str(out),
        'collision_inventory':str(ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'),
        'cart_frozen':frozen,'robot_radius_m':radius,'visibility':frozen['visibility_rules']}
    for name,value in [('preregistered.json',{'config':cfg}),('candidates.json',[candidate]),('summary.json',{'obstacle_type':'cart','selected_candidate_id':1 if qualifies else None,'diagnostic_candidate_id':1}),('source_evidence_sha256.json',hashes)]:write_json(viewer/name,value)
    verify_hashes(hashes);write_json(out/'metadata.json',{'source_evidence_sha256':hashes,'source_files_unchanged':True,**provenance(ROOT)})
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
