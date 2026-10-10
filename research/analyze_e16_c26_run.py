"""Actual authorized C26 cart run: first reveal, pre-contact revision, saved GUI."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_e16_c26 import BASELINE,PREVIOUS,csv_rows
from research.compare_e16_cart_shift import analyze_run
from research.cart_bypass import plot_coordinates
from research.cart_geometry import outline
from research.e16_c26_tools import instruction_audit,precontact
from research.hospital_spatial_gate import wall_inventory
from research.records import write_json,provenance
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['run-dir','preflight-dir','output-dir']:p.add_argument('--'+key,required=True)
    p.add_argument('--camera-dir',help='Saved static-ray audit at this actual run camera poses')
    a=p.parse_args();pre=Path(a.preflight_dir).resolve();out=validate_destination(a.output_dir,[pre,a.run_dir,BASELINE,*PREVIOUS]);out.mkdir(parents=True)
    runs={'baseline':load_saved(BASELINE),'exact_C19':load_saved(PREVIOUS[0]),'C19_plus_Y':load_saved(PREVIOUS[1]),'C26':load_saved(a.run_dir)}
    run=runs['C26'];frozen=json.loads((pre/'freeze.json').read_text());cart=frozen['cart'];radius=frozen['robot_radius_m']
    if run['cfg']['stationary_cart']!=frozen:raise ValueError('Cart differs from immutable preflight freeze')
    if {k:v for k,v in run['cfg'].items() if k!='stationary_cart'}!=runs['baseline']['cfg']:raise ValueError('Baseline settings changed')
    hashes=evidence_hashes([pre,*(r['path'] for r in runs.values()),Path(__file__)]+([Path(a.camera_dir)] if a.camera_dir else []))
    data=analyze_run(run,cart,radius);rows=data['pairs'];details=data['details'];vis=[]
    from PIL import Image
    from research.reveal_window import state
    for e in run['events']:
        rid=e['request_id'];name=Path(e['rgb_observation_reference']).name
        v=json.loads((run['path']/'raw/visibility'/(name+'.json')).read_text())
        mask_path=run['path']/'raw/visibility'/(name+'.mask.png');mask=np.asarray(Image.open(mask_path))>0
        if int(mask.sum())!=v['cart_visible_pixel_count']:raise ValueError('Actual mask count mismatch')
        stats={k.replace('cart_','human_'):value for k,value in v.items() if k.startswith('cart_')}
        if state(stats,frozen['visibility_rules'])!=v['state']:raise ValueError('Visibility classification mismatch')
        v.update(request_id=rid,sim_time=e['observation']['sim_time'],rgb=str(run['path']/e['rgb_observation_reference']),mask=str(mask_path));vis.append(v)
    visible=next((v['request_id'] for v in vis if v['visible']),None);clear=next((v['request_id'] for v in vis if v['state']=='CLEAR'),None)
    first_contact=min((c['start_sim_time'] for c in run['contacts']),default=None)
    for r in rows:
        i=r['old_request_id'];d=details[str(i)];e=run['events'][i]
        r['precontact']=precontact(r,first_contact)
        r['relevant_precontact_approach']=r['precontact'] and r['interaction_window']
        r['fresh_lookahead_native_left_m']=float(run['chunks'][i][e['lookahead_index'],1])
        r['fresh_lookahead_native_bearing_rad']=float(np.arctan2(run['chunks'][i][e['lookahead_index'],1],run['chunks'][i][e['lookahead_index'],0]))
        r['fresh_controller_w']=float(e['controller_command'][1])
        r['command_sign_matches_native_lookahead']=bool(np.sign(r['fresh_controller_w'])==np.sign(r['fresh_lookahead_native_left_m']))
        d.update(old_full_world_xy=run['worlds'][i-1].tolist(),old_native=run['chunks'][i-1].tolist(),fresh_native=run['chunks'][i].tolist(),
            old_observation=run['events'][i-1]['observation'],fresh_observation=e['observation'],application=e['application'])
    window=[r for r in rows if r['relevant_precontact_approach'] and r['max_abs_lateral_revision_m'] is not None]
    peak=max(window,key=lambda r:(r['max_abs_lateral_revision_m'],-r['old_request_id'])) if window else None
    improved=[r for r in rows if r['precontact'] and r['old_conflict_fresh_improved']]
    qualified=[r for r in improved if r['fresh_clear'] and r['current_cart_clearance_m']>0 and r['predicted_conflict_sim_time']>r['application_sim_time']]
    comparison=dict(data['comparison'])
    for key in ['max_lateral_revision_in_interaction_m','max_abs_delta_w_in_interaction']:
        comparison[key+'_including_postcontact']=comparison.pop(key)
    post=[r for r in rows if not r['precontact'] and r['max_abs_lateral_revision_m'] is not None]
    def at(rid):return rows[rid-2] if rid and rid>1 else None
    selected=peak['old_request_id'] if peak else (clear-1 if clear and clear>1 else 1)
    summary={'run':str(run['path']),'complete':run['complete'],'request_count':len(run['events']),'measured_pose_count':len(run['robot']),
        'physical_gate_override':run['metadata']['freeze_receipt']['explicit_user_authorization'],
        'first_visible_request_id':visible,'first_clear_request_id':clear,'visible_at_C1':vis[0]['visible'],
        'first_visible_pair':at(visible),'first_clear_pair':at(clear),'largest_precontact_approach_lateral_pair':peak,
        'selection_rule':'First visible/CLEAR reported independently. Inspection = largest maximum absolute lateral revision in existing cart approach window with observation AND application strictly before first non-floor contact. Descriptive only; no hard-case threshold.',
        'first_precontact_old_conflict_fresh_improvement':improved[0] if improved else None,
        'precontact_conflict_to_clear_pairs':[[r['old_request_id'],r['fresh_request_id']] for r in qualified],
        'outcome':data['outcome'],'comparison':comparison,'first_structural_contact':data['first_structural_contact'],
        'largest_postcontact_lateral_pair_excluded':max(post,key=lambda r:r['max_abs_lateral_revision_m']) if post else None,
        'first_nonfloor_contact_time':first_contact,'cart_contact_spans':data['cart_contacts'],'all_nonfloor_contact_span_count':len(run['contacts']),
        'precontact_side_counts':{side:sum(r['precontact'] and r['fresh_predicted_side']==side for r in rows) for side in sorted({r['fresh_predicted_side'] for r in rows})},
        'instruction_audit':instruction_audit(runs),
        'interpretation':'Actual single run with explicit bypass-gate exception. Post-contact pairs excluded from primary inspection; no graph/correspondence/reconciliation or causal performance validation.'}
    write_json(out/'summary.json',summary);write_json(out/'pair_metrics.json',rows);write_json(out/'pair_details.json',details)
    csv_rows(out/'pair_metrics.csv',rows);write_json(out/'visibility.json',vis)
    csv_rows(out/'visibility.csv',[{k:v[k] for k in ['request_id','sim_time','state','cart_visible_pixel_count','cart_visible_fraction','cart_bbox_if_visible']} for v in vis])
    coords=plot_coordinates(runs);coords['selected_pair']=selected;coords['cart']=cart
    coords['contact_points_world_xy']={name:[c['position'][:2] for span in r['contacts'] for c in span['first_report']['contacts']] for name,r in runs.items()}
    write_json(out/'trajectory_coordinates.json',coords)
    csv_rows(out/'observation_poses.csv',[{'request_id':r['request_id'],'sim_time':r['sim_time'],'x_world_m':r['pose'][0],'y_world_m':r['pose'][1],'yaw_rad':r['pose'][2]} for r in coords['C26']['observations']])
    plot(out,runs,cart,radius,details,summary,selected,window)
    if a.camera_dir:camera_figures(out,Path(a.camera_dir),vis,visible,clear)
    viewer=out/'viewer_data';viewer.mkdir();d=details[str(selected)];m=rows[selected-1];physical=json.loads((pre/'physical.json').read_text());b=physical['bypass']
    candidate={'id':1,'position':cart['center_world_xyz'],'visibility':[{**v,**{k.replace('cart_','human_'):x for k,x in v.items() if k.startswith('cart_')}} for v in vis],
        'strict_qualified':False,'selection_rank':None,'failures':['Explicit bypass-gate override; descriptive saved run'],
        'old_request_id':selected,'fresh_request_id':selected+1,'first_visible_request_id':visible,'first_clear_request_id':clear,
        'old_full_world_xy':d['old_full_world_xy'],'remaining_world_xy':d['old_remaining_world_xy'],'fresh_world_xy':d['fresh_world_xy'],
        'old_min_clearance_m':m['old_remaining_cart_clearance_m'],'robot_current_clearance_at_reveal_m':m['current_cart_clearance_m'],'conflict':d['old_conflict'],
        'bypass':{'pass':bool(b),'centerline_xy':b['world_xy'] if b else [],'clearance_m':b['clearance_m'] if b else None},
        'cart_outline':outline(cart).tolist(),'inflated_outline':outline(cart,radius).tolist(),'approach_sim_time':data['outcome']['interaction_start_s'],
        'closest_sim_time':data['outcome']['closest_approach']['sim_time'],'structural_contact_sim_time':first_contact}
    cfg={'baseline_run':str(run['path']),'reference_baseline_run':str(BASELINE),'baseline_analysis':str(out),
        'collision_inventory':str(ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'),
        'cart_frozen':frozen,'robot_radius_m':radius,'visibility':frozen['visibility_rules'],'cart_event_jumps':True,'secondary_cart_runs':[str(p) for p in PREVIOUS]}
    for name,value in [('preregistered.json',{'config':cfg}),('candidates.json',[candidate]),('summary.json',{'obstacle_type':'cart','selected_candidate_id':None,'diagnostic_candidate_id':1}),('source_evidence_sha256.json',hashes)]:write_json(viewer/name,value)
    verify_hashes(hashes);write_json(out/'metadata.json',{'source_evidence_sha256':hashes,'source_files_unchanged':True,**provenance(ROOT)})
    print(json.dumps({k:summary[k] for k in ['run','request_count','first_visible_request_id','first_clear_request_id','comparison','largest_precontact_approach_lateral_pair','precontact_conflict_to_clear_pairs']},indent=2))


def plot(out,runs,cart,radius,details,summary,selected,window):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'pdf.fonttype':42,'font.size':10})
    walls=wall_inventory(json.loads((ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json').read_text()));run=runs['C26']
    def context(ax,bounds):
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.15)
        for w in walls:
            lo,hi=np.array(w['world_aabb_min']),np.array(w['world_aabb_max'])
            if hi[0]<bounds[0] or lo[0]>bounds[1] or hi[1]<bounds[2] or lo[1]>bounds[3]:continue
            ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],fc='#e4e7ea',ec='#b5bdc4',lw=.4,zorder=0))
        ax.plot(*outline(cart).T,c='#bc277b',lw=2,label='Fixed C26 cart');ax.plot(*outline(cart,radius).T,c='#bc277b',ls='--',lw=1.2,label='Inflated proxy')
    def save(fig,stem):
        fig.savefig(out/(stem+'.png'),dpi=190);fig.savefig(out/(stem+'.pdf'));plt.close(fig)
    for name,bounds in [('trajectory_result',[-22,9.5,0,14.5]),('trajectory_turn_zoom',[-7,9.3,5,14])]:
        combined=np.vstack([r['xy'] for r in runs.values()])
        if name=='trajectory_result':bounds=[min(-22,combined[:,0].min()-1),max(9.5,combined[:,0].max()+1),min(0,combined[:,1].min()-1),max(14.5,combined[:,1].max()+1)]
        fig,ax=plt.subplots(figsize=(16,8.6));context(ax,bounds)
        for key,color,label in [('baseline','#467d98','No-cart baseline'),('exact_C19','#baa477','Prior exact C19'),('C19_plus_Y','#9cb3a5','Prior C19 +0.40Y'),('C26','#159b60','Actual C26 cart run')]:
            r=runs[key];ax.plot(*r['xy'].T,c=color,lw=2.2 if key=='C26' else 1.1,label=label)
            if r['contacts']:
                xy=np.array([c['position'][:2] for span in r['contacts'] for c in span['first_report']['contacts']])
                if len(xy):ax.scatter(*xy.T,c=color,marker='x',s=18,alpha=.65,label=key+' reported contact points')
        obs=np.array([e['agent_pose_at_observation'] for e in run['events']]);artist=ax.scatter(*obs[:,:2].T,s=23,c='#159b60',edgecolors='white',lw=.4,zorder=6)
        np.testing.assert_array_equal(artist.get_offsets(),obs[:,:2]);last=None
        for i,(x,y,yaw) in enumerate(obs,1):
            if not (bounds[0]<=x<=bounds[1] and bounds[2]<=y<=bounds[3]):continue
            if last is not None and np.linalg.norm([x-last[0],y-last[1]])<.4 and i not in [selected,selected+1,len(obs)]:continue
            last=[x,y];sign=1 if i%2 else -1;off=(-np.sin(yaw)*sign*22,np.cos(yaw)*sign*22)
            if i==selected:off=(-40,-35)
            if i==selected+1:off=(35,35)
            ax.annotate(f'C{i}',(x,y),xytext=off,textcoords='offset points',ha='center',fontsize=9,bbox=dict(fc='white',ec='none',alpha=.9,pad=.5),arrowprops=dict(arrowstyle='-',lw=.4))
        ax.scatter(*run['xy'][0],marker='s',c='#159b60',s=65,label='C26 run start');ax.scatter(*run['xy'][-1],marker='X',c='#159b60',s=90,label='C26 run final',zorder=7)
        ax.set_title('Hospital E16 | Actual fixed C26 cart run | '+summary['outcome']['actual_bypass_side'],pad=17,fontsize=14)
        ax.legend(loc='upper center',bbox_to_anchor=(.5,-.08),ncol=4,fontsize=9,frameon=False)
        fig.text(.5,.025,'All actual C observation markers retained; labels spaced at stalls. Bypass-gate exception explicitly authorized; no placement/controller tuning.',ha='center',fontsize=10)
        fig.subplots_adjust(left=.055,right=.98,top=.91,bottom=.22);save(fig,name)
    reveal_ids={r['old_request_id'] for r in [summary['first_visible_pair'],summary['first_clear_pair']] if r}
    ids=sorted(reveal_ids|{selected}|{r['old_request_id'] for r in window})
    for i in ids:
        d=details[str(i)];ow=np.array(d['old_full_world_xy']);fw=np.array(d['fresh_world_xy']);rp=np.array(d['old_remaining_world_xy']);oe=d['old_observation'];fe=d['fresh_observation']
        points=np.vstack([ow,fw,oe['position'][:2],fe['position'][:2],outline(cart,radius)]);lo,hi=points.min(0)-.8,points.max(0)+.8
        fig,ax=plt.subplots(figsize=(13.5,7.8));context(ax,[lo[0],hi[0],lo[1],hi[1]])
        ax.plot(*run['xy'].T,c='#159b60',lw=1.3,alpha=.7,label='Actual C26 executed path')
        ax.plot(*ow.T,c='#ca8b4c',ls='--',lw=2,label=f'OLD C{i} FULL (30)');ax.plot(*rp.T,c='#e47718',lw=2.5,label='OLD remaining at FRESH obs');ax.plot(*fw.T,c='#7541a5',lw=2,label=f'FRESH C{i+1} FULL (30)')
        for e,label,col,marker in [(oe,'OLD observation','#e47718','s'),(fe,'FRESH observation','#7541a5','D'),(d['application'],'Actual switch B','black','x')]:ax.scatter(*e['position'][:2],s=85,c=col,marker=marker,zorder=8,label=label)
        ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),frameon=False,fontsize=10)
        role='Largest pre-contact approach lateral revision' if i==selected and summary['largest_precontact_approach_lateral_pair'] else 'First reveal' if i in reveal_ids else 'Pre-contact approach'
        ax.set_title(f'Actual C{i} -> C{i+1} | {role}',fontsize=13,pad=16)
        fig.text(.5,.04,'Native curves use their own observation anchors; nominal timing and tangents are derived. No correction or correspondence.',ha='center',fontsize=10)
        fig.subplots_adjust(left=.075,right=.73,top=.88,bottom=.15);save(fig,'old_fresh_pair_comparison' if i==selected else f'pair_C{i:02d}_C{i+1:02d}')
    d=details[str(selected)];a=d['alignment']
    if a:
        from research.cart_geometry import rotation
        t=np.array(a['absolute_times'])-d['fresh_observation']['sim_time'];delta=np.array(a['delta']);local=delta@rotation(d['fresh_observation']['pose'][2])
        fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True);axes[0].plot(t,np.linalg.norm(delta,axis=1));axes[0].set_ylabel('Aligned displacement [m]')
        axes[1].plot(t,local[:,0],label='Signed forward');axes[1].plot(t,local[:,1],label='Signed left');axes[1].legend();axes[1].set(xlabel='Nominal horizon after FRESH [s]',ylabel='Revision in FRESH frame [m]')
        for ax in axes:ax.grid(alpha=.2)
        fig.suptitle(f'Actual C26 cart run | C{selected} -> C{selected+1} | pre-contact inspection',fontsize=13);fig.tight_layout(rect=(0,.03,1,.94));save(fig,'trajectory_revision')


def camera_figures(out,source,vis,visible,clear):
    import matplotlib.pyplot as plt
    from PIL import Image
    projections=json.loads((source/'projections.json').read_text());rows=[]
    for row in projections:
        for name,g in row['groups'].items():rows.append({'request_id':row['request_id'],'group':name,'inside_fov':g['inside_count'],'unoccluded_samples':g['unoccluded_count'],'sample_count':g['sample_count']})
    csv_rows(out/'camera_samples.csv',rows)
    for rid in sorted({max(1,(visible or 2)-1),visible or 1,clear or 1,24}):
        v=vis[rid-1];rgb=np.array(Image.open(v['rgb']));mask=np.asarray(Image.open(v['mask']))>0
        overlay=rgb.copy();overlay[mask]=(overlay[mask]*.45+np.array([255,20,150])*.55).astype('uint8')
        fig,axes=plt.subplots(1,2,figsize=(18,6));axes[0].imshow(rgb);axes[0].set_title('Original archived model RGB')
        axes[1].imshow(overlay);axes[1].set_title('Cart semantic mask + candidate-passage samples')
        for name,color in [('north','#ffc035'),('south','#32ed88')]:
            g=projections[rid-1]['groups'][name];uv=np.array(g['pixels']);inside=np.array(g['inside_fov']);free=np.array(g['unoccluded_inside_fov'])
            axes[1].scatter(*uv[inside&free].T,c=color,marker='o',s=20,label=name+' clear rays')
            axes[1].scatter(*uv[inside&~free].T,c=color,marker='x',s=22,label=name+' blocked rays')
        for ax in axes:ax.set(xlim=(0,1920),ylim=(1080,0));ax.axis('off')
        axes[1].legend(loc='lower right',fontsize=8)
        fig.suptitle(f'Actual C{rid} | {v["state"]}, {v["cart_visible_pixel_count"]} cart pixels | FOV/rays do not prove traversability',fontsize=12)
        fig.tight_layout();fig.savefig(out/f'camera_C{rid:02d}.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
