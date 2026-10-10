"""Saved-only C26 preflight stop report; never labels baseline as cart response."""
import argparse,csv,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.e16_cart_preflight import BASELINE
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_e16_cart import pair
from research.e16_c26_tools import instruction_audit,preflight_gate
from research.cart_geometry import outline,path_clearance
from research.cart_bypass import plot_coordinates,closest_approach
from research.hospital_spatial_gate import wall_inventory
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.records import write_json,provenance

PREVIOUS=[ROOT/'outputs/nova-e16-cart-20261010-01',ROOT/'outputs/nova-e16-cart-plus-y-20261010-01']
CONTEXT='NO-CART baseline predictions evaluated against inserted C26 cart; no cart-conditioned response'


def csv_rows(path,rows):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def analyze(preflight,camera_dir,out):
    preflight,camera_dir=Path(preflight).resolve(),Path(camera_dir).resolve()
    out=validate_destination(out,[BASELINE,*PREVIOUS,preflight,camera_dir]);out.mkdir(parents=True)
    runs={'baseline':load_saved(BASELINE),'exact_C19':load_saved(PREVIOUS[0]),'C19_plus_Y':load_saved(PREVIOUS[1])};run=runs['baseline']
    hashes=evidence_hashes([BASELINE,*PREVIOUS,preflight,camera_dir,ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'])
    frozen=json.loads((preflight/'freeze.json').read_text());physical=json.loads((preflight/'physical.json').read_text());vis=json.loads((preflight/'visibility.json').read_text())
    gate=preflight_gate(physical,vis,run)
    if gate['model_run_permitted']:raise ValueError('This report is exclusively preflight-stop; do not suppress an authorized passing run')
    cart=frozen['cart'];radius=frozen['robot_radius_m'];pairs=[];details={}
    for old_id in range(1,len(run['events'])):
        row,d=pair(run,old_id,cart,radius);old,fresh=run['events'][old_id-1:old_id+1]
        row['evidence_context']=CONTEXT
        row['predicted_conflict_lead_from_fresh_s']=row['predicted_conflict_sim_time']-row['fresh_observation_sim_time'] if row['predicted_conflict_sim_time'] is not None else None
        row['baseline_application_to_predicted_conflict_s']=row['predicted_conflict_sim_time']-row['application_sim_time'] if row['predicted_conflict_sim_time'] is not None else None
        d.update(old_native=run['chunks'][old_id-1].tolist(),fresh_native=run['chunks'][old_id].tolist(),
            old_full_world_xy=run['worlds'][old_id-1].tolist(),old_observation=old['observation'],fresh_observation=fresh['observation'],
            baseline_application=fresh['application'],evidence_context=CONTEXT)
        pairs.append(row);details[str(old_id)]=d
    first_conflict=next((r for r in pairs if r['old_predicts_collision'] and r['current_cart_clearance_m']>0 and r['baseline_application_to_predicted_conflict_s']>0),None)
    selected=first_conflict['old_request_id'] if first_conflict else 23
    first_visible=next((v['request_id'] for v in vis if v['visible']),None);first_clear=next((v['request_id'] for v in vis if v['state']=='CLEAR'),None)
    reveal_metrics={str(rid):pairs[rid-2] for rid in sorted({first_visible,first_clear}-{None,1})}
    instructions=instruction_audit(runs);instructions['C26_runtime']='NOT RUN: physical preflight stop; no fourth runtime instruction exists'
    instructions['planned_C26_instruction_equal']=True
    sources=plot_coordinates(runs)
    request_conflicts=[]
    for e,world in zip(run['events'],run['worlds']):
        c=path_clearance(world,cart,radius,e['observation']['sim_time']+.1*np.arange(1,31))
        request_conflicts.append({'request_id':e['request_id'],'observation_sim_time':e['observation']['sim_time'],**c})
    closest=closest_approach(run['xy'],run['times'],cart,radius)
    summary={'decision':'PREFLIGHT ONLY - NO INFERENCE','gate':gate,'C26_observation':frozen['C26_observation'],
        'frozen_cart':cart,'physical':{k:v for k,v in physical.items() if k not in ['bypass_checks','floor_checks']},
        'first_visible_request_id':first_visible,'first_clear_request_id':first_clear,'C1_pixels':vis[0]['cart_visible_pixel_count'],
        'reveal_baseline_pairs':reveal_metrics,'most_relevant_baseline_conflict_pair':first_conflict,
        'baseline_conflict_selection':'Earliest adjacent OLD remaining conflict with positive current separation and baseline application preceding nominal conflict; no cart response claim',
        'instruction_audit':instructions,'actual_C26_run':None,'actual_bypass':None,'actual_first_contact':None,
        'cart_conditioned_precontact_revision':None,'qualifying_reconciliation_pair':False,'model_calls':0,
        'interpretation':'Later reveal than C19, but physical gate fails. Baseline hypothetical conflicts cannot establish FRESH obstacle response, direction bias, or reconciliation relevance.'}
    write_json(out/'summary.json',summary);write_json(out/'pair_metrics.json',pairs);write_json(out/'pair_details.json',details)
    write_json(out/'request_conflicts.json',request_conflicts);write_json(out/'instruction_audit.json',instructions)
    csv_rows(out/'pair_metrics.csv',pairs)
    poses=[{'request_id':e['request_id'],'sim_time':e['observation']['sim_time'],'x_world_m':e['agent_pose_at_observation'][0],
        'y_world_m':e['agent_pose_at_observation'][1],'yaw_rad':e['agent_pose_at_observation'][2]} for e in run['events']]
    csv_rows(out/'observation_poses.csv',poses)
    csv_rows(out/'executed_baseline_path.csv',[{'sim_time':float(t),'x_world_m':float(x),'y_world_m':float(y)} for t,(x,y) in zip(run['times'],run['xy'])])
    for v,e in zip(vis,run['events']):
        from PIL import Image
        mask=np.asarray(Image.open(preflight/f"visibility/C{v['request_id']:02d}.mask.png"))>0
        if int(mask.sum())!=v['cart_visible_pixel_count']:raise ValueError('Mask count mismatch')
        v.update(rgb=str(preflight/f"visibility/C{v['request_id']:02d}.png"),mask=str(preflight/f"visibility/C{v['request_id']:02d}.mask.png"),
            baseline_rgb=str(run['path']/e['rgb_observation_reference']))
    write_json(out/'visibility.json',vis)
    csv_rows(out/'visibility.csv',[{k:v[k] for k in ['request_id','sim_time','state','cart_visible_pixel_count','cart_visible_fraction','cart_bbox_if_visible']} for v in vis])
    make_figures(out,runs,cart,radius,details,pairs,selected,sources)
    camera_figures(out,camera_dir,vis,first_visible,first_clear)
    d=details[str(selected)];m=pairs[selected-1]
    candidate={'id':1,'position':cart['center_world_xyz'],'visibility':[{**v,**{k.replace('cart_','human_'):x for k,x in v.items() if k.startswith('cart_')}} for v in vis],
        'strict_qualified':False,'selection_rank':None,'failures':gate['stop_reasons'],'old_request_id':selected,'fresh_request_id':selected+1,
        'first_visible_request_id':first_visible,'first_clear_request_id':first_clear,
        'old_full_world_xy':d['old_full_world_xy'],'remaining_world_xy':d['old_remaining_world_xy'],'fresh_world_xy':d['fresh_world_xy'],
        'old_min_clearance_m':m['old_remaining_cart_clearance_m'],'robot_current_clearance_at_reveal_m':m['current_cart_clearance_m'],
        'conflict':d['old_conflict'],'reveal_lead_s':m['predicted_conflict_lead_from_fresh_s'],
        'switch_to_conflict_margin_s':m['baseline_application_to_predicted_conflict_s'],
        'bypass':{'pass':False,'centerline_xy':[],'clearance_m':None},'cart_outline':outline(cart).tolist(),'inflated_outline':outline(cart,radius).tolist(),
        'approach_sim_time':run['events'][22]['observation']['sim_time'],'closest_sim_time':closest['sim_time'],'structural_contact_sim_time':None}
    viewer=out/'viewer_data';viewer.mkdir()
    cfg={'baseline_run':str(BASELINE),'reference_baseline_run':str(BASELINE),'baseline_analysis':str(out),
        'collision_inventory':str(ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'),
        'cart_frozen':frozen,'robot_radius_m':radius,'visibility':frozen['visibility_rules'],'cart_event_jumps':True,
        'secondary_cart_runs':[str(p) for p in PREVIOUS]}
    for name,value in [('preregistered.json',{'config':cfg}),('candidates.json',[candidate]),
        ('summary.json',{'obstacle_type':'cart','preflight_only':True,'selected_candidate_id':None,'diagnostic_candidate_id':1}),('source_evidence_sha256.json',hashes)]:write_json(viewer/name,value)
    verify_hashes(hashes);write_json(out/'metadata.json',{'source_evidence_sha256':hashes,'source_files_unchanged':True,'model_calls':0,
        'native_convention':'Forward/left metres at own observation pose; timing assigned +0.1..3s, no native timestamp or yaw',**provenance(ROOT)})
    print(json.dumps({k:summary[k] for k in ['decision','first_visible_request_id','first_clear_request_id','most_relevant_baseline_conflict_pair']},indent=2))


def make_figures(out,runs,cart,radius,details,pairs,selected,sources):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'pdf.fonttype':42,'font.size':10})
    walls=wall_inventory(json.loads((ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json').read_text()))
    run=runs['baseline'];coords={'context':CONTEXT,'runs':sources,'cart':cart,'outline':outline(cart).tolist(),
        'inflation':outline(cart,radius).tolist(),'selected_pair':selected,'pair_details':details,'contact_points':{}}
    def context(ax,bounds):
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.15)
        for w in walls:
            lo,hi=np.array(w['world_aabb_min']),np.array(w['world_aabb_max'])
            if hi[0]<bounds[0] or lo[0]>bounds[1] or hi[1]<bounds[2] or lo[1]>bounds[3]:continue
            ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],fc='#e4e7ea',ec='#b5bdc4',lw=.4,zorder=0))
        ax.plot(*outline(cart).T,color='#bc277b',lw=2,label='Fixed C26 cart')
        ax.plot(*outline(cart,radius).T,color='#bc277b',lw=1.3,ls='--',label='Inflated proxy')
    def save(fig,name):
        fig.savefig(out/(name+'.png'),dpi=190);fig.savefig(out/(name+'.pdf'));plt.close(fig)
    for name,bounds in [('trajectory_result',[-22,9.5,0,14.5]),('trajectory_turn_zoom',[-6,9.3,5.,14.])]:
        fig,ax=plt.subplots(figsize=(16,8.6));context(ax,bounds)
        for key,color,label in [('baseline','#1477a5','No-cart actual baseline'),('exact_C19','#a79064','Prior exact C19 (secondary)'),('C19_plus_Y','#749783','Prior C19 +0.40Y (secondary)')]:
            r=runs[key];ax.plot(*r['xy'].T,color=color,lw=2 if key=='baseline' else 1,alpha=1 if key=='baseline' else .8,label=label)
            contacts=[]
            for c in r['contacts']:
                i=int(np.searchsorted(r['times'],c['start_sim_time']));contacts.append(r['xy'][min(i,len(r['xy'])-1)].tolist())
            coords['contact_points'][key]=contacts
            if contacts:ax.scatter(*np.array(contacts).T,marker='x',s=14,c=color,label=key+' contact robot poses',alpha=.55)
        obs=np.array([e['agent_pose_at_observation'] for e in run['events']]);plotted=ax.scatter(*obs[:,:2].T,s=20,c='#e97521',edgecolors='white',linewidths=.4,zorder=5)
        np.testing.assert_array_equal(plotted.get_offsets(),obs[:,:2])
        for i,(x,y,yaw) in enumerate(obs,1):
            if not (bounds[0]<=x<=bounds[1] and bounds[2]<=y<=bounds[3]):continue
            off=(-np.sin(yaw)*(1 if i%2 else -1)*22,np.cos(yaw)*(1 if i%2 else -1)*22)
            if i==1:off=(-24,-24)
            if i==2:off=(27,-8)
            ax.annotate(f'C{i}',(x,y),xytext=off,textcoords='offset points',fontsize=9,ha='center',bbox=dict(fc='white',ec='none',pad=.5,alpha=.8),arrowprops=dict(arrowstyle='-',lw=.4))
        for xy,label,marker in [(run['xy'][0],'Baseline start','s'),(run['xy'][-1],'Baseline final','X')]:ax.scatter(*xy,s=90,marker=marker,c='#1477a5',label=label,zorder=6)
        ax.set_title('Hospital E16 | Exact C26 cart preflight STOP | No new executed cart path',pad=18,fontsize=15)
        ax.legend(loc='upper center',bbox_to_anchor=(.5,-.09),ncol=4,fontsize=9,frameon=False)
        fig.text(.5,.03,'C markers = saved baseline observation poses. Previous contacts belong only to previous cart runs. Cart placement unchanged.',ha='center',fontsize=10)
        fig.subplots_adjust(left=.055,right=.98,top=.91,bottom=.20);save(fig,name)
    selected_ids=sorted(set([selected,10,11,*range(23,29)]))
    for old_id in selected_ids:
        d=details[str(old_id)];m=pairs[old_id-1];ow=np.array(d['old_full_world_xy']);fw=np.array(d['fresh_world_xy']);rp=np.array(d['old_remaining_world_xy'])
        oe,fe=d['old_observation'],d['fresh_observation'];points=np.vstack([ow,fw,oe['position'][:2],fe['position'][:2],outline(cart,radius)])
        lo,hi=points.min(0)-.9,points.max(0)+.9;bounds=[lo[0],hi[0],lo[1],hi[1]]
        fig,ax=plt.subplots(figsize=(13.5,7.8));context(ax,bounds)
        ax.plot(*run['xy'].T,c='#1477a5',lw=1.2,alpha=.6,label='Actual no-cart path')
        ax.plot(*ow.T,c='#cb7c30',ls='--',lw=2,label=f'OLD C{old_id} FULL (30)')
        ax.plot(*rp.T,c='#e27616',lw=2.6,label='OLD remaining at FRESH obs')
        ax.plot(*fw.T,c='#7040a0',lw=2,label=f'FRESH C{old_id+1} FULL (30)')
        for e,label,color,marker in [(oe,'OLD observation','#e27616','s'),(fe,'FRESH observation','#7040a0','D'),(d['baseline_application'],'Baseline switch B','black','x')]:
            ax.scatter(*e['position'][:2],marker=marker,s=85,c=color,label=label,zorder=8)
        ax.set_title(f'No-cart C{old_id} -> C{old_id+1} | aligned RMSE {m["aligned_overlap_rmse_m"]:.3f} m',pad=16,fontsize=14)
        ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),frameon=False,fontsize=10)
        ax.text(1.02,.40,f'OLD proxy: {m["old_remaining_cart_clearance_m"]:+.3f} m\nFRESH proxy: {m["fresh_cart_clearance_m"]:+.3f} m\nCurrent proxy: {m["current_cart_clearance_m"]:+.3f} m\n\nNot cart-conditioned.\nNo new model run.',transform=ax.transAxes,fontsize=10,va='top')
        fig.text(.5,.045,CONTEXT+'\nTiming +0.1..3.0 s is derived; native arrays are unchanged forward/left points.',ha='center',fontsize=10)
        fig.subplots_adjust(left=.075,right=.73,top=.88,bottom=.15)
        save(fig,'old_fresh_pair_comparison' if old_id==selected else f'pair_C{old_id:02d}_C{old_id+1:02d}')
    d=details[str(selected)];m=pairs[selected-1];a=d['alignment'];h=np.array(a['absolute_times'])-m['fresh_observation_sim_time'];delta=np.array(a['delta'])
    from research.cart_geometry import rotation
    local=delta@rotation(d['fresh_observation']['pose'][2]);fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    axes[0].plot(h,np.linalg.norm(delta,axis=1),color='#1477a5');axes[0].set_ylabel('Aligned displacement [m]')
    axes[1].plot(h,local[:,0],label='Signed forward');axes[1].plot(h,local[:,1],label='Signed left');axes[1].legend();axes[1].set(xlabel='Nominal horizon after FRESH [s]',ylabel='Revision in FRESH frame [m]')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle(f'No-cart C{selected} -> C{selected+1} revision | hypothetical conflict inspection',fontsize=13)
    fig.text(.5,.02,'This is baseline replanning variation, not a response to the inserted cart.',ha='center');fig.tight_layout(rect=(0,.05,1,.95));save(fig,'trajectory_revision')
    write_json(out/'trajectory_coordinates.json',coords)


def camera_figures(out,camera_dir,vis,first_visible,first_clear):
    import matplotlib.pyplot as plt
    from PIL import Image
    projections=json.loads((camera_dir/'projections.json').read_text());counts=[]
    ids=sorted(set([max(1,first_visible-1),first_visible,first_clear,23]))
    for row in projections:
        for name,g in row['groups'].items():counts.append({'request_id':row['request_id'],'group':name,'inside_fov':g['inside_count'],'unoccluded_inside_fov':g['unoccluded_count'],'samples':g['sample_count']})
    csv_rows(out/'camera_sample_counts.csv',counts)
    for rid in ids:
        v=vis[rid-1];row=projections[rid-1];fig,axes=plt.subplots(1,2,figsize=(18,6))
        for ax,key,label in [(axes[0],'baseline_rgb','Archived original no-cart model RGB'),(axes[1],'rgb','Saved-pose cart preflight RGB')]:
            ax.imshow(Image.open(v[key]));ax.set(xlim=(0,1920),ylim=(1080,0));ax.axis('off');ax.set_title(label)
            for name,color in [('cart','#ff30b2'),('south','#32ed88'),('north','#ffc035')]:
                g=row['groups'][name];uv=np.array(g['pixels']);inside=np.array(g['inside_fov']);clear=np.array(g['unoccluded_inside_fov'])
                ax.scatter(*uv[inside&clear].T,c=color,s=16,marker='o',label=name+' ray clear')
                ax.scatter(*uv[inside&~clear].T,c=color,s=24,marker='x',label=name+' ray blocked')
        axes[1].legend(loc='lower right',fontsize=8)
        fig.suptitle(f'C{rid} | cart {v["state"]}, {v["cart_visible_pixel_count"]} pixels | projections are geometric samples, not a visibility oracle',fontsize=12)
        fig.tight_layout();fig.savefig(out/f'camera_C{rid:02d}.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['preflight-dir','camera-dir','output-dir']:p.add_argument('--'+k,required=True)
    a=p.parse_args();analyze(a.preflight_dir,a.camera_dir,a.output_dir)
