"""Saved-only scientific figures; all plotted coordinates are exported separately."""
import json
import subprocess
import numpy as np
from research.e16_curve_fitting import methods
from research.route_geometry import wrap

COLORS={'RAW':'#333333','SPLINE_01cm':'#e49c16','SPLINE_03cm':'#087fa1','SPLINE_05cm':'#ac3b67','POLY3_DIAGNOSTIC':'#7754a3'}


def figures(out,chunk_rows,pair_rows,sens,details,cfg):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'pdf.fonttype':42})
    names=[m for m,_ in methods(cfg)];fitted=names[1:];primary='SPLINE_03cm';manifest={}
    def save(fig,name,sources):
        fig.tight_layout();fig.savefig(out/'figures'/(name+'.png'),dpi=170,bbox_inches='tight')
        fig.savefig(out/'figures'/(name+'.pdf'),bbox_inches='tight');plt.close(fig);manifest[name]=sources
    def chunk(run,rid):return json.loads((out/'chunks'/f'{run}_C{rid:02d}.json').read_text())
    def boxes(ax,rows,key,ms=names):
        arrays=[[r[key] for r in rows if r['method']==m and r.get(key) is not None] for m in ms]
        labels=[m.replace('SPLINE_','').replace('_DIAGNOSTIC','')+f'\nn={len(a)}' for m,a in zip(ms,arrays)]
        b=ax.boxplot(arrays,labels=labels,patch_artist=True)
        for box,m in zip(b['boxes'],ms):box.set(facecolor=COLORS[m],alpha=.55)
        ax.tick_params(axis='x',labelsize=8)
    # Representative chunk identities frozen before fitting; no selection by outcome.
    fig,axes=plt.subplots(2,2,figsize=(13,10))
    for ax,(run,rid) in zip(axes.flat,cfg['representative_chunks']):
        d=chunk(run,rid);native=np.array(d['native_local_xy'])
        ax.plot(*native.T,'o:',c=COLORS['RAW'],ms=3,label='Native points (prediction)')
        for m in fitted:
            if m in d['fits']:ax.plot(*np.array(d['fits'][m]['grid_local_xy']).T,c=COLORS[m],lw=1.5,ls='--' if m.startswith('POLY') else '-',label=m)
        ax.scatter(0,0,s=60,facecolors='none',edgecolors='black',label='Observation origin')
        ax.set(xlabel='Observation-local forward [m]',ylabel='Observation-local left [m]',title=f'{run} C{rid} | observation phase: {d["phase"]}')
        ax.set_aspect('equal');ax.legend(fontsize=7)
    fig.suptitle('Independent native-chunk fits; origin is not a fitting datum or native waypoint')
    save(fig,'native_and_fitted_chunks',cfg['representative_chunks'])
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    for ax,key,label in zip(axes.flat,['forward_rmse_m','lateral_rmse_m','forward_mad_m','lateral_mad_m'],['Tangent residual RMSE [m]','Normal residual RMSE [m]','Tangent residual MAD x1.4826 [m]','Normal residual MAD x1.4826 [m]']):
        boxes(ax,chunk_rows,key,fitted);ax.set_ylabel(label)
    fig.suptitle('96 chunks from two runs: fitting residual scales, not verified model noise')
    save(fig,'fitting_residual_distributions',['chunk_metrics.json'])
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    for ax,key,label in zip(axes.flat,['forward_rmse_m','lateral_rmse_m','xy_rmse_m','relative_arc_length_change'],['Tangent RMSE [m]','Normal RMSE [m]','XY displacement RMSE [m]','Relative arc-length change']):
        for run,style in [('A','-'),('B','--')]:
            for group,color in [('straight','#176b93'),('left_turn','#bd533f')]:
                data=[]
                for m in fitted[:3]:
                    a=[r.get(key) for r in chunk_rows if r['method']==m and r['run']==run and (r['phase'] in ['initial_travel','hallway_straight'] if group=='straight' else r['phase']=='left_turn') and r.get(key) is not None]
                    data.append(np.median(a) if a else np.nan)
                ax.plot([1,3,5],data,'o'+style,c=color,label=f'{run} {group}')
        ax.set(xlabel='Prespecified per-coordinate RMS budget [cm]',ylabel=label);ax.legend(fontsize=8)
    fig.suptitle('Phase and smoothing sensitivity; straight/turn use saved observation context')
    save(fig,'residual_scale_sensitivity',['chunk_metrics.json','statistics.csv'])
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    raw={r['pair_id']:r for r in pair_rows if r['method']=='RAW'}
    for ax,m in zip(axes,names[1:4]):
        rows=[r for r in pair_rows if r['method']==m and r.get('boundary_position_m') is not None]
        x=[raw[r['pair_id']]['boundary_position_m'] for r in rows];y=[r['boundary_position_m'] for r in rows]
        ax.scatter(x,y,c=['#bd533f' if r['phase']=='left_turn' else '#176b93' for r in rows],s=23,alpha=.65)
        lim=max(x+y)*1.06;ax.plot([0,lim],[0,lim],'--',c='gray');ax.set(xlim=(0,lim),ylim=(0,lim),xlabel='RAW seam [m]',ylabel='Fitted seam [m]',title=m+f' | n={len(rows)}');ax.set_aspect('equal')
    fig.suptitle('Same measured switch B; smoothing is not boundary alignment (red=left turn)')
    save(fig,'boundary_position',['pair_metrics.json'])
    fig,axes=plt.subplots(1,2,figsize=(15,5))
    normal=[r for r in pair_rows if not r['bootstrap']]
    for ax,key,label,chordkey in zip(axes,['boundary_robot_heading_deg','boundary_executed_heading_deg'],['Geometric tangent vs measured robot yaw [deg]','Geometric tangent vs valid executed direction [deg]'],['raw_graph_chord_robot_heading_deg','raw_graph_chord_executed_heading_deg']):
        arrays=[[r.get(key) for r in normal if r['method']==m and r.get(key) is not None] for m in names]
        chord=[r[chordkey] for r in normal if r['method']=='RAW' and r.get(chordkey) is not None]
        arrays.insert(1,chord);labels=['RAW adjacent','RAW chord',*names[1:]]
        ax.boxplot(arrays,labels=[m.replace('SPLINE_','').replace('_DIAGNOSTIC','')+f'\nn={len(a)}' for m,a in zip(labels,arrays)]);ax.set_ylabel(label);ax.tick_params(axis='x',labelsize=8)
    fig.suptitle('Boundary heading sensitivity | 92 nonbootstrap pairs; valid counts shown, no invented yaw')
    save(fig,'boundary_heading_estimators',['pair_metrics.json'])
    fig,axes=plt.subplots(2,3,figsize=(15,9))
    for row,scope in enumerate(['full','editable']):
        for ax,key,label in zip(axes[row],[scope+'_rmse_m',scope+'_mean_abs_lateral_m',scope+'_mean_abs_tangent_deg'],['XY overlap RMSE [m]','Mean |revision normal to fixed RAW chord| [m]','Mean derivative-heading difference [deg]']):
            boxes(ax,pair_rows,key);ax.set_ylabel(label);ax.set_title(scope+' nominal overlap')
    fig.suptitle('RAW vs independently fitted OLD/FRESH: shared times and fixed revision basis\nEditable points strictly after measured switch; no spatial registration')
    save(fig,'future_disagreement',['pair_metrics.json'])

    def case_plot(key):
        d=details[key];fig,axes=plt.subplots(1,2,figsize=(14,6));b=np.array(d['boundary']);fresh_pose=d['fresh_observation_pose']
        for ax in axes:
            ax.plot(*np.array(d['measured_committed_xy']).T,c='#278343',lw=3,label='Measured committed execution')
            for run,rid,old in [(d['run'],d['old_request_id'],True),(d['run'],d['fresh_request_id'],False)]:
                c=chunk(run,rid);p=np.array(c['native_world_xy'])
                ax.plot(*p.T,'.:',c='#999999' if old else '#333333',ms=3,label=('OLD' if old else 'FRESH')+' native prediction')
                if primary in c['fits']:ax.plot(*np.array(c['fits'][primary]['grid_world_xy']).T,c='#9864a8' if old else COLORS[primary],lw=2,label=('OLD' if old else 'FRESH')+' spline 3cm')
                ax.scatter(*c['observation_pose'][:2],facecolors='none',edgecolors='#9864a8' if old else 'black',s=50,label=('OLD' if old else 'FRESH')+' observation origin')
            ax.scatter(*b[:2],marker='*',c='#278343',s=130,zorder=8,label='Measured switch B')
            ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]')
        centers=[b[:2],fresh_pose[:2]]
        for m in names[:4]:
            if m not in d['methods']:continue
            v=d['methods'][m]['boundary'];p=np.array(v['boundary_world_xy']);der=np.array(v['boundary_derivative_world']);centers.append(p)
            axes[1].scatter(*p,c=COLORS[m],s=25,label=m+' at switch')
            if v['boundary_derivative_valid']:
                u=der/np.linalg.norm(der)*.16;axes[1].arrow(*p,*u,color=COLORS[m],width=.0015,length_includes_head=True,zorder=7)
        q=np.array(centers);lo=q.min(0)-.25;hi=q.max(0)+.25;axes[1].set(xlim=[lo[0],hi[0]],ylim=[lo[1],hi[1]],title='Switch geometry; arrows at predicted positions')
        axes[0].legend(fontsize=7);axes[1].legend(fontsize=6,loc='best')
        a=raw[key];s=next(r for r in pair_rows if r['pair_id']==key and r['method']==primary)
        fig.suptitle(f"{key} | full RMSE RAW {a['full_rmse_m']:.3f} -> spline03 {s.get('full_rmse_m',np.nan):.3f} m\nOriginal/fitted predictions are not executed paths; no Graph rerun",fontsize=12)
        save(fig,'case_'+key,[key])
    for run,i in cfg['selected_pairs']:case_plot(f'{run}_C{i:02d}_C{i+1:02d}')

    fig,axes=plt.subplots(2,2,figsize=(14,9))
    for row,key in enumerate(['A_C13_C14','B_C15_C16']):
        d=details[key];t=np.array(d['graph_reference_times'])-d['fresh_observation_sim_time'];yaw=d['fresh_observation_pose'][2]
        chord=np.array(d['graph_reference_poses'])[:,2]
        axes[row,0].plot(t,np.degrees(wrap(chord-yaw)),'o:',c='#444444',ms=3,label='Existing raw chord (invalid gaps)')
        for m in names[:4]:
            if m not in d['methods']:continue
            v=d['methods'][m];phi=np.array(v['reference_heading_rad']);speed=np.array(v['reference_derivative_speed'])
            axes[row,0].plot(t,np.degrees(wrap(phi-yaw)),c=COLORS[m],alpha=.8,label=m+' derivative')
            axes[row,1].plot(t,speed,c=COLORS[m],label=m)
        axes[row,0].set(title=key,ylabel='Derived heading vs observation axes [deg]');axes[row,1].set(title=key,ylabel='Derivative norm [m / nominal s]')
        axes[row,1].axhline(cfg['minimum_derivative_m_per_nominal_s'],c='red',ls='--',label='Reliability floor')
        for ax in axes[row]:ax.set_xlabel('Nominal seconds after FRESH observation');ax.legend(fontsize=7)
    fig.suptitle('Original adjacent derivative, existing graph chord, and spline derivative\nWhole remaining reference; heading recovery does not change prior Graph eligibility')
    save(fig,'tangent_reliability',['A_C13_C14','B_C15_C16'])
    collapsed=[]
    for key,d in details.items():
        g=d['original_graph_metrics'].get('GRAPH')
        if g and not g['first_segment_valid']:collapsed.append(key)
    fig,ax=plt.subplots(figsize=(12,5));x=np.arange(len(collapsed))
    for offset,m,label,color in [(-.25,'RAW','Original RAW','#333333'),(0,primary,'Independent spline03',COLORS[primary]),(.25,'GRAPH','Previously saved Graph','#d58500')]:
        vals=[]
        for key in collapsed:
            if m=='GRAPH':v=details[key]['original_graph_metrics']['GRAPH']['first_segment_length_m']
            else:v=next(r['first_sampled_segment_m'] for r in pair_rows if r['pair_id']==key and r['method']==m)
            vals.append(v*1000)
        ax.bar(x+offset,vals,.24,label=label,color=color)
    ax.set_yscale('log');ax.axhline(10,c='red',ls='--',label='Prior 10mm direction-evaluation floor');ax.set_xticks(x,collapsed,rotation=20);ax.set_ylabel('First sampled segment length [mm], logarithmic');ax.legend(fontsize=8)
    ax.set_title('All saved Graph short-first-segment cases, including B startup\nSame switch and next target times; spline is not reanchored to B')
    save(fig,'prior_graph_degeneracy',collapsed)
    valid=[r for r in pair_rows if r['method']==primary and not r['bootstrap'] and r.get('boundary_robot_heading_deg') is not None and raw[r['pair_id']].get('boundary_robot_heading_deg') is not None and next(s for s in sens if s['pair_id']==r['pair_id'])['boundary_heading_spread_deg'] is not None]
    chosen={}
    if valid:
        chosen['largest_heading_decrease']=min(valid,key=lambda r:r['boundary_robot_heading_deg']-raw[r['pair_id']]['boundary_robot_heading_deg'])['pair_id']
        chosen['largest_heading_increase']=max(valid,key=lambda r:r['boundary_robot_heading_deg']-raw[r['pair_id']]['boundary_robot_heading_deg'])['pair_id']
        cutoff=np.median([raw[r['pair_id']]['full_rmse_m'] for r in valid]);large=[r for r in valid if raw[r['pair_id']]['full_rmse_m']>=cutoff]
        chosen['persistent_future_example']=min(large,key=lambda r:abs(r['full_rmse_m']/raw[r['pair_id']]['full_rmse_m']-1))['pair_id']
    fig,axes=plt.subplots(2,3,figsize=(15,8))
    for col,(label,key) in enumerate(chosen.items()):
        rs=[next(r for r in pair_rows if r['pair_id']==key and r['method']==m) for m in names[:4]]
        for ax,k,unit in [(axes[0,col],'boundary_robot_heading_deg','Boundary heading seam [deg]'),(axes[1,col],'full_rmse_m','Full future XY RMSE [m]')]:
            ax.bar(['RAW','1cm','3cm','5cm'],[r.get(k,np.nan) for r in rs],color=[COLORS[m] for m in names[:4]]);ax.set_ylabel(unit)
        axes[0,col].set_title(label.replace('_',' ')+'\n'+key,fontsize=10)
    fig.suptitle('Supplemental descriptive sensitivity examples; four required cases remain unchanged')
    save(fig,'discontinuity_sensitivity_examples',chosen)
    # Per-time residuals expose endpoint concentration and prevent a single sigma interpretation.
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    for col,(run,rid) in enumerate([['A',26],['B',16]]):
        d=chunk(run,rid)
        for m in fitted:
            if m not in d['fits']:continue
            v=d['fits'][m]
            for row,k in enumerate(['residual_forward','residual_lateral']):axes[row,col].plot(v['nominal_times'],v[k],c=COLORS[m],label=m)
        axes[0,col].set_title(f'{run} C{rid}');axes[0,col].set_ylabel('Tangent residual [m]');axes[1,col].set_ylabel('Normal residual [m]')
        for ax in axes[:,col]:ax.set_xlabel('Nominal future time [s]');ax.legend(fontsize=7)
    fig.suptitle('Native minus fit at original waypoint times; synthetic fixtures excluded')
    save(fig,'residuals_over_nominal_time',[['A',26],['B',16]])
    pdfs=sorted((out/'figures').glob('*.pdf'))
    subprocess.run(['pdfunite',*[str(p) for p in pdfs],str(out/'comparison_figures.pdf')],check=True)
    return dict(figure_sources=manifest,supplemental_examples=chosen,graph_short_cases=collapsed,book_pages=len(pdfs))
