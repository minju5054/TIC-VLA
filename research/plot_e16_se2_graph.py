"""Offline geometric figures; measured history and candidate futures stay distinct."""
import numpy as np

COLORS={'RAW':'#333333','RIGID':'#d58500','GRAPH':'#0674b3','GRAPH_NO_CORR':'#a6448b'}


def figures(out,results,cfg,phase):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    methods=list(COLORS);selected=[f'{k}_C{i:02d}_C{i+1:02d}' for k,i in cfg['selected_pairs']]
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.18,'pdf.fonttype':42})
    def save(fig,name):
        fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=170,bbox_inches='tight');fig.savefig(out/(name+'.pdf'),bbox_inches='tight');plt.close(fig)
    for key in selected:
        if key not in results:continue
        d=results[key];p=d['inputs'];fig,axes=plt.subplots(1,2,figsize=(14,6))
        if p['eligibility']!='ELIGIBLE':
            for ax in axes:ax.axis('off')
            axes[0].text(.05,.5,key+' INELIGIBLE\n'+p['exclusion_reason'],wrap=True)
        else:
            b=p['boundary'];h=p['executed_times']>=p['switch_sim_time']-1.
            for ax in axes:
                ax.plot(*p['executed_xy'][h].T,c='#298541',lw=3,label='Measured committed OLD execution')
                if p['old_remaining_xy'] is not None:ax.plot(*p['old_remaining_xy'].T,c='#888888',ls=':',lw=1.6,label='OLD remaining prediction')
                for m in methods:
                    x=d['outputs'].get(m)
                    if x is not None:ax.plot(*x[:,:2].T,c=COLORS[m],ls='--' if m=='GRAPH_NO_CORR' else '-',lw=1.7,label=m)
                ax.plot([b[0],p['reference'][0,0]],[b[1],p['reference'][0,1]],c='#777777',ls='--',lw=1)
                ax.scatter(*b[:2],c='#298541',marker='*',s=130,zorder=6,label='Fixed measured switch B')
                ax.scatter(*p['fresh_observation_pose'][:2],facecolors='none',edgecolors='black',marker='o',s=65,zorder=5,label='FRESH observation origin')
                ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]')
            points=np.vstack([p['executed_xy'][h],*[v[:,:2] for v in d['outputs'].values() if v is not None],p['fresh_observation_pose'][:2]])
            lo,hi=points.min(0)-.15,points.max(0)+.15;axes[0].set(xlim=[lo[0],hi[0]],ylim=[lo[1],hi[1]],title='Remaining nominal future; no measured future target')
            near=np.vstack([v[p['times']-p['times'][0]<=.5+1e-9,:2] for v in d['outputs'].values() if v is not None]);near=np.vstack([near,b[:2],p['fresh_observation_pose'][:2]])
            lo,hi=near.min(0)-.08,near.max(0)+.08;axes[1].set(xlim=[lo[0],hi[0]],ylim=[lo[1],hi[1]],title='Switch + first 0.5 nominal seconds')
            axes[0].legend(fontsize=7,loc='best')
            values=[]
            for m in methods[:3]:
                mm=d['metrics'][m];angle=mm['first_segment_robot_heading_deg'];values.append(f"{m}: join {'NA' if angle is None else f'{angle:.2f}'} deg, far {mm['far_fresh_xy_rmse_m']:.3f} m")
            fig.suptitle(f"{key} | {p['phase']} | derived geometric headings, not native yaw\n"+'; '.join(values),fontsize=11)
        save(fig,key)
    if phase!='all':return
    eligible=[d for d in results.values() if d['inputs']['eligibility']=='ELIGIBLE'];normal=[d for d in eligible if not d['inputs']['bootstrap']]
    total=len(results);excluded=total-len(eligible)
    title=f'{len(normal)} eligible nonbootstrap pairs; {excluded}/{total} excluded; two dependent runs, no navigation claim'
    def boxes(ax,key,ylabel,include=None):
        ms=include or methods;arrays=[];labels=[]
        for m in ms:
            a=[d['metrics'][m][key] for d in normal if d['metrics'].get(m) and d['metrics'][m][key] is not None];arrays.append(a);labels.append(m+f'\nn={len(a)}')
        artists=ax.boxplot(arrays,labels=labels,showfliers=True,patch_artist=True)
        for box,m in zip(artists['boxes'],ms):box.set_facecolor(COLORS[m]);box.set_alpha(.45)
        ax.set_ylabel(ylabel);ax.tick_params(axis='x',labelsize=8)
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for ax,key,label in zip(axes,['first_segment_robot_heading_deg','executed_to_future_tangent_deg','near_heading_total_variation_deg'],['First-segment vs measured yaw [deg]','Executed-to-future tangent [deg]','Near polyline total heading variation [deg]']):boxes(ax,key,label)
    fig.suptitle('Boundary continuity beyond the fixed-position constraint\n'+title,fontsize=11);save(fig,'boundary_continuity')
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for ax,key,label in zip(axes,['relative_translation_rmse_m','relative_rotation_rmse_deg','shape_turn_distortion_rmse_deg'],['Relative SE(2) translation error [m]','Relative SE(2) rotation error [deg]','Polyline turn distortion [deg]']):boxes(ax,key,label)
    fig.suptitle('Additional FRESH deformation is a cost; RAW/RIGID relative geometry unchanged\n'+title,fontsize=11);save(fig,'fresh_distortion')
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for ax,key,label in zip(axes,['far_fresh_xy_rmse_m','endpoint_displacement_m','derived_curve_heading_change_rmse_deg'],['Far XY RMSE vs FRESH [m], tau >=1.5s','Endpoint displacement from FRESH [m]','Geometric chord heading change [deg]']):boxes(ax,key,label)
    fig.suptitle('Preservation of later FRESH geometry; RAW is the reference, not ground truth\n'+title,fontsize=11);save(fig,'far_future')
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for m in methods:
        ds=[d for d in normal if d['metrics'].get(m) and d['metrics'][m]['executed_to_future_tangent_deg'] is not None]
        for ax,y in zip(axes,['far_fresh_xy_rmse_m','relative_translation_rmse_m']):ax.scatter([d['metrics'][m]['executed_to_future_tangent_deg'] for d in ds],[d['metrics'][m][y] for d in ds],s=22,c=COLORS[m],alpha=.65,label=m)
    axes[0].set_ylabel('Far XY RMSE from FRESH [m]');axes[1].set_ylabel('Relative SE(2) translation distortion [m]')
    for ax in axes:ax.set_xlabel('Executed-to-future tangent discontinuity [deg]');ax.legend(fontsize=8)
    fig.suptitle('Continuity versus intent and deformation: lower on both axes preferred\n'+title,fontsize=11);save(fig,'continuity_intent_tradeoff')
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for ax,key,label in zip(axes,['runtime_ms','nfev','final_objective'],['Solve wall time [ms]','Function evaluations (nfev)','Final normalized objective']):
        ms=['GRAPH','GRAPH_NO_CORR'];values=[[d['solvers'][m][key] for d in eligible if d['solvers'][m][key] is not None] for m in ms]
        ax.boxplot(values,labels=ms);ax.set_ylabel(label);ax.tick_params(axis='x',labelsize=8)
    counts='; '.join(f"{m}: {sum(d['solvers'][m]['success'] for d in eligible)}/{len(eligible)} converged" for m in ['GRAPH','GRAPH_NO_CORR'])
    fig.suptitle('All eligible pairs including startup | '+counts+'\nIterations unavailable; nfev/njev/callback calls exported',fontsize=11);save(fig,'solver_cost')
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for ax,key,label in zip(axes,['executed_to_future_tangent_deg','far_fresh_xy_rmse_m','old_correspondence_xy_rmse_m'],['Change in join discontinuity [deg]','Change in far FRESH XY RMSE [m]','Change in OLD XY disagreement [m]']):
        for k,mark in [('A','o'),('B','x')]:
            ds=[d for d in normal if d['run']==k and all(d['metrics'].get(m) and d['metrics'][m][key] is not None for m in ['GRAPH','GRAPH_NO_CORR'])]
            ax.scatter([d['inputs']['old_request_id'] for d in ds],[d['metrics']['GRAPH'][key]-d['metrics']['GRAPH_NO_CORR'][key] for d in ds],label=k,marker=mark,s=25)
        ax.axhline(0,c='gray',lw=.8);ax.set(xlabel='OLD request ID (within run)',ylabel=label);ax.legend()
    fig.suptitle('Correspondence ablation: enabled minus disabled, every other weight identical\n'+title,fontsize=11);save(fig,'correspondence_ablation')
    fig,axes=plt.subplots(2,2,figsize=(14,9),sharex=True)
    for row,k in enumerate(['A','B']):
        for m in methods:
            for col,key in enumerate(['first_segment_robot_heading_deg','far_fresh_xy_rmse_m']):
                ds=[d for d in eligible if d['run']==k and d['metrics'].get(m)]
                axes[row,col].plot([d['inputs']['old_request_id'] for d in ds],[d['metrics'][m][key] for d in ds],'.-',c=COLORS[m],label=m,ms=3,lw=.9)
        for ax in axes[row]:
            ax.axvspan(.5,1.5,color='#dddddd',alpha=.6);ax.set_title('Run '+k+'; gray=C1->C2 startup');ax.set_xlabel('OLD request ID')
        axes[row,0].set_ylabel('First-segment yaw gap [deg]');axes[row,1].set_ylabel('Far XY RMSE [m]')
    axes[0,0].legend(fontsize=8);fig.suptitle(f'All 94 source pairs accounted; {len(eligible)} eligible, {excluded} excluded\nGaps/undefined metrics retained in CSV; no outcomes discarded',fontsize=11);save(fig,'pair_distributions')
