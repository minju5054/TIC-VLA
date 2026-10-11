"""Static scientific figures from measured/saved evidence only."""
import numpy as np


def figures(out,runs,results,pairs,chosen,inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from PIL import Image
    from research.hospital_spatial_gate import wall_inventory
    primary=[k for k,d in results.items() if d['summary']['primary']]
    colors={k:plt.cm.tab10(i) for i,k in enumerate(primary)}
    colors.update(Prev_zero_1='#333333',Prev_zero_2='#888888',Ref_A='#aaaaaa',Ref_B='#bbbbbb')
    g=runs[primary[0]]['cfg']['spatial_turn_gate'];goal=runs[primary[0]]['cfg']['scene']['goal'][:2]
    def context(ax,bounds):
        for w in wall_inventory(inventory):
            lo=np.array(w['world_aabb_min'][:2]);hi=np.array(w['world_aabb_max'][:2]);ax.add_patch(Rectangle(lo,*(hi-lo),fc='#ededed',ec='#cccccc',lw=.4,zorder=0))
        ax.plot([g['exit_center_x_max_m']]*2,[g['center_y_min_m'],g['center_y_max_m']],color='#444444',ls='--',label='Original exit gate')
        for y in [g['center_y_min_m'],g['center_y_max_m']]:ax.axhline(y,color='gray',ls=':',lw=.7)
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.15)
    def save(fig,name):
        fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=160,bbox_inches='tight');plt.close(fig)
    for zoom in [False,True]:
        fig,ax=plt.subplots(figsize=(14,8));allxy=np.vstack([runs[k]['xy'] for k in primary]);lo,hi=allxy.min(0)-1,allxy.max(0)+1
        context(ax,[-1.5,8.5,7.5,13.5] if zoom else [lo[0],hi[0],lo[1],hi[1]])
        for k in primary:
            r=runs[k];d=results[k];m=d['summary'];label=k+(' PASS' if m['corner_success'] else ' FAIL')
            ax.plot(*r['xy'].T,c=colors[k],ls='-' if m['condition']=='NO_HOOK' else '--',lw=1.7,label=label)
            if m['selected_request_id']:
                e=r['events'][m['selected_request_id']-1];ax.scatter(*e['agent_pose_at_observation'][:2],s=35,c=[colors[k]],marker='o')
            if m['first_contact']:ax.scatter(*np.asarray(m['first_contact']['reported_points'])[:,:2].T,c=[colors[k]],marker='+',s=100,zorder=5)
        if not zoom:ax.scatter(*goal,marker='*',s=100,c='black',label='Goal')
        ax.legend(fontsize=8);ax.set_title('Six new runs | solid NO_HOOK; dashed ZERO_HOOK\nMeasured execution; o spatially selected observation; + PhysX contact')
        save(fig,'corner_comparison' if zoom else 'six_run_world_xy')
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for k in primary:
        m=results[k]['summary'];xgate=runs[k]['cfg']['spatial_turn_gate']['exit_center_x_max_m']
        rows=[r for r in results[k]['rows'] if r['precontact'] and r['observation_x_m']>xgate and r['observation_north_velocity_m_s']>0 and 2<=r['observation_y_m']<13]
        for ax,key in zip(axes,['controller_w','native_lookahead_left_m']):ax.plot([r['observation_y_m'] for r in rows],[r[key] for r in rows],'.-',c=colors[k],label=k)
    axes[0].set(ylabel='Applied command w [rad/s]');axes[1].set(xlabel='Measured observation world Y [m], east approach only',ylabel='Native lookahead left [m]')
    for ax in axes:ax.axhline(0,c='black',lw=.7);ax.grid(alpha=.2)
    axes[0].legend(fontsize=8,ncol=3);fig.suptitle('Physical progress comparison, precontact only; differing XY/yaw remain explicit in CSV')
    save(fig,'angular_command_vs_progress')
    fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
    for k in primary:
        r=runs[k];rows=r['robot'];t=r['times'];yaw=np.unwrap([float(v['yaw']) for v in rows]);contact=results[k]['summary']['first_contact'];mask=t<=(contact['sim_time'] if contact else np.inf)
        axes[0].plot(t[mask],np.degrees(yaw[mask]),c=colors[k],label=k)
        axes[1].plot(t[mask],np.array([float(v['body_yaw_rate_measured']) for v in rows])[mask],c=colors[k])
        axes[2].plot(t[mask],np.array([float(v['body_v_forward']) for v in rows])[mask],c=colors[k])
    for ax,label in zip(axes,['Measured unwrapped yaw [deg]','Measured yaw rate [rad/s]','Measured forward speed [m/s]']):ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[-1].set_xlabel('Simulation time [s]');axes[0].legend(fontsize=8,ncol=3);fig.suptitle('Measured robot response; curves end at first contact or run end')
    save(fig,'measured_yaw_and_velocity')
    fig,axes=plt.subplots(1,2,figsize=(14,6));low=chosen['first_two_bin_steering_divergence_low_y_m'] if chosen else 9
    names=primary+(['Prev_zero_1'] if not any(not results[k]['summary']['corner_success'] for k in primary) else [])
    for ax,binlow in zip(axes,[low,low+1]):
        for k in names:
            b=next(b for b in results[k]['bins'] if b['low_y_m']==binlow);rid=b['representative_request_id']
            if rid is None:continue
            c=runs[k]['chunks'][rid-1];e=runs[k]['events'][rid-1];label=f"{k} C{rid}, Y={e['agent_pose_at_observation'][1]:.2f}"
            ax.plot(*c.T,'.-',c=colors[k],lw=1,label=label,ms=3)
            ax.scatter(*c[e['lookahead_index']],c=[colors[k]],marker='X',s=55)
        ax.axhline(0,c='gray',lw=.7);ax.set(xlabel='Native forward [m]',ylabel='Native left [m]',title=f'Observation world Y in [{binlow},{binlow+1}) m');ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=6.5)
    fig.suptitle('Saved native FRESH curves in EACH observation frame; X=original lookahead\nDifferent poses/views, not a frame-invariant cross-run subtraction; prior failure labeled Prev_zero_1 if needed',fontsize=10)
    save(fig,'fresh_native_progress_comparison')
    fig,axes=plt.subplots(1,3,figsize=(15,6))
    for ax,key,unit in zip(axes,['action_wall_s','observation_to_switch_wall_s','detection_to_application_wall_ms'],['Action-call wall [s]','Observation-to-application wall [s]','Detection-to-application wall [ms]']):
        arrays=[[r[key] for r in results[k]['rows'][1:]] for k in primary]
        ax.boxplot(arrays,labels=[k.replace('_','\n',1) for k in primary],showfliers=True)
        for i,k in enumerate(primary,1):
            rid=results[k]['summary']['selected_request_id']
            if rid:ax.scatter(i,results[k]['rows'][rid-1][key],c=[colors[k]],marker='D',s=40,zorder=5)
        ax.set_ylabel(unit);ax.tick_params(axis='x',labelsize=7);ax.grid(axis='y',alpha=.2)
    fig.suptitle('47 nonbootstrap handoffs per run; diamond=spatial selection\nRight panel includes validation/controller/application and is NOT exact hook cost',fontsize=11)
    save(fig,'timing_distributions')
    fig,ax=plt.subplots(figsize=(14,4));ax.axis('off');data=[]
    def f(v):return '—' if v is None else f'{v:.3f}'
    for k in primary:
        m=results[k]['summary'];data.append([k,'PASS' if m['corner_success'] else 'FAIL',f(m['northward_overshoot_m']),f(m['passage_sim_time']),f(m['first_contact']['sim_time'] if m['first_contact'] else None),f(m['goal_entry_sim_time']),str(m['selected_request_id']),f(m['selected_detection_to_application_wall_ms'])])
    table=ax.table(cellText=data,colLabels=['Run','Corner','Overshoot m','Passage s','Contact s','Goal entry s','Selected C','Detect→apply ms'],loc='center',cellLoc='center');table.auto_set_font_size(False);table.set_fontsize(9);table.scale(1,1.8)
    ax.set_title('New trials only; original navigation gates; contact evidence separate from overshoot\nEvery run: 48 predictions, zero extra physics; same runtime config',fontsize=11);save(fig,'navigation_outcomes')
    # A missing prespecified directional onset remains undefined, not replaced
    # with the first floating-point/image difference.
    fig=plt.figure(figsize=(14,9));grid=fig.add_gridspec(2,2)
    if chosen:
        first=chosen['first_two_bin_steering_divergence_low_y_m'];names=[chosen['a'],chosen['b']]
        for column,k in enumerate(names):
            b=next(b for b in results[k]['bins'] if b['low_y_m']==first);rid=b['representative_request_id'];row=results[k]['rows'][rid-1]
            ax=fig.add_subplot(grid[0,column]);ax.imshow(Image.open(row['rgb']));ax.axis('off');ax.set_title(f"{k} C{rid} | XY=({row['observation_x_m']:.3f},{row['observation_y_m']:.3f})\nyaw={np.degrees(row['observation_yaw_rad']):.2f}°, w={row['controller_w']:.4f}",fontsize=10)
        ax=fig.add_subplot(grid[1,0]);context(ax,[-1,8,6.5,13.5])
        for k in names:
            b=next(b for b in results[k]['bins'] if b['low_y_m']==first);rid=b['representative_request_id'];r=runs[k];e=r['events'][rid-1]
            mask=(r['times']>=e['observation']['sim_time']-.5)&(r['times']<=e['observation']['sim_time']+1)
            ax.plot(*r['xy'][mask].T,c=colors[k],label=k+' measured');ax.plot(*r['worlds'][rid-1].T,c=colors[k],ls=':',label=f'{k} C{rid} prediction')
            ax.scatter(*e['agent_pose_at_observation'][:2],c=[colors[k]],marker='o')
        ax.legend(fontsize=6)
        ax=fig.add_subplot(grid[1,1])
        for k in names:
            bs=[b for b in results[k]['bins'] if b['count']];ax.plot([b['low_y_m']+.5 for b in bs],[b['median_w'] for b in bs],'.-',c=colors[k],label=k)
        ax.axvspan(first,first+2,color='gray',alpha=.15);ax.axhline(0,c='gray',lw=.7);ax.set(xlabel='Observed Y bin midpoint [m]',ylabel='Median applied w [rad/s]');ax.legend(fontsize=8)
        fig.suptitle(f'First frozen two-bin steering disagreement: Y [{first},{first+2}) m\nDifferent observed states/images; descriptive divergence, not causal onset',fontsize=12)
    else:
        # Show the first two runs by protocol order, without replacing the
        # undefined meaningful-onset result with a numerical difference.
        names=primary[:2]
        pair=next(p for p in pairs if p['a']==names[0] and p['b']==names[1])
        for column,k in enumerate(names):
            row=results[k]['rows'][0];ax=fig.add_subplot(grid[0,column])
            ax.imshow(Image.open(row['rgb']));ax.axis('off')
            ax.set_title(f"{k} C1 | original saved RGB\nw={row['controller_w']:.5f} rad/s",fontsize=11)
        ax=fig.add_subplot(grid[1,0])
        for k in names:
            c=runs[k]['chunks'][0];ax.plot(*c.T,'.-',c=colors[k],label=k+' C1')
        ax.set(xlabel='Native forward [m]',ylabel='Native left [m]',title='Bootstrap numerical difference; not a failure onset')
        ax.set_aspect('equal');ax.legend(fontsize=9);ax.grid(alpha=.2)
        ax=fig.add_subplot(grid[1,1]);ax.axis('off')
        rmse=pair['first_native_rmse_same_pose_m'];rmse_label='undefined' if rmse is None else f'{rmse:.5f} m'
        ax.text(.03,.95,'Primary meaningful divergence: UNDEFINED\nNo success/failure pair meets the frozen rule.\n\n'
                f"First two runs, selected by protocol order:\nC1 saved SE(2) identical: {pair['first_rgb']['exact_pose_equal']}\n"
                f"RGB MAE: {pair['first_rgb']['mae_0_255']:.3f} / 255\nSame-pose native RMSE: {rmse_label}\n\n"
                'Initial image/prediction differences do not establish\nwhy a later navigation failure would occur.\nNo causal onset or KV tensor equality is inferred.',
                va='top',fontsize=11,linespacing=1.55)
        fig.suptitle('Earliest-divergence diagnostic | numeric differences already at C1\nFrozen meaningful steering-onset rule remains undefined',fontsize=12)
    save(fig,'earliest_divergence_diagnostic')
    fig,axes=plt.subplots(2,3,figsize=(15,7))
    for ax,k in zip(axes.flat,primary):
        row=results[k]['rows'][0];ax.imshow(Image.open(row['rgb']));ax.axis('off');ax.set_title(f"{k} C1\nRGB SHA {row['rgb_sha256'][:10]}; w={row['controller_w']:.5f}",fontsize=9)
    fig.suptitle('First observed RGB, original saved PNGs | numerical differences are not causal onset');save(fig,'bootstrap_rgb')
    fig,axes=plt.subplots(6,1,figsize=(13,11),sharex=True)
    for ax,k in zip(axes,primary):
        rows=results[k]['rows'];ids=np.array([r['request_id'] for r in rows]);starts=[r['request_id'] for r in rows if r['reasoning_new_start_step'] is not None]
        changes=[r['request_id'] for i,r in enumerate(rows) if i==0 or r['reasoning_completion_poll_step']!=rows[i-1]['reasoning_completion_poll_step']]
        texts=[r['request_id'] for i,r in enumerate(rows) if i==0 or r['reasoning_text_sha256']!=rows[i-1]['reasoning_text_sha256']]
        ax.scatter(starts,[2]*len(starts),marker='|',s=110,c='blue');ax.scatter(changes,[1]*len(changes),marker='s',s=20,c='orange');ax.scatter(texts,[0]*len(texts),marker='.',c='green')
        ax.set(yticks=[0,1,2],yticklabels=['text change','poll change','new start'],ylim=[-.5,2.5],title=k);ax.tick_params(axis='y',labelsize=7);ax.grid(axis='x',alpha=.2)
    axes[-1].set_xlabel('Within-run request ID (descriptive lifecycle index, not cross-run physical alignment)')
    fig.suptitle('Saved reasoning lifecycle | poll detection is not exact worker completion; KV tensor identity unavailable',fontsize=11)
    save(fig,'reasoning_lifecycle')
