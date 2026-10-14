"""Saved-only figures for the prespecified E16 delay suite."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from tools.e16_wheelchair_delay.common import read,write,rotation,to_world,snapshot,verify_snapshot
from tools.e16_wheelchair_delay.evidence import rows


def render(suite):
    out=suite/'figures';out.mkdir(exist_ok=False)
    root=Path(__file__).resolve().parents[2];protocol=read(suite/'protocol.json');cfg=read(suite/'config.json')
    wheel=read(suite/'runtime-reference.json')['wheelchairs_expected']
    records=[read(p) for p in sorted(suite.glob('trial-*/analysis/result.json'))]
    data=[]
    for r in records:
        raw=Path(r['source']);rr=rows(raw/'robot_state.csv');xy=np.array([[float(x[k]) for k in ('x','y')] for x in rr]);t=np.array([float(x['sim_time']) for x in rr])
        data.append((r,rr,xy,t))
    colors=plt.cm.tab10(np.arange(len(records)))
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,ax=plt.subplots(figsize=(14,8))
    for color,(r,rr,xy,t) in zip(colors,data):
        goal=r.get('goal_entry_sim_s');mask=t<=(goal if goal is not None else np.inf)
        ax.plot(*(100*xy[mask]).T,color=color,label=f'{r["index"]} {r["mode"]}',lw=2)
        if not mask.all():ax.plot(*(100*xy[~mask]).T,color=color,ls='--',alpha=.4,lw=1)
        if r.get('detail'):
            d=r['detail'];ax.scatter(*(100*np.array(d['hold_start']['pose'][:2])),color=color,marker='o',s=45)
            ax.scatter(*(100*np.array(d['hold_end']['pose'][:2])),color=color,marker='x',s=55)
        contact=r.get('first_nonfloor_contact')
        if contact:
            p=contact['contacts'][0]['position'];ax.scatter(*(100*np.array(p[:2])),marker='X',color='red',s=65)
    for name,w in wheel.items():
        h=np.array(w['footprint_half_m']);p=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*h
        p=p@rotation(w['yaw_rad']).T+w['center_world_xyz'][:2]
        ax.add_patch(Polygon(100*p,color='#666666',alpha=.45));ax.text(*(100*np.array(w['center_world_xyz'][:2])),name,ha='center',va='center')
    trigger=100*np.array(protocol['trigger']['world_corners_xy_m']);ax.add_patch(Polygon(trigger,fill=False,color='#b632ac',lw=2,ls=':'))
    goal=100*np.array(cfg['scene']['goal'][:2]);ax.add_patch(Circle(goal,200,color='#d4b843',alpha=.15));ax.scatter(*goal,marker='*',color='#ac8c00',s=130)
    ax.set_aspect('equal');ax.set(xlabel='Hospital world X [cm]',ylabel='Hospital world Y [cm]')
    ax.set_title('Figure 1 | Independent actual E16 runs; fixed spatial trigger\nCircles B0; crosses B_delta; red X first nonfloor callback; dashed paths after goal')
    ax.grid(alpha=.2);ax.legend(ncol=2);fig.tight_layout();fig.savefig(out/'figure1-world.png',dpi=160);plt.close(fig)
    n=len(data);fig,axes=plt.subplots(2,3,figsize=(16,10));axes=axes.ravel()
    for ax,color,(r,rr,xy,t) in zip(axes,colors,data):
        d=r.get('detail')
        if not d:ax.text(.5,.5,'No eligible trigger / incomplete evidence',ha='center',transform=ax.transAxes);ax.set_title(f'{r["index"]} {r["mode"]}');continue
        B0=np.array(d['hold_start']['pose'][:2]);convert=lambda p:100*(np.asarray(p)-B0)
        source=Path(r['source']);old=read(source/'raw/requests'/f'request_{d["old_id"]:06d}.json')
        old_world=to_world(np.load(source/'raw/requests'/f'request_{d["old_id"]:06d}.npy'),old['observation']['pose'])
        a=old['application']['tick'];b=d['hold_end']['tick'];hs=d['hold_start']['tick']
        ax.plot(*convert(old_world).T,'--',color='#cf922c',label='OLD prediction')
        ax.plot(*convert(d['world_xy_m']).T,color='#347eb9',label='Same FRESH')
        ax.plot(*convert(xy[a:b+1]).T,color='#343c43',lw=2.5,label='Committed OLD')
        ax.plot(*convert(xy[hs:b+1]).T,color='#d54470',lw=3,label='Extra hold (measured)')
        ax.scatter(0,0,marker='o',color='#e78b2a',s=65,label='B0');ax.scatter(*convert(d['hold_end']['pose'][:2]),marker='X',color='#d62838',s=60,label='B_delta')
        for when,ls in (('before','--'),('after',':')):
            v=r['paired'][when]
            if v['expected_xy_m'] is not None:ax.scatter(*convert(v['expected_xy_m']),marker='+',s=100,color='#15936d')
            con=np.asarray(v['connector_xy_m'])
            if len(con):ax.plot(*convert(con).T,ls,color='#a63ba5',lw=1.8,label='Virtual connector' if when=='after' else None)
        ep=r['paired']['after']['position_error_m'];ax.set_title(f'{r["index"]} {r["mode"]} | physical request {d["request_id"]}\n'+('e_p undefined' if ep is None else f'e_p = {100*ep:.3f} cm'))
        ax.set(xlim=(-50,50),ylim=(-50,50),xlabel='World X relative to B0 [cm]',ylabel='World Y relative to B0 [cm]');ax.set_aspect('equal');ax.grid(alpha=.2)
    for ax in axes[n:]:ax.axis('off');ax.text(.5,.5,'Positive-delay run not executed\nBaseline gate or safety stop',transform=ax.transAxes,ha='center',va='center')
    if data and data[0][0].get('detail'):axes[0].legend(fontsize=8,loc='lower left')
    fig.suptitle('Figure 2 | Matched 100 x 100 cm handoff views; independent trials\nWorld axes unchanged; B0 display origin only. Green +: nominal FRESH position; not native yaw/time.',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'figure2-handoffs.png',dpi=160);plt.close(fig)
    stage=read(suite/'stage-a/result.json')['results'];x=[r['extra_delay_s'] for r in stage]
    fig,axes=plt.subplots(2,3,figsize=(15,9));axes=axes.ravel()
    axes[0].plot(x,[100*r['kinematic_old_path_m'] for r in stage],'o-');axes[0].set_ylabel('Additional OLD path [cm]')
    axes[1].plot(x,[100*r['paired']['after']['position_error_m'] for r in stage],'o-',label='Time-aligned')
    axes[1].plot(x,[100*r['paired']['after']['fixed_reference_position_error_m'] for r in stage],'s--',label='One fixed reference time');axes[1].set_ylabel('Position error [cm]');axes[1].legend()
    value=lambda r,k:np.nan if r['paired']['after'][k] is None else r['paired']['after'][k]
    axes[2].plot(x,[np.degrees(value(r,'direction_error_rad')) for r in stage],'o-');axes[2].set_ylabel('Direction error [deg]')
    axes[3].plot(x,[100*value(r,'longitudinal_error_m') for r in stage],'o-',label='Longitudinal')
    axes[3].plot(x,[100*value(r,'lateral_error_m') for r in stage],'s-',label='Lateral');axes[3].set_ylabel('Signed time-aligned error [cm]');axes[3].legend()
    axes[4].plot(x,[r['paired']['after']['remaining_horizon_s'] for r in stage],'o-');axes[4].set_ylabel('Remaining nominal horizon [s]')
    actual=[r for r in records if r['delay_s']>0 and r.get('paired')]
    axes[5].axis('off')
    note='Actual 0.25 s trials\n'
    if not actual:note+='Not executed: baseline/safety gate.\nCounterfactual curves are NOT PhysX evidence.'
    for r in actual:
        p=r['paired'];fmt=lambda v:'undefined' if v is None else f'{100*v:.3f} cm'
        note+=f'Run {r["index"]}: e_p {fmt(p["before"]["position_error_m"])} -> {fmt(p["after"]["position_error_m"])}\n'
        note+=f'Hold path {100*r["transport"]["hold"]["path_m"]:.3f} cm\n'
    axes[5].text(.03,.8,note,transform=axes[5].transAxes,va='top',fontsize=12,linespacing=1.8)
    for ax in axes[:5]:ax.set_xlabel('Extra kinematic delay [s]');ax.grid(alpha=.2);ax.set_xticks(x)
    fig.suptitle('Figure 3 | Stage A: same saved C13 FRESH, constant OLD unicycle approximation\nDifferent nominal boundary times and fixed-reference spatial changes shown separately',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'figure3-delay-sensitivity.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(14,10),sharex=True,gridspec_kw={'height_ratios':[1.1,1,1]})
    for color,(r,rr,xy,t) in zip(colors,data):
        idx=r['index'];axes[0].plot([t[0],t[-1]],[idx,idx],color=color,alpha=.35)
        for name,marker in (('C10','^'),('C24','s')):
            pt=r.get('navigation',{}).get(name,{}).get('passage_sim_s')
            if pt is not None:axes[0].scatter(pt,idx,marker=marker,color=color,s=65)
        goal=r.get('goal_entry_sim_s')
        if goal is not None:axes[0].scatter(goal,idx,marker='*',color=color,s=150)
        ct=r.get('first_nonfloor_contact')
        if ct:axes[0].scatter(ct['sim_time'],idx,marker='x',color='red',s=65)
        axes[1].plot(t,[100*float(row['body_v_forward']) for row in rr],color=color,label=f'{idx} {r["mode"]}',lw=1)
        axes[2].plot(t,[np.degrees(float(row['body_yaw_rate_measured'])) for row in rr],color=color,lw=1)
    axes[0].set_yticks([r['index'] for r in records],[f'{r["index"]} {r["mode"]}' for r in records]);axes[0].invert_yaxis()
    axes[0].set_title('Figure 4 | Actual navigation outcomes and measured motion\nTriangle C10 full passage; square C24; star goal entry; red x first nonfloor callback')
    axes[1].set_ylabel('Body forward velocity [cm/s]');axes[2].set_ylabel('Body yaw rate [deg/s]');axes[2].set_xlabel('Simulation time [s]')
    for ax in axes:ax.grid(alpha=.2)
    axes[1].legend(ncol=3,fontsize=9);fig.tight_layout();fig.savefig(out/'figure4-navigation.png',dpi=160);plt.close(fig)
    write(out/'figure-manifest.json',dict(source_trial_indices=[r['index'] for r in records],stage_a='KINEMATIC_APPROXIMATION_ONLY',
         position_units='cm',same_trial_B0_Bdelta_separate_from_independent_runs=True))
    print(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);a=p.parse_args();render(a.suite.resolve())
