"""Display-only C24 failure comparison in cm; no simulator or model."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from research.audit_e16_wheelchair_handoffs import read,write,rotation,digest


def render(out, subdir='figures'):
    if Path(subdir).name!=subdir or subdir in ('.','..'):raise ValueError('One new figure directory name required')
    dest=out/subdir;dest.mkdir(exist_ok=False)
    fig,axes=plt.subplots(2,2,figsize=(14,11))
    selected=[]
    for column,i in enumerate((3,4)):
        a=read(out/f'run-{i}.json');r=a['rows'][-1];w=a['wheelchair'];center=np.array(w['center_world_xyz'][:2])
        B=np.array(r['boundary']['pose'][:2]);old=np.array(r['old_remaining_world_xy_m']);fresh=np.array(r['fresh_remaining_world_xy_m'])
        expected=np.array(r['boundary']['expected_xy_m']);connector=np.array(r['raw_connector_world_xy_m'])
        with (Path(a['source'])/'robot_state.csv').open() as f:state=list(csv.DictReader(f))
        before=[row for row in state if r['observation_sim_s']-.5<=float(row['sim_time'])<=r['switch_sim_s']+1e-8]
        after=[row for row in state if r['switch_sim_s']-1e-8<=float(row['sim_time'])<=a['first_contact_sim_s']+1e-8]
        history=np.array([[float(s[k]) for k in ('x','y')] for s in before]);executed=np.array([[float(s[k]) for k in ('x','y')] for s in after])
        for row in range(2):
            ax=axes[row,column];origin=center if row==0 else B;cv=lambda p:100*(np.asarray(p)-origin)
            ax.plot(*cv(old).T,'--',color='#d18a20',lw=2,label='OLD remaining prediction')
            ax.plot(*cv(fresh).T,color='#227ab2',lw=2,label='FRESH remaining prediction')
            ax.plot(*cv(history).T,color='#444444',lw=2.5,label='Measured committed OLD')
            ax.plot(*cv(executed).T,color='#b52b42',lw=2.5,label='Measured after FRESH command')
            ax.plot(*cv(connector).T,':',color='#973ca6',lw=2,label='Virtual RAW connector')
            ax.scatter(*cv(B),marker='o',color='#292b31',s=65,zorder=8,label='Actual switch B')
            ax.scatter(*cv(expected),marker='+',color='#07915b',s=130,zorder=9,label='Nominal FRESH at switch')
            ax.set_aspect('equal');ax.grid(alpha=.2)
            if row==0:
                h=np.array(w['footprint_half_m']);corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*h
                corners=corners@rotation(w['yaw_rad']).T+center
                ax.add_patch(Polygon(cv(corners),color='#777777',alpha=.3,label='C24 bounding OBB (proxy)'))
                ax.add_patch(Circle(cv(B),100*a['robot_radius_m'],fill=False,color='#555555',ls=':',label='Conservative robot disk'))
                ax.scatter(*cv(executed[-1]),marker='X',color='red',s=90,label='Robot at first callback',zorder=10)
                ax.set(xlim=(-75,175),ylim=(-125,125),xlabel='World X relative to C24 center [cm]',ylabel='World Y relative to C24 center [cm]')
                ax.set_title(f'Run {i}: C{r["old_id"]} -> C{r["fresh_id"]}\nSwitch {r["switch_sim_s"]:.3f} s; first callback {a["first_contact_sim_s"]:.3f} s')
            else:
                ax.set(xlim=(-5,5),ylim=(-5,5),xlabel='World X relative to switch B [cm]',ylabel='World Y relative to switch B [cm]')
                ax.set_title(f'10 x 10 cm boundary view | position gap {100*r["boundary"]["position_error_m"]:.3f} cm\nExecuted direction gap {np.degrees(r["boundary"]["direction_error_rad"]):.3f} deg')
        selected.append(dict(run=i,old=r['old_id'],fresh=r['fresh_id'],rule='Last switch strictly before first nonfloor callback, not a selected safe candidate'))
    fig.suptitle('C24 failures: OLD and FRESH do not provide a qualified safe bypass\nObserved command application is distinct from following the entire native prediction',fontsize=15)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='lower center',ncol=3,fontsize=9,frameon=False)
    fig.tight_layout(rect=(0,.12,1,.94));p=dest/'c24-precontact-cm.png';fig.savefig(p,dpi=170);plt.close(fig)
    write(dest/'manifest.json',dict(selection=selected,units='cm',source_results={f'run-{i}.json':digest(out/f'run-{i}.json') for i in (3,4)},
          display='Translation and x100 only; observation anchors and world axes unchanged'))
    print(p)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--figure-subdir',default='figures');a=p.parse_args();render(a.out.resolve(),a.figure_subdir)
