"""Saved-only review figures, explicitly separating conflict and safety margins."""
import argparse
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.two_hz_preflight import read,load_reference,PRIMARY,RADIUS,SEPARATION
from research.analyze_two_hz_preflight import base_axes
from research.records import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',required=True);args=parser.parse_args()
    out=Path(args.output_dir);dest=out/'stage-b/review-figures';dest.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    selected=read(out/'stage-b/selected-site.json');c=selected['candidate'];x,y=c['position_world_xyz'][:2]
    ref=load_reference(PRIMARY);g=read(out/'stage-a/corridor_geometry_validated.json');collisions=read(out/'stage-a/collision_geometry.json')
    fig,ax=plt.subplots(figsize=(12,6));base_axes(ax,g,collisions)
    ax.plot(*ref['xy'].T,color='#2563eb',lw=2,label='Measured original path (no human)')
    ax.add_patch(Circle((x,y),RADIUS+.25,facecolor='#ef4444',alpha=.13,label='Potential center-path conflict: r=0.856983 m'))
    ax.add_patch(Circle((x,y),SEPARATION,fill=False,ls=':',lw=2,color='#b91c1c',label='Required separation +0.05 m margin'))
    ax.add_patch(Circle((x,y),.25,facecolor='#f59e0b',edgecolor='black',label='Stationary human capsule radius'))
    vertices=np.array(selected['connected_bypass']['vertices_xy'])
    ax.plot(*vertices.T,'o--',color='#15803d',lw=2,label='Connected RIGHT candidate — never executed')
    conflict=read(out/'stage-b/physical'/f"{c['id']}.json")['relevance'][PRIMARY]
    a=ref['xy'][conflict['first_conflict']['segment']];b=ref['xy'][conflict['first_conflict']['segment']+1]
    p=a+conflict['first_conflict']['fraction']*(b-a)
    ax.scatter(*p,color='#b91c1c',marker='x',s=60,label='First hypothetical conflict on saved path')
    ax.annotate('West travel',xy=(x-1.5,12.),xytext=(x+.8,12.),arrowprops={'arrowstyle':'->'},ha='center')
    ax.text(x-2.7,8.35,'World metres, +Z up; LEFT = -Y, RIGHT = +Y\nDisk/collider AABBs are conservative proxies; no dynamic feasibility claim',fontsize=8)
    ax.set_xlim(x-3.3,x+3.3);ax.set_ylim(8.1,12.9);ax.legend(loc='upper left',fontsize=8)
    ax.set_title('MODEL-FREE HUMAN PLACEMENT PREFLIGHT — NO HUMAN-CONDITIONED NAVIGATION\nH05 selected; original scene geometry unchanged',fontsize=10)
    fig.tight_layout();fig.savefig(dest/'selected-site-conflict-and-bypass.png',dpi=180);plt.close(fig)
    write_json(dest/'plotted-coordinates.json',{'candidate':c['position_world_xyz'],'first_hypothetical_conflict_xy':p.tolist(),
        'measured_path_source':PRIMARY,'goal_cut_tick':ref['validation']['goal_entry_tick'],
        'bypass_vertices_xy':vertices.tolist(),'human_radius_m':.25,'baseline_exclusion_radius_m':RADIUS+.25,
        'required_separation_m':SEPARATION,'model_calls':0,'physics_steps':0})


if __name__=='__main__':main()
