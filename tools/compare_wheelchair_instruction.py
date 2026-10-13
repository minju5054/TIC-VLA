"""Descriptive saved-only comparison of two wheelchair instructions."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def read(path):
    return json.loads(path.read_text())


def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def compare(previous,current,output):
    from tools.wheelchair_instruction.policy import scene_config, INSTRUCTION
    from PIL import Image
    old_cfg=read(previous/'metadata.json')['config'];new_cfg=read(current/'metadata.json')['config']
    assert scene_config(new_cfg,old_cfg)==old_cfg
    snapshots={}
    for run in (previous,current):
        snapshots[run]=read(run/'raw-sha256.json')
        if any(digest(run/p)!=h for p,h in snapshots[run].items()):raise ValueError('Raw hash mismatch')
    events={run:[read(p) for p in sorted((run/'raw/requests').glob('*.json'))] for run in snapshots}
    assert len(events[previous])==len(events[current])==48
    assert all(e['instruction']==INSTRUCTION for e in events[current])
    assert all(e['instruction']==old_cfg['instruction'] for e in events[previous])
    old=read(previous/'derived/c24-wheelchair/summary.json');new=read(current/'derived/c24-wheelchair/summary.json')
    traces={}
    for run in snapshots:
        with (run/'robot_state.csv').open() as stream:
            traces[run]=np.array([[float(r[k]) for k in ('sim_time','x','y')] for r in csv.DictReader(stream)])
    first=[]
    for run in snapshots:
        with Image.open(run/events[run][0]['rgb_observation_reference']) as im:first.append(np.array(im.convert('RGB'),dtype=float))
    difference=np.abs(first[0]-first[1])
    result={'comparison':'One previous original-instruction run versus one new avoidance-instruction run',
        'changed_configuration_keys':['instruction'],'previous_run':previous.name,'current_run':current.name,
        'previous_instruction':old_cfg['instruction'],'current_instruction':INSTRUCTION,
        'actual_request_instruction_verified':{'previous':48,'current':48},
        'initial_pose_equal':events[previous][0]['observation']['position']==events[current][0]['observation']['position'],
        'initial_rgb_mean_absolute_difference_0_255':float(difference.mean()),
        'initial_rgb_max_absolute_difference_0_255':float(difference.max()),
        'previous':old,'current':new,
        'limitation':'Descriptive single-run comparison. Identical non-instruction configuration does not isolate stochastic model/renderer behavior or asynchronous scheduling; no causal improvement claim.'}
    output.mkdir(parents=True,exist_ok=False)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Polygon
    from research.cart_geometry import rotation
    fig,axes=plt.subplots(1,2,figsize=(15,6),gridspec_kw={'width_ratios':[1.6,1]})
    goal=np.array(new_cfg['scene']['goal'][:2])
    for run,label,color in [(previous,'Original instruction','#8996a4'),(current,'Added obstacle-avoidance instruction','#147b91')]:
        trace=traces[run];axes[0].plot(trace[:,1],trace[:,2],color=color,lw=2,label=label)
        axes[0].scatter(*trace[-1,1:],marker='X',s=65,color=color)
        axes[1].plot(trace[:,0],np.linalg.norm(trace[:,1:]-goal,axis=1),color=color,lw=2,label=label)
    h=new_cfg['fixed_humans'][0];axes[0].add_patch(Circle(h['position'][:2],h['radius_m'],color='#dc8629'))
    wa=read(current/'wheelchair_asset.json');lo,hi=np.array(wa['local_bounds']);matrix=np.array(wa['matrix_row_vector'])
    corners=np.array([[lo[0],lo[1],0,1],[hi[0],lo[1],0,1],[hi[0],hi[1],0,1],[lo[0],hi[1],0,1]])@matrix
    axes[0].add_patch(Polygon(corners[:,:2],color='#75409a',alpha=.65))
    for name,p in [('C10 human',h['position']),('C24 wheelchair',wa['center_world_xyz'])]:axes[0].annotate(name,p[:2],xytext=(7,-18),textcoords='offset points',fontsize=9)
    axes[0].add_patch(Circle(goal,2,color='#e2c469',alpha=.2));axes[0].scatter(*goal,marker='*',s=120,color='#a47a00',label='Goal / 2m')
    axes[0].set(xlabel='Hospital world X [m]',ylabel='Hospital world Y [m]',title='Measured robot paths; crosses = final positions');axes[0].axis('equal');axes[0].legend(fontsize=8)
    axes[1].axhline(2,color='#a47a00',ls='--');axes[1].set(xlabel='Simulation time [s]',ylabel='Goal distance [m]',title='Distance to unchanged goal')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Same C10 human + C24 wheelchair | instruction comparison',fontsize=16)
    fig.text(.5,.01,'One run per instruction; descriptive, not causal. Hospital origin, world XY metres,+Z up. Walls omitted.',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.045,1,.94]);fig.savefig(output/'instruction-comparison.png',dpi=170);plt.close(fig)
    if any(digest(run/p)!=h for run,hashes in snapshots.items() for p,h in hashes.items()):raise ValueError('Raw changed')
    print(json.dumps({k:v for k,v in result.items() if k not in ('previous','current')},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--previous',type=Path,required=True);p.add_argument('--current',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();compare(a.previous.resolve(),a.current.resolve(),a.output.resolve())
