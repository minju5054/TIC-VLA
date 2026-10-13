"""Plot saved measured motion and replay RGB observations; never start Isaac."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(path.read_text())


def make_view(source, output):
    source, output = source.resolve(), output.resolve()
    if output == source or output.is_relative_to(source):
        raise ValueError('Keep this review separate from the original run')
    if output.exists():
        raise FileExistsError('A fresh review directory is required')
    protected = {str(p.relative_to(source)): digest(p) for p in source.rglob('*') if p.is_file()}
    for relative, expected in read(source/'raw-sha256.json').items():
        if protected.get(relative) != expected:
            raise ValueError('Raw evidence changed: '+relative)
    cfg = read(source/'runtime-config.json')
    events = [read(p) for p in sorted((source/'raw/requests').glob('*.json'))]
    summary = read(source/'derived/api-smoke/summary.json')
    if not summary['runtime_pass'] or len(events) != summary['predictions']:
        raise ValueError('Completed runtime result and all observations required')
    with (source/'robot_state.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    dense = np.array([[float(r[k]) for k in ('sim_time','x','y')] for r in rows])
    if not np.isfinite(dense).all() or np.any(np.diff(dense[:,0]) <= 0):
        raise ValueError('Invalid measured path/timestamps')
    ticks = {int(r['tick']): r for r in rows}
    for e in events:
        o = e['observation']; r = ticks[o['tick']]
        if o['sim_time'] != float(r['sim_time']) or o['position'][:2] != [float(r['x']),float(r['y'])]:
            raise ValueError('Observation does not match the saved physics tick')
    t, xy = dense[:,0], dense[:,1:]
    goal = np.array(cfg['scene']['goal'][:2])
    with (source/'raw/human_state.csv').open() as stream:
        human_rows = list(csv.DictReader(stream))
    human_xy = np.array([[float(r[k]) for k in ('x','y')] for r in human_rows])
    if len(human_xy) != len(xy) or not np.isfinite(human_xy).all() or np.max(np.linalg.norm(human_xy-human_xy[0],axis=1)) > 1e-6:
        raise ValueError('Matched stationary measured human trace required')
    human = human_xy[0]
    goal_distance = np.linalg.norm(xy-goal,axis=1)
    entry = np.flatnonzero(goal_distance < 2.)
    first = int(entry[0]) if len(entry) else None
    obs_t = np.array([e['observation']['sim_time'] for e in events])
    obs_xy = np.array([e['observation']['position'][:2] for e in events])
    if not np.all(np.diff(obs_t)>0):
        raise ValueError('Observation times are not increasing')
    output.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig, ax = plt.subplots(figsize=(13,6.6))
    if first is None:
        ax.plot(*xy.T,color='#126c9c',lw=2.3,label='Measured path')
    else:
        ax.plot(*xy[:first+1].T,color='#126c9c',lw=2.3,label='Measured path before first goal entry')
        ax.plot(*xy[first:].T,color='#8994a3',lw=2.3,ls='--',label='Continued motion after goal entry')
        ax.scatter(*xy[first],marker='D',s=60,color='#176b45',zorder=5,label=f'First <2 m entry: {t[first]:.2f} s')
    ax.scatter(*xy[0],marker='s',s=90,color='#176b45',label='Start',zorder=5)
    ax.scatter(*xy[-1],marker='X',s=100,color='#b83a38',label='Final position',zorder=5)
    ax.scatter(*goal,marker='*',s=170,color='#b38a00',label='Goal',zorder=5)
    ax.add_patch(Circle(goal,2.,facecolor='#f9e9ab',edgecolor='#b38a00',alpha=.35,label='Goal radius: 2 m'))
    ax.add_patch(Circle(human,cfg['stationary_human']['radius_m'],facecolor='#ed8b31',alpha=.7))
    ax.scatter(*human,marker='o',s=35,color='#bd550b',label='Measured stationary human',zorder=5)
    indices=sorted(set([0,11,len(events)-1,*range(7,len(events),8)]))
    ax.scatter(*obs_xy[indices].T,s=18,color='#126c9c',zorder=4)
    for i in indices:
        ax.annotate(f'C{i+1}',obs_xy[i],xytext=(0,9 if i%2 else -17),textcoords='offset points',ha='center',fontsize=8)
    ax.set(xlabel='Hospital world X [m]',ylabel='Hospital world Y [m]')
    ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,1.22),ncol=3,fontsize=9,frameon=False)
    fig.suptitle('H05 modified-copy smoke | measured robot path',y=.98,fontsize=16)
    fig.text(.5,.06,'Original Hospital world origin; +Z up. C# marks an actual observation pose. Scene walls are not drawn.',ha='center',fontsize=10)
    fig.text(.5,.025,f'Closest goal distance {goal_distance.min():.3f} m; final distance {goal_distance[-1]:.3f} m. Goal entry does not mean the robot stopped there.',ha='center',fontsize=10)
    fig.subplots_adjust(left=.07,right=.97,bottom=.15,top=.79)
    fig.savefig(output/'trajectory.png',dpi=170);plt.close(fig)

    fig,ax=plt.subplots(figsize=(11,4.2))
    ax.plot(t,goal_distance,color='#126c9c',lw=2)
    ax.axhline(2,color='#b38a00',ls='--',label='Strict goal criterion: distance <2 m')
    if first is not None:
        ax.axvline(t[first],color='#176b45',ls=':',label=f'First entry {t[first]:.2f} s')
    ax.set(xlabel='Simulation time [s]',ylabel='Distance to goal [m]',title='The fixed 48-prediction run continues after goal entry')
    ax.grid(alpha=.2);ax.legend(frameon=False);fig.tight_layout()
    fig.savefig(output/'goal-distance.png',dpi=160);plt.close(fig)

    near=int(np.argmin(np.linalg.norm(obs_xy-human,axis=1)))
    entry_obs=int(np.argmin(np.abs(obs_t-t[first]))) if first is not None else len(events)-1
    selected=sorted(set([0,11,max(0,near-1),near,entry_obs,len(events)-1]))
    fig,axes=plt.subplots(2,3,figsize=(14,6.9))
    for ax in axes.flat:ax.axis('off')
    for ax,i in zip(axes.flat,selected):
        with Image.open(source/events[i]['rgb_observation_reference']) as image:
            ax.imshow(image)
        ax.set_title(f'C{i+1} | sim {obs_t[i]:.3f} s',fontsize=11)
    fig.suptitle('Saved RGB observations | original camera frames',fontsize=16)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(output/'rgb-contact-sheet.png',dpi=135);plt.close(fig)

    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    frames=[]
    for e in events:
        with Image.open(source/e['rgb_observation_reference']) as original:
            rgb=original.convert('RGB');rgb.thumbnail((960,540),Image.Resampling.LANCZOS)
        canvas=Image.new('RGB',(960,572),'#14212c');canvas.paste(rgb,(0,32))
        ImageDraw.Draw(canvas).text((12,7),f'C{e["request_id"]:02d} | sim {e["observation"]["sim_time"]:.3f} s | saved RGB observation',font=font,fill='white')
        frames.append(canvas.quantize(colors=192))
    durations=np.r_[np.diff(obs_t),t[-1]-obs_t[-1]]
    frames[0].save(output/'rgb-observations.gif',save_all=True,append_images=frames[1:],
                   duration=[max(20,round(1000*d)) for d in durations],loop=0,disposal=2,optimize=False)
    if any(digest(source/p)!=h for p,h in protected.items()):
        raise ValueError('Source changed during saved-only rendering')
    if set(protected)!={str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()}:
        raise ValueError('Source inventory changed during saved-only rendering')
    report={'source':str(source),'request_count':len(events),'dense_pose_count':len(rows),
        'first_goal_entry_sim_s':float(t[first]) if first is not None else None,
        'closest_goal_distance_m':float(goal_distance.min()),'final_goal_distance_m':float(goal_distance[-1]),
        'last_sim_time_s':float(t[-1]),'selected_rgb_request_ids':[i+1 for i in selected],
        'coordinate_convention':'Original Hospital world origin; X/Y in metres, +Z up; no coordinate conversion',
        'animation':'48 saved observations held until the next simulation observation time; GIF timing rounded to 10 ms; no intermediate frames reconstructed; preview downscaled',
        'raw_data_unchanged':True,'source_sha256':protected,'script_sha256':digest(Path(__file__)),
        'new_simulation_steps':0,'new_model_calls':0}
    with (output/'view-metadata.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    make_view(args.run_dir,args.output_dir)
