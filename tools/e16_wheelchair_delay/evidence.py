"""90-prediction evidence contract, including held and unapplied responses."""
import csv
import json
from pathlib import Path
import numpy as np
from research.control import validate_chunk, waypoint_command
from research.records import write_json
from tools.e16_wheelchair_delay.common import read, digest


def rows(path):
    with Path(path).open() as f:return list(csv.DictReader(f))


def verify_state(state, robot):
    r=robot[state['tick']]
    values=np.array([float(r[k]) for k in ('x','y','yaw')])
    if not np.allclose(values,state['pose'],rtol=0,atol=1e-8) or abs(float(r['sim_time'])-state['sim_time'])>1e-8:
        raise ValueError('Recorded state and robot row disagree')
    if not np.allclose([float(r[k]) for k in ('qw','qx','qy','qz')],state['quaternion_wxyz'],rtol=0,atol=1e-8):
        raise ValueError('Quaternion and CSV disagree')


def pending_rows(robot,rid,obs,end,old,command,wheels,detected):
    selected=[r for r in robot if r['pending_request_id']==str(rid)]
    if [int(r['tick']) for r in selected]!=list(range(obs['tick']+1,end['tick']+1)):
        raise ValueError('Missing or extra pending/held physics ticks')
    for r in selected:
        targets=[float(r[k]) for k in ('target_v','target_w','applied_v','applied_w','wheel_target_left','wheel_target_right')]
        if r['active_control_source_request_id']!=str(old or '') or not np.allclose(targets,[*command,*command,*wheels],rtol=0,atol=1e-12):
            raise ValueError('OLD command/source/wheel targets changed')
        start,finish=int(r['physics_step_start_monotonic_ns']),int(r['monotonic_ns'])
        if not obs['monotonic_ns']<=start<=finish<=end['monotonic_ns']:
            raise ValueError('Pending/held tick clocks invalid')
        if int(r['tick'])<=detected['tick'] and finish>detected['monotonic_ns']:
            raise ValueError('Inference pending tick beyond response detection')
    return selected


def load_run(path, require_complete=True):
    path=Path(path);meta=read(path/'metadata.json');cfg=meta['config']
    if cfg['simulation']['predictions']!=90:raise ValueError('This experiment requires 90 predictions')
    runtime=read(path/'inference-runtime.json')
    if not runtime['strict_checkpoint'] or runtime['simulation_app_started'] or cfg['simulation']['pause_physics_during_inference'] or meta['command_variant']!='direct':
        raise ValueError('Model/continuous/direct contract mismatch')
    robot=rows(path/'robot_state.csv');ticks=[int(r['tick']) for r in robot]
    times=np.array([float(r['sim_time']) for r in robot]);xy=np.array([[float(r[k]) for k in ('x','y')] for r in robot])
    yaw=np.unwrap([float(r['yaw']) for r in robot])
    if ticks!=list(range(len(robot))) or not np.allclose(np.diff(times),cfg['simulation']['dt'],rtol=0,atol=1e-7):
        raise ValueError('Noncontiguous robot state')
    files=sorted((path/'raw/requests').glob('*.json'))
    if len(files)>90 or require_complete and len(files)!=90 or not files:
        raise ValueError('Incomplete/extra applied native predictions')
    if {p.stem for p in files}!={p.stem for p in (path/'raw/requests').glob('*.npy')}:
        raise ValueError('Unpaired native files')
    events=[];chunks=[];yf=None;inside=[]
    for rid,file in enumerate(files,1):
        e=read(file);c=validate_chunk(np.load(file.with_suffix('.npy'),allow_pickle=False))
        if (e['request_id']!=rid or e['bootstrap']!=(rid==1) or e['old_control_source_request_id']!=(rid-1 if rid>1 else None)
            or e['native_frame']!='observation_body' or e['native_axes']!=['forward','left'] or e['native_units']!='meters'
            or e['native_tensor_shape']!=[1,30,2] or e['raw_action_shape']!=[30,2] or e['native_yaw'] is not None
            or e['agent_pose_at_ready'] is not None or e['agent_pose_at_observation']!=e['observation']['pose']):
            raise ValueError('Native/anchor/source identity mismatch')
        if e['instruction']!=cfg['instruction']:raise ValueError('Instruction changed')
        cmd,yf,index=waypoint_command(c,yf,**cfg['controller'])
        if not np.allclose(cmd,e['controller_command'],rtol=0,atol=1e-12) or index!=e['lookahead_index'] or abs(yf-e['yaw_filter_state'])>1e-12:
            raise ValueError('Controller replay mismatch')
        if rid>1 and (e['old_controller_command']!=events[-1]['controller_command'] or e['old_wheel_commands']!=events[-1]['wheel_commands']):
            raise ValueError('OLD differs from previous accepted FRESH')
        stamp=[e[k]['monotonic_ns'] for k in ('observation','request_submit','inference_start','ready','response_detected','application')]
        if stamp!=sorted(stamp):raise ValueError('Event clock order mismatch')
        for key in ('observation','response_detected','application'):verify_state(e[key],robot)
        if not (path/e['rgb_observation_reference']).is_file():raise ValueError('Missing observation RGB')
        selected=pending_rows(robot,rid,e['observation'],e['application'],rid-1 if rid>1 else None,
                              e['old_controller_command'] if rid>1 else [0.,0.],
                              e['old_wheel_commands'] if rid>1 else [0.,0.],e['response_detected'])
        inside.append(sum(e['inference_start']['monotonic_ns']<=int(r['physics_step_start_monotonic_ns']) and int(r['monotonic_ns'])<=e['ready']['monotonic_ns'] for r in selected))
        events.append(e);chunks.append(c)
    hold=read(path/'raw/delay/hold.json') if (path/'raw/delay/hold.json').exists() else None
    received=read(path/'raw/delay/received.json') if hold else None
    if hold:
        rid=hold['request_id'];native=validate_chunk(np.load(path/'raw/delay/received.npy',allow_pickle=False))
        import hashlib
        if hashlib.sha256(native.tobytes()).hexdigest()!=hold['native_sha256'] or received['request']['observation']['pose']!=hold['anchor'] or not hold['fresh_unchanged']:
            raise ValueError('Held FRESH mutated')
        verify_state(hold['start'],robot);verify_state(hold['end'],robot)
        actual=hold['end']['tick']-hold['start']['tick']
        if actual!=hold['realized_ticks'] or len(hold['hold_states'])!=actual:
            raise ValueError('Hold tick mismatch')
        for i,state in enumerate(hold['hold_states'],hold['start']['tick']+1):
            if state['tick']!=i:raise ValueError('Noncontiguous hold states')
            verify_state(state,robot)
        if abs(hold['realized_delay_sim_s']-actual*cfg['simulation']['dt'])>1e-7:raise ValueError('Hold sim grid mismatch')
        if hold['fresh_application_authorized']:
            if actual!=hold['requested_ticks'] or len(events)<rid:raise ValueError('Incomplete authorized hold')
            e=events[rid-1]
            if not np.array_equal(chunks[rid-1],native) or e['application']['tick']!=hold['end']['tick'] or e['late_handoff']!=hold:
                raise ValueError('Held FRESH was not identically applied')
        else:
            if len(events)!=rid-1:raise ValueError('Stopped FRESH must not be applied')
            pending_rows(robot,rid,received['request']['observation'],hold['end'],received['old']['old_control_source_request_id'],
                         received['old']['old_controller_command'],received['old']['old_wheel_commands'],received['response_detected'])
    for i,e in enumerate(events):
        start=e['application']['tick']+1;end=events[i+1]['application']['tick'] if i+1<len(events) else len(robot)-1
        for r in robot[start:end+1]:
            actual=[float(r[k]) for k in ('target_v','target_w','applied_v','applied_w','wheel_target_left','wheel_target_right')]
            if r['active_control_source_request_id']!=str(e['request_id']) or not np.allclose(actual,[*e['controller_command'],*e['controller_command'],*e['wheel_commands']],rtol=0,atol=1e-12):
                raise ValueError('Applied target retention mismatch')
    return dict(path=path,metadata=meta,cfg=cfg,robot=robot,times=times,xy=xy,yaw=yaw,
                events=events,chunks=chunks,inside_action_ticks=inside,hold=hold,received=received)


def validate_continuous(path):
    run=load_run(path,True);valid=sum(n>0 for n in run['inside_action_ticks'][1:])>=3
    result=dict(continuous_handoff_validated=valid,expected_predictions=90,applied_predictions=len(run['events']),
                additional_hold_validated=run['hold'] is not None,
                physics_ticks_inside_action_by_request=run['inside_action_ticks'],
                transport_decision='90-call actual OLD retention and separate post-response hold checked')
    out=Path(path)/'derived/handoff';out.mkdir(parents=True,exist_ok=False);write_json(out/'summary.json',result)
    return result
