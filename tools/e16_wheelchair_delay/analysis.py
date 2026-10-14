"""Saved-only counterfactual and actual delay evaluation. Never imports Graph/Isaac."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from tools.e16_wheelchair_delay.common import *
from tools.e16_wheelchair_delay.evidence import load_run,rows
from research.audit_e16_wheelchair_handoffs import crossing

ROOT=Path(__file__).resolve().parents[2]


def geometry(cfg,wheels):
    inventory_path=ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'
    if digest(inventory_path)!=cfg['spatial_turn_gate']['source_inventory_sha256']:raise ValueError('Geometry source changed')
    return build_geometry(cfg,wheels,read(inventory_path))[0]


def paired_boundaries(world,nominal,B0,B1,t0,t1,d0,d1,policy,boxes,radius,wheel,observation_yaw):
    fixed=max(float(nominal[0]),float(t0))
    before=boundary_metrics(world,nominal,B0,t0,d0,policy,fixed,observation_yaw)
    after=boundary_metrics(world,nominal,B1,t1,d1,policy,fixed,observation_yaw)
    result=dict(before=before,after=after,delta={},fixed_reference_rule='max(first native nominal time, hold-start sim time); one common reference for both B',
                note='time-aligned errors use each actual boundary time; fixed-reference errors isolate spatial change')
    for key in ('position_error_m','direction_error_rad','body_yaw_error_rad','longitudinal_error_m','lateral_error_m','fixed_reference_position_error_m'):
        result['delta'][key]=after[key]-before[key] if before[key] is not None and after[key] is not None else None
    for name,pose,time in (('before',B0,t0),('after',B1,t1)):
        _,future,_=remaining(nominal,world,time);indices=np.flatnonzero(nominal>time+1e-9)
        connector=np.vstack([pose[:2],world[indices[0]]]) if len(indices) else np.empty((0,2))
        result[name].update(fresh_proxy=proxy_clearance(future,boxes,radius,policy['proxy_sample_step_m']),
            boundary_proxy=proxy_clearance([pose[:2]],boxes,radius,policy['proxy_sample_step_m']),
            connector_proxy=proxy_clearance(connector,boxes,radius,policy['proxy_sample_step_m']),
            connector_xy_m=connector.tolist(),fresh_remaining_xy_m=future.tolist(),
            predicted_passage=crossing(future,wheel['center_world_xyz'],wheel['yaw_rad'],wheel['footprint_half_m'][1],radius))
    result['side_preserved']=(before['predicted_passage'].get('side')==after['predicted_passage'].get('side')
                             if before['predicted_passage']['status']=='CROSSES' and after['predicted_passage']['status']=='CROSSES' else None)
    return result


def stage_a(suite):
    protocol=read(suite/'protocol.json');source=ROOT/'outputs'/protocol['reference_run']
    protected=read(suite/'reference-before.json');verify_snapshot(source,protected)
    run=load_run(source,True);e=run['events'][12];old=run['events'][11]
    cfg=run['cfg'];world=to_world(run['chunks'][12],e['observation']['pose']);nominal=e['observation']['sim_time']+.1*np.arange(1,31)
    B0=np.asarray(e['response_detected']['pose']);t0=e['response_detected']['sim_time'];wheels=read(source/'wheelchairs_asset.json')
    boxes=geometry(cfg,wheels);radius=cfg['spatial_turn_gate']['robot_conservative_radius_m']
    d0=motion_direction(run['xy'],e['response_detected']['tick'])['yaw_rad'];result=[]
    for seconds in protocol['stage_a_extra_delay_s']:
        n=ticks_for(seconds);states=np.array([unicycle(B0,old['controller_command'],i/60) for i in range(n+1)])
        B1=states[-1]
        delta=states[-1,:2]-states[max(0,n-6),:2]
        d1=d0 if n==0 else float(np.arctan2(delta[1],delta[0]))
        paired=paired_boundaries(world,nominal,B0,B1,t0,t0+seconds,d0,d1,protocol['policy'],boxes,radius,wheels['C10'],e['observation']['pose'][2])
        result.append(dict(extra_delay_s=seconds,ticks=n,kinematic_old_path_m=abs(old['controller_command'][0])*seconds,
            kinematic_net_displacement_m=float(np.linalg.norm(B1[:2]-B0[:2])),rollout_xy_m=states[:,:2].tolist(),
            paired=paired,method='Constant recorded OLD (v,w) unicycle from recorded response-detection pose; not PhysX evidence'))
    out=suite/'stage-a';out.mkdir(exist_ok=False)
    write(out/'result.json',dict(status='PASS',reference_old_id=12,reference_fresh_id=13,reference_observation=e['observation'],
        response_detected=e['response_detected'],old_command=old['controller_command'],fresh_sha256=digest(source/'raw/requests/request_000013.npy'),
        results=result,actual_positive_levels_authorized_s=[.25],thresholds_unchanged=True))
    verify_snapshot(source,protected)
    print(json.dumps([dict(delay_s=r['extra_delay_s'],OLD_path_cm=100*r['kinematic_old_path_m'],
                          position_error_cm=None if r['paired']['after']['position_error_m'] is None else 100*r['paired']['after']['position_error_m'],
                          remaining_s=r['paired']['after']['remaining_horizon_s']) for r in result],indent=2))


def decide(paired,hold,primary,policy):
    if hold and hold['stop_status']=='CONTACT_DURING_HOLD':return 'CONTACT_DURING_HOLD'
    if hold and hold['stop_status'] or paired is None or not primary:return 'INSUFFICIENT_EVIDENCE'
    b,a=paired['before'],paired['after']
    if not is_clear(a['fresh_proxy'],policy):return 'UNSAFE_OR_UNQUALIFIED_FRESH'
    if not a['nominal_valid'] or not b['nominal_valid'] or not a['tangent']['valid'] or not b['tangent']['valid'] or a['direction_error_rad'] is None:
        return 'INSUFFICIENT_EVIDENCE'
    if (a['predicted_passage']['status']=='CROSSES' and is_clear(b['boundary_proxy'],policy)
        and is_clear(a['boundary_proxy'],policy) and is_clear(b['connector_proxy'],policy)
        and a['connector_proxy']['proxy_overlap_observed'] and a['remaining_horizon_s']>.1):
        return 'POTENTIAL_RECONCILIATION_CASE'
    if (is_clear(a['connector_proxy'],policy) and is_clear(a['boundary_proxy'],policy)
        and a['position_error_m']<=policy['large_position_gap_m'] and a['direction_error_rad']<=policy['large_direction_gap_rad']
        and paired['side_preserved'] is True):return 'ROBUST_HANDOFF'
    return 'MISMATCH_WITHOUT_DEMONSTRATED_RISK'


def analyze_trial(suite,index):
    setup=suite/f'trial-{index}';process=read(setup/'process-result.json');source=Path(process['run_directory'])
    protocol=read(suite/'protocol.json');cfg=read(suite/'config.json');out=setup/'analysis';out.mkdir(exist_ok=False)
    protected=snapshot(source)
    try:
        for p,h in read(source/'raw-sha256.json').items():
            if digest(source/p)!=h:raise ValueError('Raw source hash mismatch')
        status=read(source/'summary.json') if (source/'summary.json').exists() else {}
        run=load_run(source,require_complete=status.get('status')=='PASS')
        if run['cfg']!=cfg:raise ValueError('Navigation configuration changed')
        robot,xy,t,yaw=run['robot'],run['xy'],run['times'],run['yaw']
        contacts=[json.loads(s) for s in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
        nonfloor=[e for e in contacts if contact_is_nonfloor(e,cfg['robot']['prim'])]
        gd=np.linalg.norm(xy-np.asarray(cfg['scene']['goal'][:2]),axis=1);inside=np.flatnonzero(gd<2.)
        goal=float(t[inside[0]]) if len(inside) else None;primary_end=goal if goal is not None else float(t[-1])
        before_goal=[e for e in nonfloor if e['sim_time']<=primary_end]
        wheels=read(source/'wheelchairs_asset.json');expected=read(suite/'runtime-reference.json')['wheelchairs_expected']
        if wheels!=expected:raise ValueError('Wheelchair geometry differs from reference')
        actor=rows(source/'raw/wheelchairs_state.csv')
        if len(actor)!=2*len(robot):raise ValueError('Missing wheelchair rows')
        navigation={}
        for name,w in wheels.items():
            own=[r for r in actor if r['id']==name]
            if [r['tick'] for r in own]!=[r['tick'] for r in robot] or any(float(r['matrix_max_error'])!=0 or not np.allclose([float(r[k]) for k in ('x','y','z')],w['center_world_xyz'],rtol=0,atol=1e-10) for r in own):
                raise ValueError('Wheelchair moved or rows misaligned')
            along=(xy-np.asarray(w['center_world_xyz'][:2])) @ rotation(w['yaw_rad'])[:,0]
            passed=np.flatnonzero(along>cfg['spatial_turn_gate']['robot_conservative_radius_m']+max(w['footprint_half_m']))
            pt=float(t[passed[0]]) if len(passed) else None
            hits=[e for e in nonfloor if any(e[k].startswith(w['copy_prim']) for k in ('collider0','collider1'))]
            relevant=[e for e in hits if e['sim_time']<=(pt if pt is not None else primary_end)]
            navigation[name]=dict(passage_sim_s=pt,passed=pt is not None,contact_free_passage=pt is not None and not relevant,
                                 contact_reports_through_passage=len(relevant),all_contact_reports=len(hits),
                                 min_center_distance_m=float(np.linalg.norm(xy-np.asarray(w['center_world_xyz'][:2]),axis=1).min()))
        candidates=[];checks=[]
        for e in run['events']:
            obs=e['observation'];passed=any(fully_passed([*xy[i],yaw[i]],protocol['trigger']) for i in range(obs['tick']+1))
            prior=any(c['sim_time']<=obs['sim_time']+1e-9 for c in nonfloor)
            ck=trigger_check(obs,protocol['trigger'],e['old_control_source_request_id'],passed,prior)
            checks.append(dict(request_id=e['request_id'],**ck))
            if ck['eligible']:candidates.append(e['request_id'])
        hold=run['hold'];received=run['received']
        if hold and not hold['fresh_application_authorized']:
            req=received['request'];obs=req['observation'];passed=any(fully_passed([*xy[i],yaw[i]],protocol['trigger']) for i in range(obs['tick']+1))
            ck=trigger_check(obs,protocol['trigger'],received['old']['old_control_source_request_id'],passed,any(c['sim_time']<=obs['sim_time']+1e-9 for c in nonfloor))
            checks.append(dict(request_id=req['request_id'],**ck))
            if ck['eligible']:candidates.append(req['request_id'])
        selected=candidates[0] if candidates else None
        mode=run['metadata']['trial_mode'];hook_valid=True
        if mode!='NO_HOOK':
            calls=read(source/'raw/hook-calls.json')
            hook_valid=calls['selected_request_id']==selected and (selected is None or hold is not None and hold['request_id']==selected)
            if not hook_valid:raise ValueError('Runtime hook differs from prespecified offline trigger')
        paired=transport=detail=None
        if selected is not None:
            if selected<=len(run['events']):
                e=run['events'][selected-1];chunk=run['chunks'][selected-1]
                B0=hold['start'] if hold else e['response_detected'];B1=hold['end'] if hold else e['application']
            else:
                e={**received['request'],**received['response'],**received['old'],
                   'response_detected':received['response_detected'],'controller_command':received['controller_command']}
                chunk=np.load(source/'raw/delay/received.npy',allow_pickle=False);B0=hold['start'];B1=hold['end']
            obs=e['observation'];det=e['response_detected'];a,b=obs['tick'],B1['tick']
            world=to_world(chunk,obs['pose']);nominal=obs['sim_time']+.1*np.arange(1,31)
            d0=motion_direction(xy,B0['tick']);d1=motion_direction(xy,B1['tick']);radius=cfg['spatial_turn_gate']['robot_conservative_radius_m']
            boxes=geometry(cfg,wheels)
            paired=paired_boundaries(world,nominal,B0['pose'],B1['pose'],B0['sim_time'],B1['sim_time'],d0['yaw_rad'],d1['yaw_rad'],protocol['policy'],boxes,radius,wheels['C10'],obs['pose'][2])
            inside_ticks=[i for i in range(a+1,b+1) if e['inference_start']['monotonic_ns']<=int(robot[i]['physics_step_start_monotonic_ns']) and int(robot[i]['monotonic_ns'])<=e['ready']['monotonic_ns']]
            transport=dict(total=movement(xy,yaw,a,b),observation_to_detected=movement(xy,yaw,a,det['tick']),
                hold=movement(xy,yaw,B0['tick'],B1['tick']),inference_fully_enclosed_ticks=inside_ticks,
                inference_path_m=float(sum(np.linalg.norm(xy[i]-xy[i-1]) for i in inside_ticks)),
                inference_yaw_change_rad=float(sum(yaw[i]-yaw[i-1] for i in inside_ticks)),
                inference_net_displacement_m=float(np.linalg.norm(sum((xy[i]-xy[i-1] for i in inside_ticks),np.zeros(2)))),
                exact_service_ready_pose_available=False)
            detail=dict(request_id=selected,old_id=selected-1,observation=obs,response_detected=det,
                inference_start=e['inference_start'],ready=e['ready'],hold_start=B0,hold_end=B1,
                actual_application=run['events'][selected-1]['application'] if selected<=len(run['events']) else None,
                old_command=e['old_controller_command'],fresh_command=e['controller_command'],
                delta_command=(np.array(e['controller_command'])-e['old_controller_command']).tolist(),
                direction_before=d0,direction_after=d1,world_xy_m=world.tolist(),nominal_sim_times=nominal.tolist(),
                measured_boundary_velocity_body=B1['linear_velocity_body_flu'],measured_boundary_yaw_rate=B1['angular_velocity_body_flu'][2],
                detected_to_application_wall_s=(run['events'][selected-1]['application']['monotonic_ns']-det['monotonic_ns'])/1e9 if selected<=len(run['events']) else None)
        primary=selected is not None and (goal is None or detail['hold_end']['sim_time']<goal)
        classification=decide(paired,hold,primary,protocol['policy'])
        valid=process['returncode']==0 and status.get('status')=='PASS' and len(run['events'])==90
        continuous=sum(n>0 for n in run['inside_action_ticks'][1:])>=3
        baseline_pass=bool(valid and continuous and selected is not None and goal is not None and not before_goal and all(n['contact_free_passage'] for n in navigation.values()) and hook_valid)
        result=dict(index=index,mode=mode,delay_s=run['metadata']['extra_hold_s'],runtime_valid=valid,applied_predictions=len(run['events']),
            delivered_native_predictions=len(run['events'])+(1 if hold and not hold['fresh_application_authorized'] else 0),
            trigger_status='TRIGGER_REACHED' if selected else 'TRIGGER_NOT_REACHED',trigger_request_id=selected,
            baseline_pass=baseline_pass,classification=classification,primary_handoff=primary,
            safety_stop=status.get('status')=='SAFETY_STOP',stop_reason=status.get('reason'),
            navigation=navigation,goal_entry_sim_s=goal,closest_goal_m=float(gd.min()),final_goal_m=float(gd[-1]),
            before_goal_nonfloor_callbacks=len(before_goal),total_nonfloor_callbacks=len(nonfloor),
            first_nonfloor_contact=nonfloor[0] if nonfloor else None,first_nonfloor_contact_kind=contact_kind(nonfloor[0]) if nonfloor else None,
            total_path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
            reverse_distance_m=float(sum(max(0.,-float(r['body_v_forward']))*cfg['simulation']['dt'] for r in robot[1:])),
            paired=paired,transport=transport,detail=detail,continuous_validated=continuous,source=str(source),
            no_hook_loop_unchanged=True,config_difference=[],original_raw_files_verified=len(read(source/'raw-sha256.json')))
        write(out/'trigger-accounting.json',checks)
        write(out/'source-before.json',protected)
        verify_snapshot(source,protected);write(out/'result.json',result)
        print(json.dumps({k:result[k] for k in ('index','mode','runtime_valid','trigger_request_id','baseline_pass','classification','goal_entry_sim_s','before_goal_nonfloor_callbacks')},indent=2))
        return result
    except Exception as exc:
        verify_snapshot(source,protected)
        result=dict(index=index,mode=PLAN[index-1][0],delay_s=PLAN[index-1][1],runtime_valid=False,baseline_pass=False,
                    classification='INSUFFICIENT_EVIDENCE',error=repr(exc),source=str(source),safety_stop=True)
        write(out/'result.json',result)
        raise


def baseline_gate(suite):
    paths=[suite/f'trial-{i}/analysis/result.json' for i in range(1,5)]
    results=[read(p) for p in paths]
    passed=all(r['baseline_pass'] for r in results)
    record=dict(status='BASELINE_STABLE' if passed else 'BASELINE_UNSTABLE',
                passed_indices=[r['index'] for r in results if r['baseline_pass']],
                failed_indices=[r['index'] for r in results if not r['baseline_pass']],
                result_hashes={str(p.relative_to(suite)):digest(p) for p in paths},positive_delay_allowed=passed)
    write(suite/'stage-b-gate.json',record);print(json.dumps(record,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('phase',choices=['stage-a','trial','gate']);p.add_argument('--suite',type=Path,required=True);p.add_argument('--index',type=int)
    a=p.parse_args();s=a.suite.resolve()
    if a.phase=='stage-a':stage_a(s)
    elif a.phase=='trial':analyze_trial(s,a.index)
    else:baseline_gate(s)
