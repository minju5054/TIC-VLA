"""Saved-only E16 repeatability: native/controller/history/timing, no intervention."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.e16_zero_repeatability import audit_saved,selected_observation,RULES
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_no_cart_speed import analyze_run,RULES as NAV_RULES,INVENTORY
from research.analyze_handoffs import metrics as timing
from research.analyze_no_cart_reproduction import KEYS,signed
from research.hospital_no_cart_reproduction import validate_reference
from research.control import waypoint_command
from research.records import write_json,provenance
from research.compare_e16_cart_shift import write_csv
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.route_geometry import wrap


def serial(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if isinstance(v,dict):return {k:serial(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [serial(x) for x in v]
    return v


def stats(values):
    a=np.asarray([v for v in values if v is not None],float)
    return dict(count=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),min=float(a.min()),max=float(a.max())) if len(a) else dict(count=0,median=None,p95=None,min=None,max=None)


def extract(path,condition,primary,rules,hook_rules):
    audit=audit_saved(path,condition,hook_rules);run=load_saved(path)
    result=analyze_run(run,json.loads(NAV_RULES.read_text()),json.loads(INVENTORY.read_text()));nav=result['summary'];cfg=run['cfg'];g=cfg['spatial_turn_gate']
    selected=selected_observation(run,hook_rules);contact=nav['first_nonfloor_contact'];cut=contact['sim_time'] if contact else np.inf
    obs_by_tick={e['observation']['tick']:e['request_id'] for e in run['events']};rows=[];yaw_filter=None
    for e,c in zip(run['events'],run['chunks']):
        cmd,yaw_filter,idx=waypoint_command(c,yaw_filter,**cfg['controller']);np.testing.assert_allclose(cmd,e['controller_command'],rtol=0,atol=1e-12)
        pose=e['agent_pose_at_observation'];a=e['application'];row=run['robot'][a['tick']];nxt=run['robot'][min(a['tick']+1,len(run['robot'])-1)]
        rgb=path/e['rgb_observation_reference'];npy=path/'raw/requests'/f"request_{e['request_id']:06d}.npy"
        r=dict(observation_sim_time=e['observation']['sim_time'],application_sim_time=a['sim_time'],
            observation_tick=e['observation']['tick'],application_tick=a['tick'],observation_x_m=pose[0],observation_y_m=pose[1],observation_yaw_rad=pose[2],
            application_x_m=a['pose'][0],application_y_m=a['pose'][1],application_yaw_rad=a['pose'][2],
            observation_north_velocity_m_s=e['observation']['linear_velocity_world'][1],native_lookahead_index=idx,
            native_lookahead_forward_m=float(c[idx,0]),native_lookahead_left_m=float(c[idx,1]),controller_v=cmd[0],controller_w=cmd[1],yaw_filter_state=yaw_filter,
            observation_measured_forward_m_s=e['observation']['linear_velocity_body_flu'][0],
            measured_switch_yaw_rate_rad_s=float(row['body_yaw_rate_measured']),measured_next_tick_yaw_rate_rad_s=float(nxt['body_yaw_rate_measured']),
            measured_switch_forward_m_s=float(row['body_v_forward']),measured_next_tick_forward_m_s=float(nxt['body_v_forward']),
            detection_to_application_wall_ms=(a['monotonic_ns']-e['response_detected']['monotonic_ns'])/1e6,
            request_submit_to_inference_start_wall_ms=(e['inference_start']['monotonic_ns']-e['request_submit']['monotonic_ns'])/1e6,
            **timing(e),**{k:e[k] for k in KEYS},native_file=str(npy),rgb=str(rgb),
            native_sha256=hashlib.sha256(npy.read_bytes()).hexdigest(),rgb_sha256=hashlib.sha256(rgb.read_bytes()).hexdigest(),
            reasoning_text_sha256=hashlib.sha256((e['reasoning_text'] or '').encode()).hexdigest(),
            motion_history_sha256=hashlib.sha256(e['previous_waypoints_text'].encode()).hexdigest(),
            generation_start_request_id=obs_by_tick.get(e['reasoning_new_start_step']),completion_poll_request_id=obs_by_tick.get(e['reasoning_completion_poll_step']),
            precontact=e['observation']['sim_time']<cut and a['sim_time']<cut)
        for kind in ['observation','request_submit','inference_start','ready','response_detected','application']:
            r[kind+'_clocks']={k:e[kind][k] for k in ['wall_time_ns','monotonic_ns']}
        rows.append(r)
    window_start=np.flatnonzero(run['xy'][:,1]>=g['center_y_min_m']-hook_rules['trigger_band_south_m'])
    end=min([float(run['times'][-1]),cut,nav['corner_passage_completion_sim_time'] or np.inf])
    mask=(run['times']>=run['times'][window_start[0]])&(run['times']<=end) if len(window_start) else np.zeros(len(run['times']),bool)
    maximum=float(run['xy'][mask,1].max()) if mask.any() else None
    firstleft=nav['first_actual_left_interval'];lr=next((r for r in run['robot'] if firstleft and float(r['sim_time'])==firstleft['start_sim_time']),None)
    eligible_rows=[r for r in rows if r['precontact'] and r['observation_x_m']>g['exit_center_x_max_m'] and r['observation_north_velocity_m_s']>0]
    approach=[r for r in eligible_rows if 2<=r['observation_y_m']<13]
    tol=cfg['hospital_validation'];negative=[r for r in approach if r['controller_w'] < -tol['command_sign_tolerance_rad_s'] and r['native_lookahead_left_m'] < -tol['position_sign_tolerance_m']]
    bins=[]
    for low,high in zip(rules['progress_y_bin_edges_m'][:-1],rules['progress_y_bin_edges_m'][1:]):
        here=[r for r in approach if low<=r['observation_y_m']<high]
        rep=min(here,key=lambda r:(abs(r['observation_y_m']-(low+high)/2),r['request_id'])) if here else None
        bins.append(dict(low_y_m=low,high_y_m=high,count=len(here),request_ids=[r['request_id'] for r in here],
            median_w=float(np.median([r['controller_w'] for r in here])) if here else None,
            median_lookahead_left=float(np.median([r['native_lookahead_left_m'] for r in here])) if here else None,
            representative_request_id=rep['request_id'] if rep else None))
    timing_keys=['action_wall_s','observation_to_switch_wall_s','pending_sim_s','robot_translation_xy_m','detection_to_application_wall_ms']
    summary=dict(source_run=str(path),condition=condition,primary=primary,request_count=48,runtime_status='PASS',
        corner_success=bool(nav['left_turn_completed'] and not contact),west_hallway_passed=bool(nav['corner_passage_completion_sim_time'] is not None),
        corner_entry=nav['corner_entry'],left_start_sim_time=firstleft['start_sim_time'] if firstleft else None,
        left_start_pose=[float(lr[k]) for k in ['x','y','yaw']] if lr else None,
        passage_sim_time=nav['corner_passage_completion_sim_time'],max_corner_y_m=maximum,northward_overshoot_m=max(0.,maximum-g['center_y_max_m']) if maximum is not None else None,
        first_contact=contact,goal_entry_sim_time=nav['first_goal_region_entry']['sim_time'] if nav['first_goal_region_entry'] else None,
        final_pose=nav['final_pose'],selected_request_id=selected['request_id'] if selected else None,hook_triggered=audit['triggered'],
        additional_physics_ticks=0,selected_detection_to_application_wall_ms=rows[selected['request_id']-1]['detection_to_application_wall_ms'] if selected else None,
        timing_nonbootstrap={k:stats([r[k] for r in rows[1:]]) for k in timing_keys},bootstrap_timing={k:rows[0][k] for k in timing_keys},
        approach_rightward_request_ids=[r['request_id'] for r in negative],first_approach_rightward=negative[0] if negative else None,
        reasoning_new_start_request_ids=[r['request_id'] for r in rows if r['reasoning_new_start_step'] is not None],
        reasoning_completion_poll_steps=sorted(set(r['reasoning_completion_poll_step'] for r in rows if r['reasoning_completion_poll_step'] is not None)),
        reasoning_distinct_text_count=len(set(r['reasoning_text_sha256'] for r in rows)),kv_available_count=sum(bool(r['kv_cache_available']) for r in rows))
    return run,dict(summary=summary,navigation=nav,rows=rows,bins=bins,corner_mask=mask)


def persistent_divergence(a,b,tol):
    common=[]
    for x,y in zip(a['bins'],b['bins']):
        if x['count'] and y['count'] and all(signed(x[k],eps)!=signed(y[k],eps) for k,eps in [('median_w',tol['command_sign_tolerance_rad_s']),('median_lookahead_left',tol['position_sign_tolerance_m'])]):
            common.append(x['low_y_m'])
    return next((v for v in common if v+1 in common),None)


def pair_diagnostics(name_a,name_b,runs,results):
    from PIL import Image
    a,b=results[name_a],results[name_b];ra,rb=runs[name_a],runs[name_b];tol=ra['cfg']['hospital_validation']
    c1a,c1b=a['rows'][0],b['rows'][0];ia=np.asarray(Image.open(c1a['rgb']).convert('RGB'));ib=np.asarray(Image.open(c1b['rgb']).convert('RGB'))
    if ia.shape!=ib.shape:raise ValueError('Different source RGB resolution')
    posea=np.array(ra['events'][0]['agent_pose_at_observation']);poseb=np.array(rb['events'][0]['agent_pose_at_observation'])
    first_nonidentical_native=next((i+1 for i,(x,y) in enumerate(zip(ra['chunks'],rb['chunks'])) if not np.array_equal(x,y)),None)
    first_bin=persistent_divergence(a,b,tol)
    comparisons=[]
    for x,y in zip(a['bins'],b['bins']):
        if not x['count'] or not y['count']:continue
        p,q=a['rows'][x['representative_request_id']-1],b['rows'][y['representative_request_id']-1]
        comparisons.append(dict(low_y_m=x['low_y_m'],a_request_id=p['request_id'],b_request_id=q['request_id'],
            a_rgb=p['rgb'],b_rgb=q['rgb'],a_native=p['native_file'],b_native=q['native_file'],
            observation_xy_distance_m=float(np.hypot(p['observation_x_m']-q['observation_x_m'],p['observation_y_m']-q['observation_y_m'])),
            observation_yaw_difference_deg=float(np.degrees(wrap(p['observation_yaw_rad']-q['observation_yaw_rad']))),
            a_median_w=x['median_w'],b_median_w=y['median_w'],a_median_lookahead_left=x['median_lookahead_left'],b_median_lookahead_left=y['median_lookahead_left'],
            a_history_sha256=p['motion_history_sha256'],b_history_sha256=q['motion_history_sha256'],
            a_reasoning_start=p['reasoning_new_start_step'],b_reasoning_start=q['reasoning_new_start_step'],
            a_reasoning_poll=p['reasoning_completion_poll_step'],b_reasoning_poll=q['reasoning_completion_poll_step'],
            a_reasoning_sha256=p['reasoning_text_sha256'],b_reasoning_sha256=q['reasoning_text_sha256']))
    c1_native_equal=bool(np.array_equal(ra['chunks'][0],rb['chunks'][0]))
    first_pose_difference=next((i+1 for i,(x,y) in enumerate(zip(ra['events'],rb['events'])) if x['agent_pose_at_observation']!=y['agent_pose_at_observation']),None)
    return dict(first_native_equal=c1_native_equal,first_pose_difference_same_index_descriptive=first_pose_difference,
        first_history_equal=c1a['previous_waypoints_text']==c1b['previous_waypoints_text'],first_model_state_input_equal=c1a['robot_state_input']==c1b['robot_state_input'],
        first_reasoning_text_equal=c1a['reasoning_text_sha256']==c1b['reasoning_text_sha256'],a=name_a,b=name_b,both_primary=a['summary']['primary'] and b['summary']['primary'],
        opposite_navigation_outcomes=a['summary']['corner_success']!=b['summary']['corner_success'],
        first_rgb=dict(a=c1a['rgb'],b=c1b['rgb'],exact_pose_equal=bool(np.array_equal(posea,poseb)),pose_xy_distance_m=float(np.linalg.norm(posea[:2]-poseb[:2])),
            pixels_equal=bool(np.array_equal(ia,ib)),mae_0_255=float(np.abs(ia.astype(float)-ib).mean()),changed_pixel_fraction=float(np.any(ia!=ib,axis=2).mean())),
        first_native_rmse_same_pose_m=float(np.sqrt(np.mean(np.sum((ra['chunks'][0].astype(float)-rb['chunks'][0])**2,axis=1)))) if np.array_equal(posea,poseb) else None,
        first_nonidentical_native_same_index_descriptive=first_nonidentical_native,
        first_two_bin_steering_divergence_low_y_m=first_bin,common_progress_bins=comparisons,
        note='Earliest numeric inequality is not meaningful/causal onset; same-index later chunks use different observation frames. No native-local difference is used to rank frame-invariant revisions.')


def csv_rows(rows):
    return [{k:json.dumps(serial(v),ensure_ascii=False) if isinstance(v,(dict,list,tuple)) else serial(v) for k,v in r.items()} for r in rows]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    receipt=json.loads((a.plan/'receipt.json').read_text());rules=json.loads(RULES.read_text());hook=receipt['hook_rules']
    paths=[ROOT/'outputs'/t['run_id'] for t in receipt['trials']]+[Path(p) for p in receipt['reference_runs']]
    out=validate_destination(a.output_dir,paths);hashes=evidence_hashes([*paths,RULES,a.plan,Path(__file__),ROOT/'research/plot_e16_repeatability.py'])
    runs={};results={}
    for i,path in enumerate(paths):
        primary=i<6;condition=receipt['trials'][i]['condition'] if primary else 'NO_HOOK' if i<8 else 'ZERO_HOOK'
        name=f"R{i+1}_{condition}" if primary else ['Ref_A','Ref_B','Prev_zero_1','Prev_zero_2'][i-6]
        run,d=extract(path,condition,primary,rules,hook);validate_reference(json.loads((a.plan/'config.json').read_text()),run['cfg'])
        runs[name]=run;results[name]=d
    pairs=[pair_diagnostics(x,y,runs,results) for x,y in itertools.combinations(results,2)]
    out.mkdir(parents=True);(out/'figures').mkdir();(out/'derived').mkdir()
    for name,d in results.items():
        write_json(out/'derived'/f'{name}_requests.json',serial(d['rows']));write_csv(out/'derived'/f'{name}_requests.csv',csv_rows(d['rows']))
        write_json(out/'derived'/f'{name}_navigation.json',serial(d['navigation']));write_csv(out/'derived'/f'{name}_progress_bins.csv',csv_rows(d['bins']))
        write_csv(out/'derived'/f'{name}_executed.csv',runs[name]['robot'])
    counts={c:dict(runs=3,corner_success=sum(d['summary']['corner_success'] for d in list(results.values())[:6] if d['summary']['condition']==c),
        corner_failure=sum(not d['summary']['corner_success'] for d in list(results.values())[:6] if d['summary']['condition']==c)) for c in ['NO_HOOK','ZERO_HOOK']}
    summary=dict(runs={k:d['summary'] for k,d in results.items()},counts=counts,comparison_note='Historical references excluded from primary counts. Descriptive six-run evidence, not causal/statistical effect.')
    write_json(out/'summary.json',serial(summary));write_csv(out/'run_metrics.csv',csv_rows([dict(run=k,**d['summary']) for k,d in results.items()]))
    write_json(out/'divergence.json',serial(pairs));write_csv(out/'c12_c18_descriptive.csv',csv_rows([dict(run=k,**r) for k,d in results.items() for r in d['rows'] if 12<=r['request_id']<=18]))
    # Earliest defined progress disagreement; no selection by error magnitude.
    candidates=[p for p in pairs if p['both_primary'] and p['opposite_navigation_outcomes'] and p['first_two_bin_steering_divergence_low_y_m'] is not None]
    chosen=min(candidates,key=lambda p:(p['first_two_bin_steering_divergence_low_y_m'],p['a'],p['b'])) if candidates else None
    write_json(out/'earliest_divergence.json',serial(dict(selected=chosen,rule=rules['divergence_rule'],undefined_reason=None if chosen else 'No primary success/failure pair meets frozen two-bin directional rule')))
    from research.plot_e16_repeatability import figures
    figures(out/'figures',runs,results,pairs,chosen,json.loads(INVENTORY.read_text()))
    verify_hashes(hashes)
    write_json(out/'metadata.json',dict(source_sha256=hashes,source_files_unchanged=True,provenance=provenance(ROOT),
        rule_config=rules,model_calls=0,physics_steps=0,native_frame='Body at own observation; forward/left metres; yaw CCW world +X, quaternion wxyz; world projection at own observation only.',
        timing_note=rules['timing_note']))
    print(json.dumps(serial(counts)));print(str(out))


if __name__=='__main__':main()
