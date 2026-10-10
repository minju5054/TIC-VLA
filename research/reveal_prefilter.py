"""Render-free, model-free possibility gates; rays never label semantic visibility."""
import numpy as np
from research.reveal_window import temporal_metrics


def human_samples(position, yaw, rules):
    c,s=np.cos(yaw),np.sin(yaw);rotation=np.array([[c,-s],[s,c]])
    samples=[]
    for group,height in rules['sample_heights_m'].items():
        for i,offset in enumerate(rules['sample_xy_offsets_m']):
            xy=rotation@np.array(offset)+position[:2]
            samples.append({'name':f'{group}_{i}','group':group,'world':[*map(float,xy),float(position[2]+height)]})
    for i,point in enumerate(rules['auxiliary_arm_points_m']):
        xy=rotation@np.array(point[:2])+position[:2]
        samples.append({'name':f'arm_{i}','group':'arm','world':[*map(float,xy),float(position[2]+point[2])]})
    return samples


def possible_projection(point, camera_matrix, horizontal_slope, vertical_slope, factor):
    local=np.r_[point,1.]@np.linalg.inv(camera_matrix)
    depth=-local[2]
    return bool(depth>0 and abs(local[0])<=factor*horizontal_slope*depth
                and abs(local[1])<=factor*vertical_slope*depth)


def ray_transition(rows, fresh_id, groups, minimum_prior=3):
    """Permissive possibility only, retaining partially occluded OLD bundles."""
    if fresh_id<=minimum_prior:return {'pass':False,'reason':'insufficient_prior_observations'}
    prior=rows[fresh_id-minimum_prior-1:fresh_id-1]
    fresh=rows[fresh_id-1];old=rows[fresh_id-2]
    blocked_groups=[]
    for row in prior:
        blocked_groups.append([g for g in ['head','torso','leg']
                               if not any(exposed for exposed,group in zip(row['exposed'],groups) if group==g)])
    newly=[i for i,(a,b) in enumerate(zip(old['exposed'],fresh['exposed'])) if b and not a]
    passed=bool(all(blocked_groups) and any(fresh['exposed']) and newly)
    return {'pass':passed,'prior_nonexposed_groups':blocked_groups,'newly_exposed_sample_indices':newly,
            'fresh_exposed_sample_count':sum(fresh['exposed']),
            'old_exposed_sample_count':sum(old['exposed']),
            'reason':'possible_transition' if passed else 'no_sampled_occlusion_release'}


def temporal_pairs(run, candidate, config):
    rows=[]
    for rid in range(2,len(run['events'])+1):
        m=temporal_metrics(run,rid,candidate['position'],config)
        conflict=m['conflict'] is not None
        positive=conflict and m['reveal_lead_s']>0
        current=m['robot_current_clearance_at_reveal_m']>0
        switch=conflict and m['switch_to_conflict_margin_s']>0
        rows.append({'old_request_id':rid-1,'fresh_request_id':rid,
                     'conflict_exists':conflict,'positive_lead':positive,'current_clear':current,'switch_before_conflict':switch,
                     'temporal_pass':bool(conflict and positive and current and switch),
                     'conflict_time_s':m['conflict']['time'] if conflict else None,
                     'conflict_point_world':m['conflict']['point'] if conflict else None,
                     'old_min_clearance_m':m['old_min_clearance_m'],
                     'reveal_sim_time':m['reveal_sim_time'],'baseline_application_sim_time':m['baseline_application_sim_time'],
                     'reveal_lead_s':m['reveal_lead_s'],'switch_to_conflict_margin_s':m['switch_to_conflict_margin_s'],
                     'current_robot_clearance_m':m['robot_current_clearance_at_reveal_m']})
    return rows


def prefilter_result(candidate, pairs, rays, groups, rules):
    g=candidate['geometry'];physical=g['floor_ok'] and not g['overlap_paths']
    bypass=candidate['bypass']['pass'];reasons=[]
    if not physical:reasons.append('physical_floor_or_overlap')
    if physical and not bypass:reasons.append('no_local_bypass')
    conflicting=[p for p in pairs if p['conflict_exists']]
    lead=[p for p in conflicting if p['positive_lead']]
    current=[p for p in lead if p['current_clear']]
    timed=[p for p in current if p['switch_before_conflict']]
    if not conflicting:reasons.append('no_remaining_conflict_at_any_pair')
    elif not lead:reasons.append('no_positive_lead')
    elif not current:reasons.append('current_proxy_conflicts_at_all_conflicting_pairs')
    elif not timed:reasons.append('no_post_application_conflict_window')
    surviving=[]
    for pair in pairs:
        pair['ray_transition']=None
        if pair['temporal_pass'] and rays is not None:
            pair['ray_transition']=ray_transition(rays,pair['fresh_request_id'],groups,rules['minimum_prior_observations'])
            if pair['ray_transition']['pass']:surviving.append(pair['fresh_request_id'])
    if physical and bypass and timed and not surviving:
        reasons.append('no_multi_ray_transition_at_temporally_valid_pair')
    return {'pass':bool(physical and bypass and surviving),'physical_pass':bool(physical),'bypass_pass':bool(bypass),
            'conflicting_pair_count':len(conflicting),'positive_lead_pair_count':len(lead),
            'current_clear_pair_count':len(current),'temporal_pass_pair_count':len(timed),
            'ray_evaluated':rays is not None,'surviving_fresh_request_ids':surviving,'rejection_reasons':reasons,
            'pair_metrics':pairs,'ray_observations':rays,
            'note':'Possible pairs are never substituted for the first CLEAR event in actual semantic evidence.'}
