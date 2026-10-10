"""Fixed world-space turn exit and descriptive wall-bound clearance proxies."""
import numpy as np


def spatial_gate(events,gate):
    """First observation beyond a fixed corner cross-section, then one more."""
    crossing=None
    for i,e in enumerate(events):
        x,y,_=e['agent_pose_at_observation']
        if x<=gate['exit_center_x_max_m'] and gate['center_y_min_m']<=y<=gate['center_y_max_m']:
            crossing=i;break
    complete=crossing is not None and crossing+1<len(events)
    end=crossing+1 if complete else len(events)-1
    return {'crossing_request_id':events[crossing]['request_id'] if crossing is not None else None,
        'through_request_id':events[end]['request_id'] if events else None,'crossing_plus_one_observation':complete,
        'required_request_ids':[e['request_id'] for e in events[:end+1]],
        'rule':'First observation west of fixed corner cross-section within footprint-clear hallway band, plus one subsequent observation'}


def wall_inventory(collisions):
    # All enabled Hospital structural wall/doorframe/trim bounds intersecting
    # robot height, independent of trajectory or model outcomes.
    return [c for c in collisions if c['enabled'] and c['world_aabb_min'] is not None
        and c['world_aabb_min'][2]<.65 and c['world_aabb_max'][2]>.15
        and any(w in c['path'].lower() for w in ['wall','doorframe','trim'])]


def bound_clearance(xy,walls,radius):
    lo=np.array([w['world_aabb_min'][:2] for w in walls]);hi=np.array([w['world_aabb_max'][:2] for w in walls])
    p=np.asarray(xy);outside=np.maximum(np.maximum(lo-p,p-hi),0)
    d=np.linalg.norm(outside,axis=1);inside=(outside==0).all(1)
    d[inside]=-np.minimum(p-lo,hi-p)[inside].min(1)
    i=int(np.argmin(d))
    return {'wall_bound_clearance_m':float(d[i]-radius),'wall_prim':walls[i]['path'],
        'proxy_note':'Conservative circumscribed robot XY disk vs structural collider world AABB; not exact distance/penetration or contact oracle'}


def first_contact(spans):
    return min(spans,key=lambda s:s['start_sim_time']) if spans else None


def require_human_prerequisites(receipt):
    if receipt.get('baseline_spatial_gate_pass') is not True or receipt.get('occlusion_preflight_pass') is not True:
        raise ValueError('STOP BEFORE HUMAN: baseline spatial gate and actual occlusion preflight must pass')
