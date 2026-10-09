"""Pure stationary-human visibility event rules, geometry and freeze contracts."""
import numpy as np
from research.nova_bright import rgb_difference

VISIBILITY_RULES={'hidden_max_pixels':0,'clearly_visible_min_fraction':.002,
                  'clearly_visible_min_bbox_height':80,'clearly_visible_min_bbox_width':20,
                  'minimum_hidden_preflight_observations':3,
                  'leakage_max_mean_abs_0_255':1.,'leakage_max_fraction_gt10':.01}


def segmentation_stats(data,labels,label='occluded_human'):
    a=np.asarray(data)
    if a.ndim!=2:raise ValueError('Uncolorized 2D segmentation IDs required')
    ids=[int(k) for k,v in labels.items() if label in str(v)]
    mask=np.isin(a,ids);ys,xs=np.where(mask)
    bbox=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)] if len(xs) else None
    return {'human_visible_pixel_count':int(mask.sum()),'human_visible_fraction':float(mask.mean()),
            'human_bbox_if_visible':bbox,'image_shape':list(a.shape),'semantic_ids':ids},mask


def visibility_state(stats,rules=VISIBILITY_RULES):
    count=stats['human_visible_pixel_count'];box=stats['human_bbox_if_visible']
    if count<=rules['hidden_max_pixels']:return 'HIDDEN'
    if (stats['human_visible_fraction']>=rules['clearly_visible_min_fraction'] and box
            and box[3]-box[1]>=rules['clearly_visible_min_bbox_height']
            and box[2]-box[0]>=rules['clearly_visible_min_bbox_width']):return 'VISIBLE'
    return 'MARGINAL'


def first_reveal(rows,rules=VISIBILITY_RULES):
    states=[visibility_state(r,rules) for r in rows]
    first=next((i for i,s in enumerate(states) if s=='VISIBLE'),None)
    if first is None:return {'old_request_id':None,'fresh_request_id':None,'valid_transition':False,'reason':'Human never clearly visible'}
    # Never skip the first sensor event for a favorable later pair.
    fresh=rows[first]['request_id'];old=rows[first-1]['request_id'] if first else None
    hidden=first>0 and states[first-1]=='HIDDEN'
    return {'old_request_id':old,'fresh_request_id':fresh,'valid_transition':hidden,
            'reason':'HIDDEN to first CLEARLY_VISIBLE' if hidden else 'First clear event has no immediately hidden OLD',
            'states':states}


def leakage_pass(absent,present,rules=VISIBILITY_RULES):
    difference=rgb_difference(absent,present)
    return {**difference,'pass':difference['mean_abs_0_255']<=rules['leakage_max_mean_abs_0_255']
            and difference['fraction_any_channel_gt10']<=rules['leakage_max_fraction_gt10']}


def stationary_velocity(previous,current,dt):
    if dt<=0:raise ValueError('Positive tick interval required')
    return (np.asarray(current,float)-np.asarray(previous,float))/dt


def assert_stationary(positions,tolerance=1e-6):
    p=np.asarray(positions,float)
    if not np.isfinite(p).all() or np.max(np.linalg.norm(p-p[0],axis=1))>tolerance:raise ValueError('Stationary human moved')
    return True


def candidate_positions(events,robot_xy):
    """Grid around measured turn-exit corridor, independent of native waypoints."""
    poses=np.array([e['agent_pose_at_observation'] for e in events]);yaw=np.unwrap(poses[:,2])-poses[0,2]
    # Turn region spans measured left yaw from 20 to 100 degrees. Unique .25m
    # grid positions around this entire measured corridor, ordered by XYZ.
    selected=poses[(yaw>=np.deg2rad(20))&(yaw<=np.deg2rad(100))]
    points=set()
    for x,y,a in selected:
        normal=np.array([-np.sin(a),np.cos(a)])
        for offset in [-.75,-.5,-.25,0.,.25,.5,.75]:
            p=np.round((np.array([x,y])+offset*normal)/.25)*.25;points.add(tuple(p))
    return [[float(x),float(y),0.] for x,y in sorted(points)]
