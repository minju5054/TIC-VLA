"""Measured/predicted cart passage in the frozen baseline travel frame."""
import numpy as np
from research.cart_geometry import rotation,segment_minimum


def travel_coordinates(xy,cart):
    return (np.asarray(xy,float)-cart['center_world_xyz'][:2])@rotation(cart['tangent_rad'])


def forward_crossing(xy,times,cart,plane=0.):
    p=np.asarray(xy,float);times=np.asarray(times,float);local=travel_coordinates(p,cart)
    for i in range(len(p)-1):
        if local[i,0]<=plane<local[i+1,0]:
            u=(plane-local[i,0])/(local[i+1,0]-local[i,0])
            return {'segment':i,'fraction':float(u),'sim_time':float(times[i]+u*(times[i+1]-times[i])),
                'world_xy':(p[i]+u*(p[i+1]-p[i])).tolist(),'left_m':float(local[i,1]+u*(local[i+1,1]-local[i,1]))}
    return None


def predicted_side(xy,times,cart,radius):
    crossing=forward_crossing(xy,times,cart);limit=min(cart['footprint_half_m'])+radius
    if crossing is None:return {'side':'UNKNOWN / DOES NOT REACH','crossing':None}
    l=crossing['left_m']
    side='SOUTH/LEFT BYPASS' if l>limit+1e-9 else 'NORTH/RIGHT BYPASS' if l<-limit-1e-9 else 'CENTER / NO CLEAR BYPASS'
    return {'side':side,'crossing':crossing}


def closest_approach(xy,times,cart,radius):
    xy=np.asarray(xy);local=(xy-cart['center_world_xyz'][:2])@rotation(cart['yaw_rad']);best=None
    for i,(a,b) in enumerate(zip(local[:-1],local[1:])):
        d,u=segment_minimum(a,b,cart['footprint_half_m'])
        if best is None or d-radius<best['clearance_m']:
            best={'clearance_m':d-radius,'segment':i,'fraction':u,'sim_time':float(times[i]+u*(times[i+1]-times[i])),
                'world_xy':(xy[i]+u*(xy[i+1]-xy[i])).tolist()}
    return best


def outcome(xy,times,cart,radius,contacts,max_v):
    """Side at centre plane; contact ordering and complete passage are separate."""
    xy=np.asarray(xy);times=np.asarray(times);extent=max(cart['footprint_half_m'])+radius
    s=travel_coordinates(xy,cart)[:,0];entry=np.flatnonzero(s>=-(extent+3.*max_v))
    approach=float(times[entry[0]]) if len(entry) else None
    side=predicted_side(xy,times,cart,radius);crossing=side['crossing'];clear=forward_crossing(xy,times,cart,extent)
    if clear and (not crossing or clear['sim_time']<crossing['sim_time']):clear=None
    first=min(contacts,key=lambda c:c['start_sim_time']) if contacts else None
    contact_before=first is not None and (crossing is None or first['start_sim_time']<crossing['sim_time'])
    actual='CONTACT BEFORE BYPASS' if contact_before else side['side']
    if actual=='UNKNOWN / DOES NOT REACH':actual='CENTER / NO CLEAR BYPASS'
    distance=None;progress=None
    if clear:
        i,u=clear['segment'],clear['fraction']
        distance=float((1-u)*np.linalg.norm(xy[i+1]-xy[i])+np.linalg.norm(np.diff(xy[i+1:],axis=0),axis=1).sum())
        progress=float(s[-1]-extent)
    return {'actual_bypass_side':actual,'geometric_passage_side':side['side'],'centre_crossing':crossing,'clear_region':clear,
        'interaction_start_s':approach,'interaction_end_s':clear['sim_time'] if clear else float(times[-1]),
        'clears_before_any_contact':bool(clear and (first is None or clear['sim_time']<first['start_sim_time'])),
        'distance_travelled_after_clearing_m':distance,'signed_forward_progress_after_clearing_m':progress,
        'closest_approach':closest_approach(xy,times,cart,radius)}


def plot_coordinates(runs):
    """Export exactly measured paths and observation markers, with no display jitter."""
    result={}
    for name,run in runs.items():
        obs=[{'request_id':e['request_id'],'sim_time':e['observation']['sim_time'],'pose':e['agent_pose_at_observation']} for e in run['events']]
        for row,e in zip(obs,run['events']):
            r=run['robot'][e['observation']['tick']]
            if row['pose']!=[float(r[k]) for k in ['x','y','yaw']]:raise ValueError('Observation/tick mismatch')
        result[name]={'world_xy':run['xy'].tolist(),'sim_times':run['times'].tolist(),'observations':obs}
    return result
