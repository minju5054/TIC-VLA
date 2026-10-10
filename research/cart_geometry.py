"""Oriented cart footprint proxies; native outputs remain forward/left points."""
import numpy as np


def rotation(yaw):
    c,s=np.cos(yaw),np.sin(yaw)
    return np.array([[c,-s],[s,c]])


def freeze_pose(events,bounds,floor_z):
    if [e['request_id'] for e in events[17:20]]!=[18,19,20]:raise ValueError('C18–C20 required')
    lo,hi=np.asarray(bounds,float);delta=np.asarray(events[19]['agent_pose_at_observation'][:2])-events[17]['agent_pose_at_observation'][:2]
    if np.linalg.norm(delta)<1e-9:raise ValueError('Undefined measured tangent')
    tangent=float(np.arctan2(delta[1],delta[0]));axis=int(np.argmax((hi-lo)[:2]))
    yaw=float((tangent-axis*np.pi/2+np.pi)%(2*np.pi)-np.pi)
    center=[*events[18]['agent_pose_at_observation'][:2],float(floor_z)]
    pivot=np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
    matrix=np.eye(4);matrix[:2,:2]=rotation(yaw).T;matrix[3,:3]=np.asarray(center)-pivot@matrix[:3,:3]
    return {'center_world_xyz':center,'yaw_rad':yaw,'tangent_rad':tangent,'long_axis_local':axis,
            'footprint_half_m':((hi-lo)[:2]/2).tolist(),'height_m':float(hi[2]-lo[2]),
            'matrix_row_vector':matrix.tolist(),'local_bounds':np.asarray(bounds).tolist()}


def clearance(points,cart,radius):
    p=(np.asarray(points,float)[...,:2]-cart['center_world_xyz'][:2])@rotation(cart['yaw_rad'])
    q=np.abs(p)-cart['footprint_half_m']
    return np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(np.max(q,axis=-1),0)-radius


def segment_minimum(a,b,half):
    """Exact minimum rectangle signed distance on a linear segment."""
    a,b,h=np.asarray(a),np.asarray(b),np.asarray(half);d=b-a;u=[0.,1.]
    for j in range(2):
        if abs(d[j])>1e-15:u.extend((v-a[j])/d[j] for v in [-h[j],0,h[j]])
    for sx in [-1,1]:
        for sy in [-1,1]:
            den=sx*d[0]-sy*d[1]
            if abs(den)>1e-15:u.append((h[0]-h[1]-sx*a[0]+sy*a[1])/den)
            if d@d>1e-24:u.append((np.array([sx,sy])*h-a)@d/(d@d))
    u=np.clip(u,0,1);q=np.abs(a+u[:,None]*d)-h
    dist=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(1),0);i=np.argmin(dist)
    return float(dist[i]),float(u[i])


def path_clearance(points,cart,radius,times=None):
    p=np.asarray(points,float)
    if len(p)==0:return {'minimum_clearance_m':None,'first_conflict':None}
    local=(p[:,:2]-cart['center_world_xyz'][:2])@rotation(cart['yaw_rad']);half=cart['footprint_half_m']
    best=float(np.min(clearance(p,cart,radius)));first=None
    for i,(a,b) in enumerate(zip(local[:-1],local[1:])):
        value,u=segment_minimum(a,b,half);best=min(best,value-radius)
        if first is None and value<=radius:
            low,high=0.,u
            def dist(v):
                q=np.abs(a+v*(b-a))-half
                return np.linalg.norm(np.maximum(q,0))+min(max(q),0)-radius
            if dist(0)>0:
                for _ in range(55):
                    mid=(low+high)/2
                    if dist(mid)>0:low=mid
                    else:high=mid
            else:high=0.
            first={'segment':i,'fraction':float(high),'point':(p[i]+high*(p[i+1]-p[i])).tolist(),
                   'sim_time':float(times[i]+high*(times[i+1]-times[i])) if times is not None else None}
    return {'minimum_clearance_m':best,'first_conflict':first}


def outline(cart,radius=0.):
    h=np.asarray(cart['footprint_half_m']);points=[]
    for sx,sy,start in [(1,1,0),(-1,1,90),(-1,-1,180),(1,-1,270)]:
        a=np.deg2rad(np.linspace(start,start+90,17));points.extend(np.array([sx,sy])*h+radius*np.column_stack([np.cos(a),np.sin(a)]))
    p=np.asarray(points)@rotation(cart['yaw_rad']).T+cart['center_world_xyz'][:2]
    return np.vstack([p,p[0]])
