"""Pure route construction, deterministic selection and observation gates."""
import numpy as np


def reference_route(episode,lane,rules,step=None):
    radius=rules['reference_turn_radius_m'];speed=rules['reference_speed_m_s']
    dt=step if step is not None else rules['reference_dt_s'];start=np.array(episode['start'][:2])
    if 'reference_turn_y' in episode:
        radius=lane;line=episode['reference_turn_y']-start[1]-radius;arc=radius*np.pi/2
        length=line+arc+episode['reference_end_x']-start[0]-radius
        poses=[]
        for s in np.r_[np.arange(0,length,dt*speed),length]:
            if s<=line:x,y,yaw=start[0],start[1]+s,np.pi/2
            elif s<=line+arc:
                angle=(s-line)/radius
                x,y,yaw=start[0]+radius-radius*np.cos(angle),start[1]+line+radius*np.sin(angle),np.pi/2-angle
            else:x,y,yaw=start[0]+radius+s-line-arc,episode['reference_turn_y'],0.
            poses.append([float(x),float(y),float(yaw)])
        return np.array(poses)
    line=start[0]-lane-radius;arc=radius*np.pi/2
    length=line+arc+rules['reference_end_y_m']-(start[1]+radius)
    distance=np.r_[np.arange(0,length,dt*speed),length];poses=[]
    for s in distance:
        if s<=line:x,y,yaw=start[0]-s,start[1],np.pi
        elif s<=line+arc:
            angle=(s-line)/radius
            x,y,yaw=lane+radius-radius*np.sin(angle),start[1]+radius-radius*np.cos(angle),np.pi-angle
        else:x,y,yaw=lane,start[1]+radius+s-line-arc,np.pi/2
        poses.append([float(x),float(y),float(yaw)])
    return np.array(poses)


def select_route(candidates):
    valid=[c for c in candidates if c['geometry_pass'] and c['transition'] is not None]
    return min(valid,key=lambda c:(-c['transition']['new_samples'],abs(c['transition']['distance_m']-2.),c['lane_x'],*c['human_position'])) if valid else None


def baseline_crossing(events,gate):
    for i,e in enumerate(events):
        x,y,yaw=e['agent_pose_at_observation']
        if gate.get('exit_x_min_m',-np.inf)<=x<=gate['exit_x_max_m'] and gate['exit_y_min_m']<=y<=gate['exit_y_max_m'] and gate['yaw_min_rad']<=yaw<=gate['yaw_max_rad']:
            return i,i+1 if i+1<len(events) else None
    return None,None


def plotted_world(chunk,observation):
    from research.analyze_chunk_geometry import project_world
    return project_world(chunk,observation['pose'])


def custom_source():
    import json
    from pathlib import Path
    from research.hospital_episode import digest
    path=Path(__file__).resolve().parents[1]/'configs/research/blind_corner_protocol.json'
    return {'episode_id':'blind_door27','source_kind':'custom geometry-only Hospital route; not official benchmark episode',
            'route_source':str(path),'route_source_sha256':digest(path),'source_convention':'USD RotateZ degrees',
            'resolved_yaw_radians':np.pi/2,'episode':json.loads(path.read_text())['custom_route']}
