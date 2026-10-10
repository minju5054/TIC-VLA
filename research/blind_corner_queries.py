"""Static Hospital queries shared by route selection and local human preflight."""
import json
from pathlib import Path
import numpy as np
from research.reveal_prefilter import human_samples


class Queries:
    def __init__(self,stage):
        import carb
        from pxr import UsdPhysics
        from omni.physx import get_physx_interface,get_physx_scene_query_interface
        self.carb=carb
        if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):UsdPhysics.Scene.Define(stage,'/World/StaticQueryScene')
        get_physx_interface().force_load_physics_from_usd();self.q=get_physx_scene_query_interface()
        self.floor_prim=self.floor([19.,0.])[1]
        self.rules=json.loads(Path('configs/research/reveal_window_prefilter.json').read_text())
        asset=json.loads(Path('outputs/nova-hospital-occlusion-geometry-20261010-03/robot_asset.json').read_text())
        self.extrinsic=np.asarray(asset['camera_body_transform'])
        camera=next(c for c in asset['cameras'] if 'front_hawk/left/camera_left' in c['path'])
        self.slopes=np.array([camera['horizontal_aperture'],camera['vertical_aperture']])/(2*camera['focal_length'])

    def floor(self,xy,door_sill=False):
        hit=self.q.raycast_closest(self.carb.Float3(*map(float,xy[:2]),.3),self.carb.Float3(0,0,-1),.6)
        path=str(hit.get('collision',''))
        tolerance=.03 if door_sill and 'DoorFloor' in path else .02
        return bool(hit['hit'] and abs(hit['position'][2])<tolerance and hit['normal'][2]>.9),path

    def overlap(self,xyz,radius=None,extent=None,yaw=0.):
        hits=[]
        def callback(h):
            path=str(h.collision)
            if path.startswith('/Root/') and path!=self.floor_prim:hits.append(path)
            return True
        if extent is None:self.q.overlap_sphere(float(radius),self.carb.Float3(*map(float,xyz)),callback,False)
        else:self.q.overlap_box(self.carb.Float3(*map(float,extent)),self.carb.Float3(*map(float,xyz)),self.carb.Float4(0,0,float(np.sin(yaw/2)),float(np.cos(yaw/2))),callback,False)
        return sorted(set(hits))

    def robot_path(self,xy,radius):
        return [{'xy':p.tolist(),'floor':self.floor(p)[0],'overlap':self.overlap([*p,.70],radius)} for p in np.asarray(xy)]

    def oriented_robot_path(self,poses):
        footprint=json.loads(Path('outputs/nova-human-preflight-20261010-02/nova_footprint.json').read_text())
        lo=np.min([s['xy_min'] for s in footprint['shapes']],axis=0)
        hi=np.max([s['xy_max'] for s in footprint['shapes']],axis=0)
        center=(lo+hi)/2;half=(hi-lo)/2+.03;rows=[]
        for x,y,yaw in poses:
            c,s=np.cos(yaw),np.sin(yaw);xy=np.array([x,y])+np.array([[c,-s],[s,c]])@center
            rows.append({'xy':[float(x),float(y)],'yaw':float(yaw),'floor':self.floor([x,y],door_sill=True)[0],
                         'overlap':self.overlap([*xy,.65],extent=[*half,.60],yaw=yaw)})
        return rows

    def human(self,position,radius=.25,yaw=0.):
        ok,floor=self.floor(position);hits=set()
        for z in [.3,.65,1.,1.35,1.6]:hits.update(self.overlap([*position[:2],z],radius))
        asset=json.loads(Path('outputs/nova-reveal-window-search-20261010-04/human_asset.json').read_text())
        lo,hi=np.array(asset['source_bounds'])*asset['source_units'];half=(hi-lo)/2
        hits.update(self.overlap([*position[:2],half[2]+.01],extent=[half[0],half[1],half[2]-.01],yaw=yaw))
        return {'floor_ok':ok,'floor_prim':floor,'overlap_paths':sorted(hits)}

    def bypass(self,position,radius,heading=np.pi/2):
        forward=np.array([np.cos(heading),np.sin(heading)]);normal=np.array([-forward[1],forward[0]])
        for offset in [-1.,1.,-1.25,1.25,-1.5,1.5]:
            xy=np.array(position[:2])+offset*normal+np.arange(-1.,1.001,.1)[:,None]*forward
            trace=self.robot_path(xy,radius+.05)
            if all(r['floor'] and not r['overlap'] for r in trace):
                return {'pass':True,'centerline_xy':xy.tolist(),'clearance_m':abs(offset)-radius-.25,'offset_m':offset}
        return {'pass':False,'centerline_xy':[],'clearance_m':None}

    def rays(self,position,poses,z=0.,yaw=0.):
        samples=human_samples(position,yaw,self.rules);xyz=np.array([s['world'] for s in samples]);rows=[]
        for rid,pose in enumerate(poses,1):
            x,y,yaw=pose;c,s=np.cos(yaw),np.sin(yaw)
            body=np.eye(4);body[:3,:3]=[[c,s,0],[-s,c,0],[0,0,1]];body[3,:3]=[x,y,z]
            matrix=self.extrinsic@body;origin=matrix[3,:3]
            local=np.column_stack([xyz,np.ones(len(xyz))])@np.linalg.inv(matrix);depth=-local[:,2]
            envelope=(depth>0)&(np.abs(local[:,:2])<=2*self.slopes*depth[:,None]).all(1)
            hits=[]
            for point in xyz:
                delta=point-origin;length=np.linalg.norm(delta);found=[]
                def cb(hit):
                    if str(hit.collision).startswith('/Root/') and hit.distance<length-1e-6:found.append({'prim':str(hit.collision),'distance_m':float(hit.distance)})
                    return True
                self.q.raycast_all(self.carb.Float3(*map(float,origin)),self.carb.Float3(*map(float,delta/length)),float(length),cb,True)
                hits.append(min(found,key=lambda h:h['distance_m']) if found else None)
            rows.append({'request_id':rid,'exposed':[bool(e and h is None) for e,h in zip(envelope,hits)],'hits':hits})
        return rows,[s['group'] for s in samples]
