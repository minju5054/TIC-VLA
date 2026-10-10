"""Additive scene lighting and observational clearance logging; unchanged control."""
import hashlib,json
import numpy as np
from research.nova_simulation import NovaSimulation
from research.hospital_lights import apply_hospital_lights
from research.nova_bright import image_stats,bright_pass
from research.records import write_json


class HospitalLightsSimulation(NovaSimulation):
    def __init__(self,cfg,records,actor_factory=None):
        super().__init__(cfg,records,actor_factory=actor_factory)
        from research.audit_hospital_lights import inspect_stage
        before=inspect_stage(self.stage)[-1]
        applied=apply_hospital_lights(self.stage,cfg['hospital_lighting'])
        after=inspect_stage(self.stage)[-1]
        if before!=after:raise RuntimeError('Lighting altered Hospital collision geometry')
        import omni.client
        result,_,data=omni.client.read_file(cfg['scene']['usd'])
        if result!=omni.client.Result.OK:raise RuntimeError('Hospital source hash unavailable')
        identity=hashlib.sha256(bytes(data)).hexdigest()
        if identity!=cfg['hospital_lighting_source_sha256']:raise RuntimeError('Official Hospital source changed')
        write_json(records.path/'hospital_lighting_applied.json',{**applied,'hospital_source_sha256':identity,
            'collision_inventory_unchanged':True,'collision_prim_count':len(before),
            'collision_inventory_sha256':hashlib.sha256(json.dumps(before,sort_keys=True).encode()).hexdigest()})
        for folder in ['brightness','wall_clearance','visibility']:(records.path/'raw'/folder).mkdir()
        self.segmentation=None
        if self.pedestrian:
            import omni.replicator.core as rep
            self.segmentation=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
            self.segmentation.attach([self.product])

    def capture(self,name):
        data,obs,path=super().capture(name)
        s=image_stats(data);write_json(self.records.path/'raw/brightness'/(name+'.json'),{**s,
            'pass':bright_pass(s,self.cfg['bright_validation']),'observation_sim_time':obs['sim_time'],'rgb_reference':str(path.relative_to(self.records.path))})
        import carb
        from omni.physx import get_physx_scene_query_interface
        samples=[];query=get_physx_scene_query_interface()
        for height in [.15,.35,.6]:
            origin=[*obs['position'][:2],height]
            for angle in np.arange(0,2*np.pi,2*np.pi/32):
                hits=[]
                def hit(h):
                    path=str(h.collision)
                    if path.startswith('/Root/') and any(w in path.lower() for w in ['wall','doorframe','trim']):
                        hits.append({'prim':path,'distance_m':float(h.distance),'position':list(map(float,h.position))})
                    return True
                query.raycast_all(carb.Float3(*origin),carb.Float3(float(np.cos(angle)),float(np.sin(angle)),0.),4.,hit,True)
                if hits:samples.append({'height_m':height,'world_angle_rad':float(angle),**min(hits,key=lambda h:h['distance_m'])})
        closest=min(samples,key=lambda r:r['distance_m']) if samples else None
        write_json(self.records.path/'raw/wall_clearance'/(name+'.json'),{'observation':obs,'target_vw':self.command,'rays':samples,'closest_ray_hit':closest,
            'radius_m':self.cfg['spatial_turn_gate']['robot_conservative_radius_m'],
            'min_ray_distance_minus_robot_radius_m':closest['distance_m']-self.cfg['spatial_turn_gate']['robot_conservative_radius_m'] if closest else None,
            'note':'96 horizontal PhysX rays at three robot heights, excluding robot; structural wall/doorframe/trim only. Sampled disk clearance proxy, not exact signed distance or penetration.'})
        if self.segmentation is not None:
            from PIL import Image
            from research.nova_occlusion import segmentation_stats,visibility_state
            packed=self.segmentation.get_data()
            if 'stationary_cart' in self.cfg:
                from research.hospital_cart import mask_stats
                stats,mask=mask_stats(packed,self.cfg['stationary_cart']['visibility_rules']);state=stats['state']
            else:
                stats,mask=segmentation_stats(packed['data'],packed['info']['idToLabels']);state=visibility_state(stats,self.cfg['occlusion']['visibility_rule'])
            write_json(self.records.path/'raw/visibility'/(name+'.json'),{**stats,'state':state,
                'observation_sim_time':obs['sim_time'],'model_input':False})
            Image.fromarray((mask*255).astype('uint8')).save(self.records.path/'raw/visibility'/(name+'.mask.png'))
        return data,obs,path
