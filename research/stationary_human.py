"""Physically present fixed human. No trigger, motion command or pose updates."""
import csv
import numpy as np
from research.human_actor import author_human
from research.records import write_json,clocks
from research.nova_occlusion import stationary_velocity


class StationaryHuman:
    def __init__(self,sim,cfg):
        self.sim,self.cfg=sim,cfg
        _,asset=author_human(sim.stage,{**cfg,'start_position':cfg['position']})
        from isaacsim.core.utils.semantics import add_labels
        add_labels(sim.stage.GetPrimAtPath(cfg['prim']),['occluded_human'])
        write_json(sim.records.path/'human_asset.json',asset)
        self.file=(sim.records.path/'raw/human_state.csv').open('x')
        self.writer=csv.DictWriter(self.file,fieldnames=['tick','sim_time','x','y','z','yaw','vx','vy','vz','motion_active'])
        self.writer.writeheader();self.previous=None;self.initial=None;self.velocity=np.zeros(3)

    def update(self,sim_time):
        # No authoring after initial scene creation. Kinematic body remains fixed.
        pass

    def snapshot(self):
        import omni.physx
        pose=omni.physx.get_physx_interface().get_rigidbody_transformation(self.cfg['prim'])
        if not pose['ret_val']:raise RuntimeError('Human PhysX pose missing')
        position=np.array(pose['position']);t=self.sim.state()['sim_time']
        if self.initial is None:self.initial=position.copy()
        if np.linalg.norm(position-self.initial)>1e-6:raise RuntimeError('Stationary human moved')
        return {'position':position.tolist(),'yaw':self.cfg['yaw'],'sim_time':t,'motion_active':False,
            'velocity_world':self.velocity.tolist(),'velocity_note':'Finite difference of consecutive measured PhysX tick positions; configured yaw fixed',
            'event_source':'main_thread_measured_PhysX',**clocks()}

    def measure(self,sim_time):
        s=self.snapshot()
        if self.previous:self.velocity=stationary_velocity(self.previous['position'],s['position'],sim_time-self.previous['sim_time'])
        self.writer.writerow(dict(zip(self.writer.fieldnames,[self.sim.tick,sim_time,*s['position'],s['yaw'],*self.velocity,False])))
        self.previous=s

    def close(self):self.file.close()
