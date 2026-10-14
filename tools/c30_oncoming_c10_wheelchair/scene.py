"""Prespecified oncoming kinematic human; no gait, crowd policy, or robot feedback."""
import copy
import csv
import hashlib
import math
import numpy as np
from research.records import write_json, clocks
from research.hospital_lights_simulation import HospitalLightsSimulation
from tools.exact_two_humans.scene import mesh_world_points, mesh_bounds, query_scene, grounding_views
from tools.two_wheelchairs_v08.scene import (
    scenario_config as static_config, WheelchairActor, wheelchair_gate, INSTRUCTION, read)

TIMING_BASE = 'outputs/two-wheelchairs-v08-20261014T015420Z-a56068'


def sole_minima(boots,matrix):
    local=(np.column_stack([boots,np.ones(len(boots))])@np.linalg.inv(matrix))[:,:3]
    split=(local[:,0].min()+local[:,0].max())/2
    return [float(boots[local[:,0]<split,2].min()),float(boots[local[:,0]>=split,2].min())]


def rotated_human_bounds(stage,prim):
    """Split the two boots in human-root X, then measure sole Z in world metres."""
    from pxr import UsdGeom
    meshes=mesh_world_points(stage,prim);bounds=mesh_bounds(meshes)
    boots=next(r['world_points'] for r in meshes if r['path'].endswith('/opaque__leather__workboots'))
    matrix=np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(prim)).ComputeLocalToWorldTransform(0))
    bounds['sole_minima_world_z']=sole_minima(boots,matrix)
    bounds['sole_partition_frame']='human-root local X, not world X; minima measured in world Z'
    return bounds


def motion_state(h, time):
    points = np.asarray(h['path_world_xyz'], float)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    distance = float(np.clip((time-h['start_sim_s'])*h['speed_m_s'], 0., cumulative[-1]))
    index = min(int(np.searchsorted(cumulative, distance, side='right')-1), len(lengths)-1)
    fraction = (distance-cumulative[index])/lengths[index]
    delta = points[index+1]-points[index]
    position = points[index]+fraction*delta
    heading = float(np.arctan2(delta[1], delta[0]))
    # Existing character faces local -Y, visually checked in the C30 side view.
    yaw = heading+math.pi/2
    return position, yaw, bool(h['start_sim_s'] <= time < h['start_sim_s']+cumulative[-1]/h['speed_m_s'])


def scenario_config(root, cfg, reference):
    human = copy.deepcopy(cfg['stationary_human'])
    result = static_config(root, cfg, reference)
    result['fixed_wheelchairs'] = [w for w in result['fixed_wheelchairs'] if w['id']=='C10']
    prior = read(root/TIMING_BASE/'derived/two-wheelchairs-v08/summary.json')
    if prior['status']!='PASS_THIS_SCENARIO' or prior['nonfloor_contact_spans_through_goal_or_end']!=0:
        raise ValueError('A successful static route is required before this dynamic trial')
    points = []
    for rid in range(30, 11, -1):
        event = read(root/reference/f'raw/requests/request_{rid:06d}.json')
        points.append([*event['observation']['position'][:2], 0.])
    target = read(root/TIMING_BASE/'raw/requests/request_000030.json')['observation']
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    nearest = int(np.argmin(np.linalg.norm(np.asarray(points)[:,:2]-target['position'][:2], axis=1)))
    start = float(target['sim_time']-distances[nearest])
    if start < 0: raise ValueError('Invalid prespecified encounter schedule')
    human.update(id='C30', prim='/World/OncomingHumanC30', position=points[0], start_position=points[0],
        path_world_xyz=points, reference_run=str(reference), reference_request_ids=list(range(30,11,-1)),
        speed_m_s=1., start_sim_s=start, label='occluded_human',
        motion='Kinematic translation along reversed original C30-C12 path; no gait or human avoidance policy',
        timing_reference=TIMING_BASE, timing_reference_observation=target,
        planned_encounter_world_xyz=points[nearest], planned_encounter_sim_s=target['sim_time'],
        root_forward_local=[0.,-1.,0.], reference_tangent_rad=float(np.arctan2(points[1][1]-points[0][1], points[1][0]-points[0][0])))
    human['yaw'] = motion_state(human, 0.)[1]
    result['oncoming_human'] = human
    result['encounter_check'] = {'corner_world_xy_min':[1.,8.], 'corner_world_xy_max':[6.5,12.5],
        'maximum_robot_human_center_distance_m':2., 'requires_measured_human_motion':True,
        'note':'Prespecified operational corner encounter check, not a general avoidance metric.'}
    return result


class MixedActor:
    def __init__(self, sim, cfg):
        from pxr import Gf, UsdGeom
        from research.human_actor import author_human
        from isaacsim.core.utils.semantics import add_labels
        self.sim, self.cfg, self.h = sim, cfg, cfg['oncoming_human']
        self.wheel = WheelchairActor(sim, cfg)
        self.wheels = self.wheel.wheels
        self.translate, info = author_human(sim.stage, self.h)
        self.rotate = next(op for op in UsdGeom.Xformable(sim.stage.GetPrimAtPath(self.h['prim'])).GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeRotateZ)
        before = rotated_human_bounds(sim.stage, self.h['prim'])
        visual = UsdGeom.Xformable(sim.stage.GetPrimAtPath(self.h['prim']+'/VisualTransform'))
        op = next(op for op in visual.GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        old = np.asarray(op.Get()); new = old.copy(); new[2] -= before['shoe_min_world_z']
        op.Set(Gf.Vec3d(*new))
        capsule = UsdGeom.Capsule(sim.stage.GetPrimAtPath(self.h['prim']+'/Collider'))
        cop = next(op for op in capsule.GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        cop.Set(Gf.Vec3d(0.,0.,self.h['capsule_cylinder_height_m']/2+self.h['radius_m']))
        add_labels(sim.stage.GetPrimAtPath(self.h['prim']), [self.h['label']])
        self.asset = {'config':self.h, 'original_asset_info':info, 'before':before,
            'after':rotated_human_bounds(sim.stage,self.h['prim']),
            'visual_translation_m':new.tolist(), 'capsule_bottom_m':0.,
            'motion_note':'Frozen rest pose translated and oriented along a prespecified path, not animated walking.'}
        world=np.vstack([r['world_points'] for r in mesh_world_points(sim.stage,self.h['prim'])])
        matrix=np.asarray(UsdGeom.Xformable(sim.stage.GetPrimAtPath(self.h['prim'])).ComputeLocalToWorldTransform(0))
        local=(np.column_stack([world,np.ones(len(world))])@np.linalg.inv(matrix))[:,:3]
        self.asset['visible_bounds_in_root_m']=[local.min(0).tolist(),local.max(0).tolist()]
        write_json(sim.records.path/'human_asset.json',self.asset)
        self.file = (sim.records.path/'raw/human_state.csv').open('x')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['tick','sim_time','x','y','z','qx','qy','qz','qw','target_x','target_y','target_z','target_yaw','motion_active','target_error_m'])
        self.current,self.yaw,_ = motion_state(self.h,0.)
        self.max_error=0.

    def update(self,time):
        from pxr import Gf
        self.current,self.yaw,_ = motion_state(self.h,time)
        self.translate.Set(Gf.Vec3d(*self.current))
        self.rotate.Set(math.degrees(self.yaw))

    def snapshot(self):
        import omni.physx
        pose=omni.physx.get_physx_interface().get_rigidbody_transformation(self.h['prim'])
        if not pose['ret_val']:raise RuntimeError('Missing measured human PhysX pose')
        time=self.sim.state()['sim_time']; _,_,active=motion_state(self.h,time)
        human={'position':list(pose['position']),'quaternion_xyzw':list(pose['rotation']),
            'target_position':self.current.tolist(),'target_yaw_rad':self.yaw,'motion_active':active,
            'sim_time':time,'event_source':'main_thread_measured_PhysX',**clocks()}
        return {'humans':{'C30':human},'wheelchairs':self.wheel.snapshot()['wheelchairs'],
            'object_type':'one_oncoming_human_one_static_wheelchair','sim_time':time}

    def measure(self,time):
        human=self.snapshot()['humans']['C30'];error=float(np.linalg.norm(np.asarray(human['position'])-self.current))
        self.max_error=max(self.max_error,error)
        if error>.001:raise RuntimeError('Human PhysX pose differs from target by more than 1 mm')
        self.writer.writerow([self.sim.tick,time,*human['position'],*human['quaternion_xyzw'],*self.current,self.yaw,human['motion_active'],error])
        self.wheel.measure(time)

    def close(self):
        self.file.close();self.wheel.close()


class MixedSimulation(HospitalLightsSimulation):
    def capture(self,name):
        from research.nova_occlusion import segmentation_stats,visibility_state
        from PIL import Image
        rgb,obs,path=super().capture(name);packed=self.segmentation.get_data()
        folder=self.records.path/'raw/object_visibility';folder.mkdir(exist_ok=True)
        np.save(folder/(name+'.ids.npy'),np.asarray(packed['data']),allow_pickle=False)
        objects={}
        labels={'C10_wheelchair':self.cfg['fixed_wheelchairs'][0]['label'],'C30_human':self.cfg['oncoming_human']['label']}
        for key,label in labels.items():
            stats,mask=segmentation_stats(packed['data'],packed['info']['idToLabels'],label)
            objects[key]={**stats,'state':visibility_state(stats,self.cfg['occlusion']['visibility_rule'])}
            Image.fromarray((mask*255).astype('uint8')).save(folder/(name+'.'+key+'.mask.png'))
        write_json(folder/(name+'.json'),{'objects':objects,'idToLabels':packed['info']['idToLabels'],'observation_sim_time':obs['sim_time'],'model_input':False})
        return rgb,obs,path


def pre_model_gate(sim,cfg,receipt,baseline):
    import omni.client
    from pxr import UsdPhysics
    before=sim.state();_,obs,_=sim.capture('preflight.png')
    robot=read(sim.records.path/'robot_asset.json');old=read(baseline/'robot_asset.json')
    light=read(sim.records.path/'hospital_lighting_applied.json');oldlight=read(baseline/'hospital_lighting_applied.json')
    h=cfg['oncoming_human'];human=sim.pedestrian.snapshot()['humans']['C30'];asset=sim.pedestrian.asset
    result,_,data=omni.client.read_file(h['asset'])
    checks={'original_instruction':cfg['instruction']==INSTRUCTION,
        'controller_and_budget':cfg['controller']['max_v']==.8 and cfg['controller']['max_w']==1.2 and cfg['simulation']['predictions']==90,
        'one_C10_wheelchair':[w['id'] for w in cfg['fixed_wheelchairs']]==['C10'],
        'one_C30_human':h['id']=='C30',
        'old_added_actors_absent':all(not sim.stage.GetPrimAtPath(p) for p in ('/World/ExactHumanC10','/World/ExactHumanC24','/World/ExactHumanC30','/World/ExactWheelchairC24','/World/PreflightHuman','/World/E16Cart')),
        'robot_camera_wheels':all(robot[k]==old[k] for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'current_robot_source':robot['source']==receipt['changes']['current_robot_source'],
        'spawn':read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting':{k:v for k,v in light.items() if not k.startswith('collision_')}=={k:v for k,v in oldlight.items() if not k.startswith('collision_')},
        'lighting_preserves_colliders':light['collision_inventory_unchanged'],
        'capture_no_physics':before['tick']==obs['tick']==0 and before['position']==obs['position'],
        'brightness':read(sim.records.path/'raw/brightness/preflight.png.json')['pass'],
        'human_source':result==omni.client.Result.OK and hashlib.sha256(bytes(data)).hexdigest()==receipt['human_source_sha256'],
        'human_exact_C30':bool(np.allclose(human['position'],h['start_position'],rtol=0,atol=1e-6)),
        'human_grounded':max(abs(z) for z in asset['after']['sole_minima_world_z'])<.001,
        'human_kinematic':UsdPhysics.RigidBodyAPI(sim.stage.GetPrimAtPath(h['prim'])).GetKinematicEnabledAttr().Get(),
        'human_preflight_identity':receipt.get('human_expected') is None or asset==receipt['human_expected']}
    query=query_scene();wheel=wheelchair_gate(sim,cfg,receipt,query,cfg['fixed_wheelchairs'][0])
    checks.update({'C10_'+k:v for k,v in wheel['checks'].items()})
    # Geometry-only preflight along the authored human path; never commands robot.
    samples=[];points=np.asarray(h['path_world_xyz'])
    lo,hi=np.asarray(asset['visible_bounds_in_root_m']);half=(hi-lo)/2;center=(hi+lo)/2
    for a,b in zip(points[:-1],points[1:]):
        for p in np.linspace(a,b,10):
            hits=[]
            def floor_hit(hit):
                if str(hit.collision).startswith('/Root/') and hit.normal[2]>.9:
                    hits.append(float(hit.position[2]))
                return True
            query.q.raycast_all(query.carb.Float3(*p[:2],.3),query.carb.Float3(0,0,-1),.6,floor_hit,True)
            overlap=set()
            for z in (.3,.65,1.,1.35,1.6):overlap.update(query.overlap([*p[:2],z],h['radius_m']))
            yaw=math.atan2(b[1]-a[1],b[0]-a[0])+math.pi/2
            c,s=math.cos(yaw),math.sin(yaw);xy=p[:2]+np.array([[c,-s],[s,c]])@center[:2]
            visual_overlap=query.overlap([*xy,center[2]+.001],extent=[*half[:2],half[2]-.002],yaw=yaw)
            samples.append({'position':p.tolist(),'floor':bool(hits and min(abs(z) for z in hits)<.001),
                'overlap_paths':sorted(overlap),'visual_bounds_overlap':visual_overlap})
    checks['human_full_capsule_path_clear']=all(r['floor'] and not r['overlap_paths'] for r in samples)
    checks['human_full_visual_path_clear']=all(not r['visual_bounds_overlap'] for r in samples)
    distance=min(np.linalg.norm(np.asarray(x['position'])[:2]-cfg['fixed_wheelchairs'][0]['position'][:2]) for x in samples)
    checks['human_path_separate_from_C10']=distance>h['radius_m']+np.linalg.norm(sim.pedestrian.wheels['C10']['footprint_half_m'])
    checks['C10_initial_semantics']=read(sim.records.path/'raw/object_visibility/preflight.png.json')['objects']['C10_wheelchair']['human_visible_pixel_count']>0
    checks={k:bool(v) for k,v in checks.items()}
    record={'pass':all(checks.values()),'checks':checks,'wheelchair':wheel,'human_path_samples':samples,'model_calls':0,
        'planned_encounter_is_not_measured_evidence':True}
    write_json(sim.records.path/'pre-model-gate.json',record)
    if not record['pass']:raise RuntimeError('MIXED_SCENE_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))


def motion_preflight(sim,cfg):
    diagnostic=copy.deepcopy(cfg);diagnostic['fixed_humans']=[cfg['oncoming_human'],*cfg['fixed_wheelchairs']]
    grounding_views(sim,diagnostic)
    h=cfg['oncoming_human'];length=float(np.linalg.norm(np.diff(h['path_world_xyz'],axis=0),axis=1).sum())
    end=h['start_sim_s']+length/h['speed_m_s']+.1
    initial=sim.pedestrian.snapshot()['humans']['C30']['position'];frames=[]
    robot_initial=np.array(sim.state()['position'][:2]);max_command=0.;min_separation=float('inf')
    for i in range(math.ceil(end/sim.dt)):
        sim.apply(0.,0.);sim.step()
        max_command=max(max_command,float(np.max(np.abs(sim.adapter.applied))))
        actual=sim.pedestrian.snapshot()['humans']['C30']['position']
        min_separation=min(min_separation,float(np.linalg.norm(np.array(actual[:2])-sim.state()['position'][:2])))
        if i%120==0:
            _,state,path=sim.capture(f'motion_check_{i:06d}.png');frames.append({'sim_time':state['sim_time'],'human':sim.pedestrian.snapshot(),'rgb':str(path.relative_to(sim.records.path))})
    final=sim.pedestrian.snapshot()['humans']['C30']['position']
    checks={'motion_measured':np.linalg.norm(np.array(final)-initial)>10.,
        'end_pose':bool(np.allclose(final,h['path_world_xyz'][-1],rtol=0,atol=.001)),
        'target_tracking':sim.pedestrian.max_error<.001,
        'robot_zero_command_all_ticks':max_command==0.,
        'human_separate_from_waiting_robot':min_separation>cfg['spatial_turn_gate']['robot_conservative_radius_m']+h['radius_m']}
    result={'pass':all(checks.values()),'checks':{k:bool(v) for k,v in checks.items()},'initial_measured':initial,'final_measured':final,
        'max_target_error_m':sim.pedestrian.max_error,'physics_steps':sim.tick,'frames':frames,'model_calls':0,
        'waiting_robot_uncommanded_displacement_m':float(np.linalg.norm(np.array(sim.state()['position'][:2])-robot_initial)),
        'minimum_human_waiting_robot_center_distance_m':min_separation,'maximum_robot_command_abs':max_command,
        'note':'Waiting robot retains normal dynamics; drift is measured, not suppressed. Model trial starts in a fresh simulator.'}
    write_json(sim.records.path/'motion-preflight.json',result)
    if not result['pass']:raise RuntimeError('Human motion preflight failed')
    return result
