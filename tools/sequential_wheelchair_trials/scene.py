"""Use the verified native wheelchair actor and capture, with scoped text override."""
import copy
import numpy as np
from research.records import write_json
from tools.two_wheelchairs_v08.scene import (
    WheelchairActor, WheelchairSimulation, wheelchair_gate, query_scene, grounding_views, read)
from tools.sequential_wheelchair_trials.config import geometry_config, SCENARIOS, SUFFIX


class Actors(WheelchairActor):
    def snapshot(self):
        value=super().snapshot()
        value['object_type']='static_wheelchairs_only'
        value['legacy_field_note']='human_at_* is a legacy key; zero humans; static USD wheelchair matrices verified'
        return value


class Simulation(WheelchairSimulation):
    def __init__(self,cfg,records,actor_factory=None):
        geometry=geometry_config(cfg)
        super().__init__(geometry,records,actor_factory=actor_factory)
        self.cfg=cfg
        write_json(records.path/'instruction-intervention.json',{
            'geometry_guard_instruction':geometry['instruction'],'model_instruction':cfg['instruction'],
            'only_instruction_projected_for_geometry':geometry_config(cfg)==geometry,
            'scope':'Original instruction is used only by unchanged official geometry guard; actual requested instruction enters every model request.'})


def pre_model_gate(sim,cfg,receipt,baseline):
    scenario=receipt['changes']['scenario'];e21=scenario==SCENARIOS[0]
    before=sim.state();rgb,obs,_=sim.capture('preflight.png');after=sim.state()
    robot=read(sim.records.path/'robot_asset.json');old=read(baseline/'robot_asset.json')
    lights=read(sim.records.path/'hospital_lighting_applied.json');oldlights=read(baseline/'hospital_lighting_applied.json')
    quality=read(sim.records.path/'raw/brightness/preflight.png.json')
    original=cfg['official_episode']['episode']['instruction']
    expected_instruction=original+SUFFIX if scenario==SCENARIOS[2] else original
    checks={
        'wheelchair_count_and_ids':[w['id'] for w in cfg['fixed_wheelchairs']]==(['C10'] if e21 else ['C10','C24']),
        'no_human_or_cart_config':not any(k in cfg for k in ('fixed_humans','stationary_human','pedestrian','stationary_cart','adjacent_cart')),
        'no_human_or_added_cart_prims':all(not sim.stage.GetPrimAtPath(p) for p in ('/World/PreflightHuman','/World/ExactHumanC10','/World/ExactHumanC24','/World/ExactHumanC30','/World/E16Cart')),
        'requested_instruction':cfg['instruction']==expected_instruction,
        'requested_speed_budget':cfg['controller']['max_v']==(.8 if e21 else .5) and cfg['simulation']['predictions']==(90 if e21 else 144) and cfg['controller']['max_w']==1.2,
        'robot_camera_wheels':all(np.allclose(robot[k],old[k],rtol=0,atol=1e-8) if k=='camera_body_transform' else robot[k]==old[k] for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'current_robot_source':robot['source']==receipt['changes']['current_robot_source'],
        'spawn':read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting':{k:v for k,v in lights.items() if not k.startswith('collision_')}=={k:v for k,v in oldlights.items() if not k.startswith('collision_')},
        'lighting_preserves_colliders':lights['collision_inventory_unchanged'],
        'completed_rgb_no_physics':before['tick']==obs['tick']==after['tick']==0 and before['position']==obs['position']==after['position'] and before['quaternion_wxyz']==after['quaternion_wxyz'],
        'full_rgb':rgb.shape==(1080,1920,3),
        'camera_clear':not read(sim.records.path/'raw/scene_observations/preflight.png.json')['camera_near_scene_collision_paths'],
    }
    if not e21:checks['initial_brightness']=quality['pass']
    query=query_scene();details={}
    for w in cfg['fixed_wheelchairs']:
        details[w['id']]=wheelchair_gate(sim,cfg,receipt,query,w)
        checks.update({w['id']+'_'+k:v for k,v in details[w['id']]['checks'].items()})
    if len(cfg['fixed_wheelchairs'])==2:
        a,b=sim.pedestrian.wheels.values()
        checks['wheelchairs_separate']=bool(np.linalg.norm(np.array(a['center_world_xyz'][:2])-b['center_world_xyz'][:2])>np.linalg.norm(a['footprint_half_m'])+np.linalg.norm(b['footprint_half_m']))
    write_json(sim.records.path/'pre-model-gate.json',{'pass':all(checks.values()),'checks':checks,'wheelchairs':details,'model_calls':0,
        'rgb_quality':quality,'rgb_quality_pass':quality['pass'],
        'scope':'E21 overexposure remains an independent FAIL, not a qualified RGB baseline. User-requested static-object diagnostic only.',
        'geometry_note':'Actual source mesh/colliders static; exact saved XY, floor Z=0. Bypass geometry queries do not control navigation.'})
    if not all(checks.values()):raise RuntimeError('SEQUENTIAL_WHEELCHAIR_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))
