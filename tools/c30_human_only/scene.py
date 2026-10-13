"""Single fixed C30 human, reusing measured boot grounding and the original controller."""
import copy
import hashlib
import numpy as np
from research.records import write_json
from research.cart_geometry import rotation
from research.hospital_lights_simulation import HospitalLightsSimulation
from tools.exact_two_humans.scene import (
    read, TwoGroundedHumans, mesh_bounds, mesh_world_points,
    query_scene, grounding_views)

INSTRUCTION = ('Move forward toward the staircase, then turn left to enter the hallway. '
               'Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.')


def exact_c30_config(root, cfg, reference):
    original = read(root/reference/'metadata.json')['config']
    if any(k in original for k in ('stationary_human', 'stationary_cart', 'pedestrian', 'fixed_humans', 'fixed_wheelchair')):
        raise ValueError('The coordinate reference must have no added obstacle')
    result = copy.deepcopy(cfg)
    template = result.pop('stationary_human')
    result['fixed_humans'] = []
    for rid in (30,):
        event = read(root/reference/f'raw/requests/request_{rid:06d}.json')
        if event['request_id'] != rid or event['agent_pose_at_observation'] != event['observation']['pose']:
            raise ValueError('Invalid saved coordinate reference')
        prev = read(root/reference/f'raw/requests/request_{rid-1:06d}.json')['agent_pose_at_observation'][:2]
        next_ = read(root/reference/f'raw/requests/request_{rid+1:06d}.json')['agent_pose_at_observation'][:2]
        delta = np.asarray(next_)-prev
        if np.linalg.norm(delta) < .1:
            raise ValueError('Reference tangent is undefined')
        human = copy.deepcopy(template)
        human.update(id=f'C{rid}', prim=f'/World/ExactHumanC{rid}',
            position=[*event['agent_pose_at_observation'][:2], 0.],
            start_position=[*event['agent_pose_at_observation'][:2], 0.],
            reference_run=str(reference), reference_request_id=rid, reference_observation=event['observation'],
            reference_tangent_rad=float(np.arctan2(delta[1], delta[0])),
            xy_offset_m=[0., 0.], motion='stationary', speed_m_s=0.,
            grounding='Actual rest-pose skinned workboot vertices to raycast floor; visual Z only. Capsule bottom at floor; no size changes.')
        result['fixed_humans'].append(human)
    if any(k in result for k in ('stationary_human', 'stationary_cart', 'pedestrian')):
        raise ValueError('Unexpected additional actor')
    return result


def scenario_config(root, cfg, reference):
    if cfg['instruction'] != INSTRUCTION:
        raise ValueError('Original instruction required')
    if any(k in cfg for k in ('fixed_wheelchair', 'fixed_humans', 'stationary_cart', 'pedestrian')):
        raise ValueError('Unexpected additional actor')
    result = exact_c30_config(root, cfg, reference)
    result['fixed_humans'] = [h for h in result['fixed_humans'] if h['id'] == 'C30']
    if len(result['fixed_humans']) != 1:
        raise ValueError('Exactly one C30 human required')
    return result


class SingleGroundedHuman(TwoGroundedHumans):
    def __init__(self, sim, cfg):
        if [h['id'] for h in cfg['fixed_humans']] != ['C30']:
            raise ValueError('Exactly one C30 human required')
        super().__init__(sim, cfg)
        from isaacsim.core.utils.semantics import add_labels
        add_labels(sim.stage.GetPrimAtPath(cfg['fixed_humans'][0]['prim']), ['occluded_human'])

    def snapshot(self):
        result = super().snapshot()
        result['object_type'] = 'one_stationary_grounded_human'
        result['legacy_field_note'] = 'human_at_* contains only measured C30; C10, C24 and added wheelchair absent'
        return result


class SingleHumanSimulation(HospitalLightsSimulation):
    def capture(self, name):
        rgb, obs, path = super().capture(name)
        packed = self.segmentation.get_data()
        folder = self.records.path/'raw/semantic_ids'
        folder.mkdir(exist_ok=True)
        np.save(folder/(name+'.npy'), np.asarray(packed['data']), allow_pickle=False)
        write_json(folder/(name+'.json'), {
            'idToLabels': packed['info']['idToLabels'],
            'label': 'occluded_human', 'prim': '/World/ExactHumanC30',
            'observation_sim_time': obs['sim_time'], 'model_input': False})
        return rgb, obs, path


def pre_model_gate(sim, cfg, receipt, baseline):
    import omni.client
    from pxr import UsdGeom, UsdPhysics
    from isaacsim.core.utils.semantics import get_labels
    before = sim.state(); _, obs, _ = sim.capture('preflight.png')
    robot = read(sim.records.path/'robot_asset.json'); old_robot = read(baseline/'robot_asset.json')
    lights = read(sim.records.path/'hospital_lighting_applied.json'); old_lights = read(baseline/'hospital_lighting_applied.json')
    checks = {'one_C30_person': [h['id'] for h in cfg['fixed_humans']] == ['C30'],
        'original_instruction': cfg['instruction'] == INSTRUCTION,
        'C10_C24_and_added_wheelchair_absent': all(not sim.stage.GetPrimAtPath(p) for p in ('/World/ExactHumanC10', '/World/ExactHumanC24', '/World/ExactWheelchairC24', '/World/ExactWheelchairC30')),
        'no_legacy_human_or_cart': not sim.stage.GetPrimAtPath('/World/PreflightHuman') and not sim.stage.GetPrimAtPath('/World/E16Cart'),
        'robot_camera_wheels': all(robot[k] == old_robot[k] for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'current_robot_source': robot['source'] == receipt['changes']['current_robot_source'],
        'spawn': read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting': {k:v for k,v in lights.items() if not k.startswith('collision_')} == {k:v for k,v in old_lights.items() if not k.startswith('collision_')},
        'lighting_preserves_colliders': lights['collision_inventory_unchanged'],
        'completed_rgb_no_physics': before['tick'] == obs['tick'] == 0 and before['position'] == obs['position'],
        'initial_brightness': read(sim.records.path/'raw/brightness/preflight.png.json')['pass']}
    query = query_scene(); measured = sim.pedestrian.snapshot()['humans']; details = {}
    radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']
    for h in cfg['fixed_humans']:
        root = sim.stage.GetPrimAtPath(h['prim']); matrix = np.array(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0))
        capsule = UsdGeom.Capsule(sim.stage.GetPrimAtPath(h['prim']+'/Collider'))
        body = UsdPhysics.RigidBodyAPI(root); current = mesh_bounds(mesh_world_points(sim.stage,h['prim']))
        floor_hits=[]
        def floor_hit(hit):
            # A ray starting inside the human capsule can hit that capsule at
            # distance zero. Only original Hospital colliders establish floor.
            if str(hit.collision).startswith('/Root/'):
                floor_hits.append({'prim':str(hit.collision),'distance_m':float(hit.distance),
                    'position':list(map(float,hit.position)),'normal':list(map(float,hit.normal))})
            return True
        query.q.raycast_all(query.carb.Float3(*h['position'][:2], .3), query.carb.Float3(0,0,-1), .6, floor_hit, True)
        floor=min(floor_hits,key=lambda r:r['distance_m']) if floor_hits else None
        floor_z = floor['position'][2] if floor else None
        cap_matrix=np.array(UsdGeom.Xformable(capsule).ComputeLocalToWorldTransform(0))
        capsule_bottom=float(cap_matrix[3,2]-capsule.GetHeightAttr().Get()/2-capsule.GetRadiusAttr().Get())
        source, _, data = omni.client.read_file(h['asset'])
        asset_sha = hashlib.sha256(bytes(data)).hexdigest() if source == omni.client.Result.OK else None
        lo,hi = np.array(current['visible_min_world']),np.array(current['visible_max_world'])
        overlaps = query.overlap(((lo+hi)/2).tolist(), extent=[*(hi[:2]-lo[:2])/2, (hi[2]-lo[2])/2-.001])
        paths=[]; center=np.array(h['position'][:2]); yaw=h['reference_tangent_rad']
        for side in (-1,1):
            off=side*(radius+h['radius_m']+.05)
            vertices=np.array([[-1.8,0],[-1.35,off],[1.35,off],[1.8,0]])@rotation(yaw).T+center
            points=np.vstack([a+np.linspace(0,1,46)[:,None]*(b-a) for a,b in zip(vertices[:-1],vertices[1:])])
            trace=query.robot_path(points,radius+.05)
            actor_clear = min(float(np.linalg.norm(points-np.array(other['position'][:2]),axis=1).min()-radius-other['radius_m']) for other in cfg['fixed_humans'])
            paths.append({'pass':bool(actor_clear>0 and all(r['floor'] and not r['overlap'] for r in trace)),
                'vertices_world_xy':vertices.tolist(),'minimum_person_proxy_clearance_m':actor_clear,'trace':trace})
        per={'source_hash':asset_sha==receipt['human_source_sha256'],
            'authored_exact_reference_xy':matrix[3,:2].tolist()==h['reference_observation']['pose'][:2],
            'measured_exact_xy_within_1um':bool(np.allclose(measured[h['id']]['position'][:2],h['reference_observation']['pose'][:2],rtol=0,atol=1e-6)),
            'root_zero_yaw':bool(np.allclose(matrix[:3,:3],np.eye(3),atol=1e-9,rtol=0)),
            'kinematic':bool(body.GetKinematicEnabledAttr().Get()),
            'capsule_size':capsule.GetRadiusAttr().Get()==.25 and capsule.GetHeightAttr().Get()==1.2 and capsule.GetAxisAttr().Get()=='Z',
            'soles_grounded_within_1mm':bool(floor_z is not None and max(abs(z-floor_z) for z in current['sole_minima_world_z']) < .001),
            'capsule_grounded_within_1mm':bool(floor_z is not None and abs(capsule_bottom-floor_z)<.001),
            'floor_support':bool(floor and floor['normal'][2]>.9 and abs(floor_z)<.001),
            'no_visual_structure_overlap':not overlaps,
            'initial_robot_separation':bool(np.linalg.norm(np.array(obs['position'][:2])-center)>radius+h['radius_m']),
            'semantic_labels':set(get_labels(root).get('class',[]))=={'occluded_human'},
            'local_bypass_exists':any(p['pass'] for p in paths)}
        checks.update({h['id']+'_'+k:v for k,v in per.items()})
        details[h['id']]={'checks':per,'current_mesh_bounds':current,'floor_z_m':floor_z,'measured':measured[h['id']],
            'floor_rays':floor_hits,'capsule_bottom_world_m':capsule_bottom,
            'authored_matrix_row_vector':matrix.tolist(),'overlap_paths':overlaps,'bypass_probes':paths}
    record={'pass':all(checks.values()),'checks':checks,'people':details,'model_calls':0,
        'grounding_tolerance_m':.001,'measured_XY_tolerance_m':1e-6,
        'note':'XY authored exactly from original no-added-obstacle C30; only visual/collider vertical placement corrected. Local bypass probes never command the robot.'}
    write_json(sim.records.path/'pre-model-gate.json',record)
    if not record['pass']:raise RuntimeError('C30_HUMAN_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))
