"""Exact original XY placement; actual shoe-mesh grounding; two fixed actors."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from research.records import write_json, clocks
from research.hospital_lights_simulation import HospitalLightsSimulation
from research.cart_geometry import rotation


def read(path):
    return json.loads(Path(path).read_text())


def exact_config(root, cfg, reference):
    original = read(root/reference/'metadata.json')['config']
    if any(k in original for k in ('stationary_human', 'stationary_cart', 'pedestrian', 'fixed_humans')):
        raise ValueError('The coordinate reference must have no added obstacle')
    result = copy.deepcopy(cfg)
    template = result.pop('stationary_human')
    result['fixed_humans'] = []
    for rid in (10, 24):
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


def mesh_world_points(stage, root_path):
    """USD default/rest pose -> world metres. Skinning points use skeleton space."""
    from pxr import Usd, UsdGeom, UsdSkel, Vt
    cache = UsdSkel.Cache()
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdSkel.Root):
            cache.Populate(UsdSkel.Root(prim), Usd.PrimDefaultPredicate)
    rows = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh): continue
        mesh = UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility() == 'invisible': continue
        points = mesh.GetPointsAttr().Get(Usd.TimeCode.Default())
        query = cache.GetSkinningQuery(prim)
        if query:
            skeleton = UsdSkel.BindingAPI(prim).GetInheritedSkeleton()
            transforms = cache.GetSkelQuery(skeleton).ComputeSkinningTransforms(Usd.TimeCode.Default())
            points = Vt.Vec3fArray(points)
            if not query.ComputeSkinnedPoints(transforms, points, Usd.TimeCode.Default()):
                raise ValueError('Skinning evaluation failed')
            matrix = UsdGeom.Xformable(skeleton).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        else:
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world = np.column_stack([np.asarray(points), np.ones(len(points))])@np.asarray(matrix)
        rows.append({'path': str(prim.GetPath()), 'world_points': world[:, :3], 'skinned': bool(query)})
    if not rows: raise ValueError('Visible actual meshes missing')
    return rows


def mesh_bounds(rows):
    points = np.vstack([r['world_points'] for r in rows])
    boots = [r for r in rows if r['path'].endswith('/opaque__leather__workboots')]
    if len(boots) != 1: raise ValueError('Exactly one existing workboot mesh required')
    feet = boots[0]['world_points']
    # Two soles split around the unchanged root X; per-foot minima are retained.
    split = (feet[:, 0].min()+feet[:, 0].max())/2
    left = feet[feet[:, 0] < split]; right = feet[feet[:, 0] >= split]
    return {'visible_min_world': points.min(0).tolist(), 'visible_max_world': points.max(0).tolist(),
        'shoe_min_world_z': float(feet[:, 2].min()),
        'sole_minima_world_z': [float(left[:, 2].min()), float(right[:, 2].min())],
        'shoe_path': boots[0]['path'], 'shoe_points': len(feet),
        'shoe_skinned': boots[0]['skinned'],
        'mesh_count': len(rows), 'visible_points': len(points)}


class TwoGroundedHumans:
    def __init__(self, sim, cfg):
        from pxr import Gf, UsdGeom
        from research.human_actor import author_human
        from isaacsim.core.utils.semantics import add_labels
        self.sim, self.cfg = sim, cfg
        self.initial = {}; self.assets = {}
        for h in cfg['fixed_humans']:
            if h['position'][:2] != h['reference_observation']['pose'][:2] or h['xy_offset_m'] != [0., 0.]:
                raise ValueError('Exact saved XY required')
            _, info = author_human(sim.stage, {**h, 'start_position': h['position']})
            before = mesh_bounds(mesh_world_points(sim.stage, h['prim']))
            # Only the visual child's vertical translation changes. Root XY stays exact.
            visual = UsdGeom.Xformable(sim.stage.GetPrimAtPath(h['prim']+'/VisualTransform'))
            op = next(op for op in visual.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate)
            old = np.array(op.Get()); new = old.copy()
            new[2] -= before['shoe_min_world_z']-h['position'][2]
            op.Set(Gf.Vec3d(*new))
            capsule = UsdGeom.Capsule(sim.stage.GetPrimAtPath(h['prim']+'/Collider'))
            cop = next(op for op in capsule.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate)
            old_capsule = np.array(cop.Get()).tolist()
            cop.Set(Gf.Vec3d(0., 0., h['capsule_cylinder_height_m']/2+h['radius_m']))
            after = mesh_bounds(mesh_world_points(sim.stage, h['prim']))
            add_labels(sim.stage.GetPrimAtPath(h['prim']), ['occluded_human', 'exact_'+h['id']])
            self.assets[h['id']] = {'id': h['id'], 'config': h, 'original_asset_info': info,
                'before': before, 'after': after, 'old_visual_translation_m': old.tolist(),
                'new_visual_translation_m': new.tolist(), 'visual_delta_world_z_m': float(new[2]-old[2]),
                'old_capsule_center_local': old_capsule,
                'capsule_center_local': [0., 0., h['capsule_cylinder_height_m']/2+h['radius_m']],
                'coordinate_contract': 'Original Hospital world origin, +Z up, metres. Mesh vertices evaluate default rest-pose skinning into skeleton space, then skeleton local-to-world USD row matrix. Root XY unchanged. Floor confirmed by actual PhysX rays before inference.'}
        write_json(sim.records.path/'humans_assets.json', self.assets)
        self.file = (sim.records.path/'raw/humans_state.csv').open('x')
        self.writer = csv.DictWriter(self.file, fieldnames=['tick','sim_time','id','x','y','z','qx','qy','qz','qw','motion_active'])
        self.writer.writeheader()

    def update(self, sim_time):
        pass

    def snapshot(self):
        import omni.physx
        state = self.sim.state(); result = {}
        for h in self.cfg['fixed_humans']:
            p = omni.physx.get_physx_interface().get_rigidbody_transformation(h['prim'])
            if not p['ret_val']: raise RuntimeError('Missing measured human PhysX pose')
            xyz = np.array(p['position']); quat = np.array(p['rotation'])
            if h['id'] not in self.initial: self.initial[h['id']] = (xyz.copy(), quat.copy())
            if np.linalg.norm(xyz-self.initial[h['id']][0]) > 1e-6 or not np.allclose(quat, self.initial[h['id']][1], rtol=0, atol=1e-7):
                raise RuntimeError('Stationary human moved or rotated')
            result[h['id']] = {'position': xyz.tolist(), 'quaternion_xyzw': quat.tolist(),
                'motion_active': False, 'sim_time': state['sim_time'], 'event_source': 'main_thread_measured_PhysX', **clocks()}
        return {'object_type': 'two_stationary_grounded_humans', 'humans': result, 'sim_time': state['sim_time'],
            'legacy_field_note': 'human_at_* now explicitly contains two individually measured people'}

    def measure(self, sim_time):
        for name, state in self.snapshot()['humans'].items():
            self.writer.writerow(dict(zip(self.writer.fieldnames, [self.sim.tick, sim_time, name,
                *state['position'], *state['quaternion_xyzw'], False])))

    def close(self):
        self.file.close()


class TwoHumanSimulation(HospitalLightsSimulation):
    def capture(self, name):
        from research.nova_occlusion import segmentation_stats, visibility_state
        from PIL import Image
        rgb, obs, path = super().capture(name)
        packed = self.segmentation.get_data()
        folder = self.records.path/'raw/individual_visibility'; folder.mkdir(exist_ok=True)
        rows = {}
        for h in self.cfg['fixed_humans']:
            stats, mask = segmentation_stats(packed['data'], packed['info']['idToLabels'], 'exact_'+h['id'])
            rows[h['id']] = {**stats, 'state': visibility_state(stats, self.cfg['occlusion']['visibility_rule'])}
            Image.fromarray((mask*255).astype('uint8')).save(folder/(name+'.'+h['id']+'.mask.png'))
        write_json(folder/(name+'.json'), {'observation_sim_time': obs['sim_time'], 'people': rows, 'model_input': False})
        return rgb, obs, path


def query_scene():
    import carb
    from omni.physx import get_physx_scene_query_interface
    from research.blind_corner_queries import Queries
    query = Queries.__new__(Queries)
    query.carb = carb; query.q = get_physx_scene_query_interface()
    query.floor_prim = query.floor([19., 0.])[1]
    return query


def pre_model_gate(sim, cfg, receipt, baseline):
    import omni.client
    from pxr import UsdGeom, UsdPhysics
    from isaacsim.core.utils.semantics import get_labels
    before = sim.state(); _, obs, _ = sim.capture('preflight.png')
    robot = read(sim.records.path/'robot_asset.json'); old_robot = read(baseline/'robot_asset.json')
    lights = read(sim.records.path/'hospital_lighting_applied.json'); old_lights = read(baseline/'hospital_lighting_applied.json')
    checks = {'two_configured_people': [h['id'] for h in cfg['fixed_humans']] == ['C10','C24'],
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
            'semantic_labels':set(get_labels(root).get('class',[]))=={'occluded_human','exact_'+h['id']},
            'local_bypass_exists':any(p['pass'] for p in paths)}
        checks.update({h['id']+'_'+k:v for k,v in per.items()})
        details[h['id']]={'checks':per,'current_mesh_bounds':current,'floor_z_m':floor_z,'measured':measured[h['id']],
            'floor_rays':floor_hits,'capsule_bottom_world_m':capsule_bottom,
            'authored_matrix_row_vector':matrix.tolist(),'overlap_paths':overlaps,'bypass_probes':paths}
    record={'pass':all(checks.values()),'checks':checks,'people':details,'model_calls':0,
        'grounding_tolerance_m':.001,'measured_XY_tolerance_m':1e-6,
        'note':'XY authored exactly from original no-added-obstacle C10/C24; only visual/collider vertical placement corrected. Local bypass probes never command the robot.'}
    write_json(sim.records.path/'pre-model-gate.json',record)
    if not record['pass']:raise RuntimeError('EXACT_HUMANS_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))


def grounding_views(sim,cfg):
    """Diagnostic side views at frozen t=0, not model observations."""
    import omni.replicator.core as rep
    from pxr import UsdGeom,Gf
    from PIL import Image
    views=[];before=sim.state()
    for h in cfg['fixed_humans']:
        forward=np.array([np.cos(h['reference_tangent_rad']),np.sin(h['reference_tangent_rad']),0.])
        for label,distance,eye_z,target_z in [('feet',1.25,.40,.12),('body',2.5,1.1,.85)]:
            center=np.array(h['position']);eye=center-distance*forward;eye[2]+=eye_z;target=center.copy();target[2]+=target_z
            path='/World/GroundingCamera_'+h['id']+'_'+label
            cam=UsdGeom.Camera.Define(sim.stage,path);cam.CreateFocalLengthAttr(20.);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100.))
            matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
            cam.AddTransformOp().Set(matrix)
            product=rep.create.render_product(path,(960,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
            sim.world.render();rep.orchestrator.step(rt_subframes=4,delta_time=0.,pause_timeline=False,wait_for_render=True)
            a=np.array(rgb.get_data())[:,:,:3].copy();name=h['id']+'_'+label+'.png'
            Image.fromarray(a).save(sim.records.path/'diagnostics'/name)
            after=sim.state()
            if before['tick']!=after['tick'] or before['position']!=after['position'] or before['sim_time']!=after['sim_time']:
                raise RuntimeError('Grounding diagnostic advanced physics')
            views.append({'id':h['id'],'view':label,'file':name,'camera_world_row_matrix':np.asarray(matrix).tolist(),
                'tick':after['tick'],'sim_time':after['sim_time'],'model_input':False})
            rgb.detach([product]);product.destroy()
    write_json(sim.records.path/'grounding_views.json',{'views':views,'navigation_steps':0,'model_calls':0})
