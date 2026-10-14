"""C10/C24 wheelchairs and a grounded cart on C24's world +Y side.

No navigation policy changes. All paths below are diagnostic queries only.
"""
import copy
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from research.records import write_json, clocks
from research.cart_geometry import rotation, clearance
from research.hospital_cart import SOURCE, COPY, LABEL, signature, author
from research.hospital_lights_simulation import HospitalLightsSimulation
from tools.two_wheelchairs_v08 import scene as base

GAP = .05


def scenario_config(root, cfg, reference):
    cfg = base.scenario_config(root, cfg, reference)
    cfg['adjacent_cart'] = {
        'id': 'C24_cart', 'prim': COPY, 'source_prim': SOURCE, 'label': LABEL,
        'reference_id': 'C24', 'requested_world_y_offset_m': .4,
        'minimum_world_y_bounds_gap_m': GAP,
        'placement': 'Same world X as C24 wheelchair; move only +world Y until enclosing world bounds have 5 cm gap; bottom at floor Z=0. Long axis parallel to C24 reference tangent.',
        'authorization': 'User selected cart, original instruction, and minimal nonoverlapping spacing instead of overlapping +0.4 m centre placement.'}
    return cfg


def cart_pose(bounds, wheelchair, tangent, gap=GAP):
    """USD row-vector local->world; metres, original Hospital origin, +Z up."""
    lo, hi = np.asarray(bounds, float)
    half = (hi-lo)[:2]/2
    axis = int(np.argmax(half))
    yaw = float(tangent-axis*np.pi/2)
    world_y_half = float(np.abs(rotation(yaw)[1]) @ half)
    center = [wheelchair['center_world_xyz'][0],
              wheelchair['world_bounds'][1][1]+gap+world_y_half, 0.]
    matrix = np.eye(4)
    matrix[:2, :2] = rotation(yaw).T
    pivot = np.array([*(lo[:2]+hi[:2])/2, lo[2]])
    matrix[3, :3] = np.asarray(center)-pivot@matrix[:3, :3]
    return {'center_world_xyz': center, 'yaw_rad': yaw, 'tangent_rad': tangent,
            'long_axis_local': axis, 'footprint_half_m': half.tolist(),
            'height_m': float(hi[2]-lo[2]), 'local_bounds': [lo.tolist(), hi.tolist()],
            'matrix_row_vector': matrix.tolist()}


class Actors(base.WheelchairActor):
    def __init__(self, sim, cfg):
        super().__init__(sim, cfg)
        import omni.client
        from pxr import UsdGeom
        source = sim.stage.GetPrimAtPath(SOURCE)
        bounds = UsdGeom.BBoxCache(0, ['default', 'render', 'proxy']).ComputeUntransformedBound(source).ComputeAlignedRange()
        cart = cart_pose([list(bounds.GetMin()), list(bounds.GetMax())],
                         self.wheels['C24'], cfg['fixed_wheelchairs'][1]['reference_tangent_rad'])
        urls = [cfg['scene']['usd'], cfg['scene']['usd'].rsplit('/', 1)[0]+'/Props/SM_SupplyCart_01e.usd']
        hashes = {}
        for url in urls:
            status, _, data = omni.client.read_file(url)
            if status != omni.client.Result.OK:
                raise ValueError('Cart asset bytes unavailable')
            hashes[url] = hashlib.sha256(bytes(data)).hexdigest()
        frozen = {'scene_usd': cfg['scene']['usd'], 'source_prim': SOURCE,
                  'source_prim_signature': signature(sim.stage, SOURCE),
                  'asset_sha256': hashes, 'cart': cart}
        copied = author(sim.stage, frozen)
        self.cart = {**cart, **copied, 'frozen': frozen,
                     'world_y_offset_from_C24_m': cart['center_world_xyz'][1]-self.wheels['C24']['center_world_xyz'][1],
                     'world_y_bounds_gap_m': copied['world_bounds'][0][1]-self.wheels['C24']['world_bounds'][1][1]}
        write_json(sim.records.path/'cart_asset.json', self.cart)
        self.cart_file = (sim.records.path/'raw/cart_state.csv').open('x')
        self.cart_writer = csv.writer(self.cart_file)
        self.cart_writer.writerow(['tick', 'sim_time', 'x', 'y', 'z', 'yaw', 'matrix_max_error', 'motion_active'])

    def snapshot(self):
        from pxr import UsdGeom
        result = super().snapshot()
        actual = np.asarray(UsdGeom.Xformable(self.sim.stage.GetPrimAtPath(COPY)).ComputeLocalToWorldTransform(0))
        error = float(np.abs(actual-self.cart['matrix_row_vector']).max())
        if error > 1e-12:
            raise ValueError('Static cart moved')
        result.update(object_type='two_static_wheelchairs_and_adjacent_cart',
                      legacy_field_note='human_at_* is legacy naming: no humans in this scene.',
                      cart={'position': self.cart['center_world_xyz'], 'yaw_rad': self.cart['yaw_rad'],
                            'matrix_max_error': error, 'motion_active': False, **clocks()})
        return result

    def measure(self, sim_time):
        super().measure(sim_time)
        s = self.snapshot()['cart']
        self.cart_writer.writerow([self.sim.tick, sim_time, *s['position'], s['yaw_rad'], s['matrix_max_error'], False])

    def close(self):
        super().close()
        self.cart_file.close()


def objects(cfg):
    return [(w['id']+'_wheelchair', w['label']) for w in cfg['fixed_wheelchairs']]+[('C24_cart', LABEL)]


class Simulation(HospitalLightsSimulation):
    def capture(self, name):
        from research.nova_occlusion import segmentation_stats, visibility_state
        from PIL import Image
        rgb, obs, path = super().capture(name)
        packed = self.segmentation.get_data()
        folder = self.records.path/'raw/object_visibility'
        folder.mkdir(exist_ok=True)
        np.save(folder/(name+'.ids.npy'), np.asarray(packed['data']), allow_pickle=False)
        labels = packed['info']['idToLabels']; rows = {}
        for key, label in objects(self.cfg):
            stats, mask = segmentation_stats(packed['data'], labels, label)
            rows[key] = {**stats, 'state': visibility_state(stats, self.cfg['occlusion']['visibility_rule'])}
            Image.fromarray((mask*255).astype('uint8')).save(folder/(name+'.'+key+'.mask.png'))
        write_json(folder/(name+'.json'), {'idToLabels': labels, 'objects': rows,
                   'observation_sim_time': obs['sim_time'], 'model_input': False})
        return rgb, obs, path


def cart_gate(sim, cfg, receipt, query, wheelchair_gate):
    a = sim.pedestrian.cart
    lo, hi = np.asarray(a['world_bounds'])
    center = np.asarray(a['center_world_xyz'])
    # Ignore the added object's own mesh while checking actual Hospital support.
    hits = []
    def floor_hit(hit):
        if str(hit.collision).startswith('/Root/'):
            hits.append({'prim': str(hit.collision), 'distance_m': float(hit.distance),
                         'position': list(map(float, hit.position)), 'normal': list(map(float, hit.normal))})
        return True
    query.q.raycast_all(query.carb.Float3(*center[:2], .3), query.carb.Float3(0, 0, -1), .6, floor_hit, True)
    floor = min(hits, key=lambda x: x['distance_m']) if hits else None
    overlaps = query.overlap([*center[:2], a['height_m']/2+.002],
                             extent=[*a['footprint_half_m'], a['height_m']/2-.002], yaw=a['yaw_rad'])
    radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']
    paths = []
    # Recheck both frozen wheelchair bypasses against the additional cart.
    for idx, b in enumerate(wheelchair_gate['C24']['bypass_probes']):
        vertices = np.asarray(b['vertices_world_xy'])
        xy = np.vstack([u+np.linspace(0, 1, 55)[:, None]*(v-u) for u, v in zip(vertices[:-1], vertices[1:])])
        gap = float(clearance(xy, a, radius).min())
        paths.append({'side': 'right' if idx == 0 else 'left', 'pass': bool(b['pass'] and gap > 0),
                      'cart_proxy_clearance_m': gap, 'vertices_world_xy': vertices.tolist(),
                      'hospital_and_wheelchair_pass': b['pass']})
    checks = {
        'cart_source_unchanged': signature(sim.stage, SOURCE) == a['frozen']['source_prim_signature'],
        'cart_preflight_identity': receipt.get('cart_expected') is None or receipt['cart_expected'] == a,
        'cart_grounded': bool(floor and floor['normal'][2] > .9 and abs(lo[2]-floor['position'][2]) < .001),
        'cart_no_hospital_overlap': not overlaps,
        'cart_static': sim.pedestrian.snapshot()['cart']['matrix_max_error'] == 0.,
        'cart_same_world_x': bool(abs(center[0]-sim.pedestrian.wheels['C24']['center_world_xyz'][0]) < 1e-9),
        'cart_minimal_positive_y_gap': bool(abs(a['world_y_bounds_gap_m']-GAP) < 1e-9),
        'cart_initial_separation': bool(np.linalg.norm(np.array(sim.state()['position'][:2])-center[:2]) > radius+np.linalg.norm(a['footprint_half_m'])),
        'left_local_bypass_with_all_actors': paths[1]['pass']}
    return {'checks': checks, 'floor': floor, 'hospital_overlap_paths': overlaps,
            'paired_bypass_checks': paths, 'coordinate_contract':
            'Side is relative to original C23->C25 route tangent; forward=(cos yaw,sin yaw), left=(-sin yaw,cos yaw). Left is approximately world -Y. Diagnostic paths never drive the robot; failure of one right probe is not proof that every right route is blocked.'}


def grounding_views(sim, cfg):
    # One combined diagnostic invocation: the helper uses exclusive creation
    # for its single grounding_views.json manifest.
    diagnostic = copy.deepcopy(cfg)
    diagnostic['fixed_wheelchairs'].append({
        'id': 'C24_cart', 'position': sim.pedestrian.cart['center_world_xyz'],
        'reference_tangent_rad': cfg['fixed_wheelchairs'][1]['reference_tangent_rad'],
        'label': LABEL})
    base.grounding_views(sim, diagnostic)


def pre_model_gate(sim, cfg, receipt, baseline):
    before=sim.state(); _,obs,_=sim.capture('preflight.png')
    robot=base.read(sim.records.path/'robot_asset.json');old_robot=base.read(baseline/'robot_asset.json')
    lights=base.read(sim.records.path/'hospital_lighting_applied.json');old_lights=base.read(baseline/'hospital_lighting_applied.json')
    forbidden=('/World/PreflightHuman','/World/ExactHumanC10','/World/ExactHumanC24',
               '/World/ExactHumanC30')
    checks={'two_wheelchairs_and_one_cart_config':[w['id'] for w in cfg['fixed_wheelchairs']]==list(base.IDS) and not any(k in cfg for k in ('fixed_humans','stationary_human','pedestrian','stationary_cart')),
        'no_human_or_old_obstacle_prims':all(not sim.stage.GetPrimAtPath(p) for p in forbidden),
        'original_instruction':cfg['instruction']==base.INSTRUCTION,
        'lower_speed':cfg['controller']['max_v']==base.MAX_V and cfg['controller']['max_w']==1.2,
        'extended_budget':cfg['simulation']['predictions']==base.PREDICTIONS,
        'robot_camera_wheels':all(robot[k]==old_robot[k] for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'current_robot_source':robot['source']==receipt['changes']['current_robot_source'],
        'spawn':base.read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting':{k:v for k,v in lights.items() if not k.startswith('collision_')}=={k:v for k,v in old_lights.items() if not k.startswith('collision_')},
        'lighting_preserves_colliders':lights['collision_inventory_unchanged'],
        'completed_rgb_no_physics':before['tick']==obs['tick']==0 and before['position']==obs['position'],
        'initial_brightness':base.read(sim.records.path/'raw/brightness/preflight.png.json')['pass']}
    query=base.query_scene();wheelchair={}
    for w in cfg['fixed_wheelchairs']:
        details=base.wheelchair_gate(sim,cfg,receipt,query,w);wheelchair[w['id']]=details
        checks.update({w['id']+'_'+k:v for k,v in details['checks'].items()})
    a,b=sim.pedestrian.wheels.values()
    checks['added_wheelchairs_separate']=bool(np.linalg.norm(np.array(a['center_world_xyz'][:2])-b['center_world_xyz'][:2])>np.linalg.norm(a['footprint_half_m'])+np.linalg.norm(b['footprint_half_m']))
    checks['C10_initial_semantics']=base.read(sim.records.path/'raw/object_visibility/preflight.png.json')['objects']['C10_wheelchair']['human_visible_pixel_count']>0
    cart=cart_gate(sim,cfg,receipt,query,wheelchair)
    checks.update(cart['checks'])
    record={'cart':cart,'pass':all(checks.values()),'checks':checks,'people':{},'wheelchairs':wheelchair,'model_calls':0,
        'grounding_tolerance_m':.001,'exact_xy_tolerance_m':1e-9,
        'note':'Two static wheelchairs at original C10/C24 XY plus one +Y cart. No people. Diagnostic bypass probes never command the robot.'}
    write_json(sim.records.path/'pre-model-gate.json',record)
    if not record['pass']:raise RuntimeError('C24_ADJACENT_CART_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))
