"""Fixed scene additions and a model-free geometry gate; no control changes."""
import copy
import json
from pathlib import Path
import numpy as np
from research.cart_geometry import rotation, outline, path_clearance


def read(path):
    return json.loads(Path(path).read_text())


def scenario_config(root, cfg, comparison, scenario):
    if scenario not in ('human', 'cart'):
        raise ValueError('Exactly one human or one cart scenario required')
    previous = read(root/comparison/'runtime-config.json')
    previous['simulation']['headless'] = cfg['simulation']['headless']
    if cfg != previous:
        raise ValueError('The successful modified-copy baseline configuration changed')
    result = read(root/comparison/'derived/api-smoke/summary.json')
    if not (result['runtime_pass'] and result['goal_entered_strict_2m'] and result['nonfloor_contact_spans'] == 0):
        raise ValueError('A completed contact-free baseline with goal entry is required')
    old = read(root/'outputs/e16-cart-preflight-20261010-01/freeze.json')
    event = read(root/comparison/'raw/requests/request_000024.json')
    lo, hi = np.asarray(old['cart']['local_bounds'])
    tangent = float(np.pi)
    axis = int(np.argmax((hi-lo)[:2]))
    yaw = float((tangent-axis*np.pi/2+np.pi)%(2*np.pi)-np.pi)
    center = [event['agent_pose_at_observation'][0], event['agent_pose_at_observation'][1]-.65, 0.]
    pivot = np.array([*(lo[:2]+hi[:2])/2, lo[2]])
    matrix = np.eye(4)
    matrix[:2, :2] = rotation(yaw).T
    matrix[3, :3] = np.asarray(center)-pivot@matrix[:3, :3]
    frozen = {k: copy.deepcopy(old[k]) for k in ('source_prim', 'asset_url', 'scene_usd', 'copy_prim',
        'asset_sha256', 'source_prim_signature', 'source_world_matrix_row_vector', 'visibility_rules', 'robot_radius_m')}
    frozen['cart'] = {'center_world_xyz': center, 'yaw_rad': yaw, 'tangent_rad': tangent,
        'long_axis_local': axis, 'footprint_half_m': ((hi-lo)[:2]/2).tolist(),
        'height_m': float(hi[2]-lo[2]), 'matrix_row_vector': matrix.tolist(), 'local_bounds': [lo.tolist(), hi.tolist()]}
    frozen.update(comparison_run=str(comparison), placement_observation=event['observation'],
        coordinate_convention='Original Hospital origin; world XYZ metres, +Z up, yaw CCW about +Z from +X; USD row-vector local-to-world matrix. Robot quaternions w,x,y,z.',
        placement_rule='Near user-selected C24: same world X, world Y minus 0.65 m to leave a north bypass; long axis parallel to Hospital world X, local long axis points west. Floor Z=0 checked before inference. Not exact C24 and not a recomputed model route.',
        motion='static collider; no rigid body, pose changes, trigger, or motion commands',
        comparison_scope='Previous single run is descriptive; stochastic and asynchronous execution is not a deterministic paired control.')
    cfg = copy.deepcopy(cfg)
    if scenario == 'human':
        cfg['stationary_human']['position'] = center
        cfg['stationary_human']['start_position'] = center.copy()
    else:
        del cfg['stationary_human']
        cfg['stationary_cart'] = frozen
    return cfg


def cart_gate(sim, cfg):
    """Query actual scene; bypass paths are feasibility probes, never commands."""
    import carb
    import omni.client
    import hashlib
    from omni.physx import get_physx_scene_query_interface
    from research.blind_corner_queries import Queries
    from research.records import write_json
    frozen = cfg['stationary_cart']; cart = frozen['cart']
    query = Queries.__new__(Queries)
    query.carb = carb; query.q = get_physx_scene_query_interface()
    query.floor_prim = query.floor([19., 0.])[1]
    assets = {}
    for url, expected in frozen['asset_sha256'].items():
        status, _, data = omni.client.read_file(url)
        assets[url] = status == omni.client.Result.OK and hashlib.sha256(bytes(data)).hexdigest() == expected
    floor = [query.floor(p) for p in outline(cart)[::17]]
    center = np.asarray(cart['center_world_xyz'][:2]); half = cart['footprint_half_m']
    height = cart['height_m']; radius = frozen['robot_radius_m']
    overlap = query.overlap([*center, height/2+.001], extent=[*half, height/2-.001], yaw=cart['yaw_rad'])
    comparison = Path(__file__).resolve().parents[2]/frozen['comparison_run']
    import csv
    with (comparison/'robot_state.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in rows])
    baseline = path_clearance(xy, cart, radius)
    initial = path_clearance([sim.state()['position'][:2]], cart, radius)['minimum_clearance_m']
    paths = []
    for side in (-1, 1):
        for margin in (.05, .10, .15, .20):
            offset = side*(min(half)+radius+margin)
            local = np.array([[-1.8, 0], [-1.35, offset], [1.35, offset], [1.8, 0]])
            vertices = local@rotation(cart['tangent_rad']).T+center
            points = np.vstack([a+np.linspace(0, 1, 46)[:, None]*(b-a) for a, b in zip(vertices[:-1], vertices[1:])])
            trace = query.robot_path(points, radius+.05)
            clear = path_clearance(points, cart, radius)['minimum_clearance_m']
            ok = clear > 0 and all(r['floor'] and not r['overlap'] for r in trace)
            paths.append({'pass': bool(ok), 'side_in_baseline_travel_frame': side, 'margin_m': margin,
                'vertices_world_xy': vertices.tolist(), 'cart_proxy_clearance_m': clear,
                'actual_scene_queries': trace})
    checks = {'asset_hashes': all(assets.values()), 'floor_support': all(r[0] for r in floor),
        'no_hospital_overlap': not overlap,
        'initial_robot_cart_separated': initial > 0, 'baseline_path_obstructed': baseline['minimum_clearance_m'] < 0,
        'connected_local_bypass_exists': any(p['pass'] for p in paths),
        'static_cart_transform_unchanged': sim.pedestrian.snapshot()['motion_active'] is False,
        'no_added_human': not sim.stage.GetPrimAtPath('/World/PreflightHuman')}
    record = {'checks': checks, 'pass': all(checks.values()), 'model_calls': 0,
        'overlap_paths': overlap, 'floor': floor, 'asset_hashes': assets, 'baseline': baseline,
        'initial_robot_cart_clearance_m': initial,
        'bypass_probes': paths, 'note': 'Local sampled geometric feasibility using enclosing bounds/disk; not a model trajectory or exact continuous collision proof.'}
    write_json(sim.records.path/'cart-pre-model-gate.json', record)
    if not record['pass']:
        raise RuntimeError('CART_PREREQUISITE_FAILED: '+str([k for k, v in checks.items() if not v]))


def cart_pre_model_gate(sim, cfg, receipt, baseline):
    from research.records import write_json
    before = sim.state()
    _, obs, _ = sim.capture('preflight.png')
    robot = read(sim.records.path/'robot_asset.json'); old = read(baseline/'robot_asset.json')
    lights = read(sim.records.path/'hospital_lighting_applied.json')
    old_lights = read(baseline/'hospital_lighting_applied.json')
    checks = {
        'robot_camera_wheels': all(robot[k] == old[k] for k in ('cameras', 'camera_body_transform',
            'verified_drive_wheels', 'verified_wheel_base', 'controller_radius', 'controller_base')),
        'robot_source_matches_prepared_copy': robot['source'] == receipt['changes']['current_robot_source'],
        'spawn_valid': read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting_unchanged': {k: v for k, v in lights.items() if not k.startswith('collision_')} ==
            {k: v for k, v in old_lights.items() if not k.startswith('collision_')},
        'lighting_preserves_collision_geometry': lights['collision_inventory_unchanged'],
        'brightness': read(sim.records.path/'raw/brightness/preflight.png.json')['pass'],
        'capture_no_physics_advance': before['tick'] == obs['tick'] == 0 and before['position'] == obs['position'],
        'exactly_cart_scenario': 'stationary_human' not in cfg and bool(sim.stage.GetPrimAtPath('/World/E16Cart')),
    }
    write_json(sim.records.path/'pre-model-gate.json', {'checks': checks, 'pass': all(checks.values()), 'model_calls': 0})
    if not all(checks.values()):
        raise RuntimeError('CART_RUNTIME_GATE_FAILED: '+str([k for k, v in checks.items() if not v]))
    cart_gate(sim, cfg)


def human_geometry_gate(sim, cfg, comparison):
    import carb
    import csv
    from omni.physx import get_physx_scene_query_interface
    from research.blind_corner_queries import Queries
    from research.records import write_json
    query = Queries.__new__(Queries)
    query.carb = carb; query.q = get_physx_scene_query_interface()
    query.floor_prim = query.floor([19., 0.])[1]
    h = cfg['stationary_human']; radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']
    center = np.array(h['position'][:2]); paths = []
    for side in (-1, 1):
        offset = side*(radius+h['radius_m']+.05)
        vertices = np.array([[-1.8, 0], [-1.35, offset], [1.35, offset], [1.8, 0]])@rotation(np.pi).T+center
        points = np.vstack([a+np.linspace(0, 1, 46)[:, None]*(b-a) for a, b in zip(vertices[:-1], vertices[1:])])
        trace = query.robot_path(points, radius+.05)
        clearance = float(np.linalg.norm(points-center, axis=1).min()-radius-h['radius_m'])
        paths.append({'pass': bool(clearance > 0 and all(r['floor'] and not r['overlap'] for r in trace)),
            'vertices_world_xy': vertices.tolist(), 'human_proxy_clearance_m': clearance, 'actual_scene_queries': trace})
    with (comparison/'robot_state.csv').open() as stream:
        xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in csv.DictReader(stream)])
    baseline_clear = float(np.linalg.norm(xy-center, axis=1).min()-radius-h['radius_m'])
    checks = {'connected_local_bypass_exists': any(p['pass'] for p in paths),
        'baseline_path_obstructed': baseline_clear < 0,
        'no_added_cart': not sim.stage.GetPrimAtPath('/World/E16Cart'),
        'exactly_human_scenario': 'stationary_cart' not in cfg}
    write_json(sim.records.path/'human-geometry-gate.json', {'checks': checks, 'pass': all(checks.values()),
        'baseline_sampled_proxy_clearance_m': baseline_clear, 'bypass_probes': paths, 'model_calls': 0})
    if not all(checks.values()):
        raise RuntimeError('HUMAN_GEOMETRY_GATE_FAILED: '+str([k for k, v in checks.items() if not v]))
