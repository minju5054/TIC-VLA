"""Initial E21 checks only; no actor creation or navigation policy changes."""
import numpy as np
from tools.e21_current_render.runner import read, write


def pre_model_gate(sim, cfg, receipt, baseline):
    before = sim.state()
    rgb, observation, _ = sim.capture('preflight.png')
    after = sim.state()
    robot = read(sim.records.path/'robot_asset.json')
    historical = read(baseline/'robot_asset.json')
    checks = {
        'official_e21': cfg['official_episode']['episode_id'] == 'episode_21',
        'no_added_actors': sim.pedestrian is None and not any(k in cfg for k in ('stationary_human', 'stationary_cart', 'dynamic_human')),
        'robot_camera_wheels': all(np.allclose(robot[k], historical[k], rtol=0, atol=1e-8) if k == 'camera_body_transform' else robot[k] == historical[k]
            for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'robot_source_matches_prepared': robot['source'] == receipt['changes']['current_robot_source'],
        'spawn_valid': read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting_unchanged': read(sim.records.path/'hospital_lighting_applied.json') == read(baseline/'hospital_lighting_applied.json'),
        'full_rgb': rgb.shape == (1080,1920,3) and str(rgb.dtype) == 'uint8',
        'brightness': read(sim.records.path/'raw/brightness/preflight.png.json')['pass'],
        'capture_no_physics_advance': before['tick'] == observation['tick'] == after['tick'] == 0 and before['position'] == observation['position'] == after['position'] and before['quaternion_wxyz'] == after['quaternion_wxyz'],
        'camera_clear': not read(sim.records.path/'raw/scene_observations/preflight.png.json')['camera_near_scene_collision_paths'],
    }
    result = {'checks': checks, 'pass': all(checks.values()), 'model_calls': 0,
        'observation': observation, 'rgb_quality': read(sim.records.path/'raw/brightness/preflight.png.json'),
        'capture_contract': 'Explicit completed Replicator render; zero simulation time advance; original front Hawk'}
    write(sim.records.path/'pre-model-gate.json', result)
    if not result['pass']:
        raise RuntimeError('E21_PREFLIGHT_FAILED: '+str([k for k,v in checks.items() if not v]))
