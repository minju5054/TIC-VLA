"""Portable, Git-free preparation for a modified H05 runtime smoke test.

Only launch/run starts Isaac or inference. Prepare/verify/analyze never does.
Historical selections, authorization receipts and raw data are never rewritten.
"""
import argparse
import ast
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import traceback
import uuid

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
SELECTION = Path('configs/research/ticvla_2hz_stationary_human_selected.json')
BASELINE = Path('outputs/nova-e16-hospital-lights-baseline-20261010-01')
BUNDLE = Path('tools/h05_api_smoke')
SCOPE = 'MODIFIED_COPY_RUNTIME_SMOKE; not unchanged Isaac5 reproduction or historical H05 replication'


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def local_path(relative):
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Expected repository-relative path: ' + str(relative))
    resolved = (ROOT/path).resolve()
    if not resolved.is_relative_to(ROOT):
        raise ValueError('Path escapes repository: ' + str(relative))
    return resolved


def session_path(value):
    path = Path(value).resolve()
    if path.parent != ROOT/'outputs':
        raise ValueError('Session must be a direct child of this repository outputs/')
    return path


def clean_identity(value):
    """Compare semantic descriptors separately from explicitly refreshed hashes."""
    if isinstance(value, dict):
        return {k: clean_identity(v) for k, v in value.items() if not k.endswith('_sha256')}
    if isinstance(value, list):
        return [clean_identity(v) for v in value]
    return value


def current_config(gui=False):
    from research.hospital_episode import official_episode, validate_config
    from research.nova_source import source_audit
    selected = read(ROOT/SELECTION)
    if selected['candidate_id'] != 'H05':
        raise ValueError('H05 selection required')
    cfg = copy.deepcopy(selected['baseline_config'])
    current = official_episode()
    if clean_identity(cfg['official_episode']) != clean_identity(current):
        raise ValueError('Episode semantics changed; source identity refresh is insufficient')
    previous = cfg['official_episode']
    cfg['official_episode'] = current
    cfg['stationary_human'] = copy.deepcopy(selected['human'])
    cfg['stationary_human']['position'] = copy.deepcopy(selected['human']['start_position'])
    cfg['occlusion'] = {'visibility_rule': {
        'clearly_visible_min_fraction': .002, 'clearly_visible_min_bbox_width': 20,
        'clearly_visible_min_bbox_height': 80, 'hidden_max_pixels': 0}}
    cfg['simulation']['headless'] = not gui
    validate_config(cfg)
    historical_robot = read(ROOT/BASELINE/'robot_asset.json')
    source = source_audit()
    if clean_identity(source) != clean_identity(historical_robot['source']):
        raise ValueError('Robot asset, camera, episode or controller constants changed')
    tree = ast.parse((ROOT/'research/h05_stationary_run.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == 'SimulationApp']
    if len(calls) != 1 or not calls[0].args:
        raise ValueError('Inspect changed SimulationApp configuration before preparing')
    launch = ast.literal_eval(calls[0].args[0])
    if launch['renderer'] not in ('RayTracedLighting', 'RealTimePathTracing'):
        raise ValueError('Unreviewed renderer: ' + launch['renderer'])
    launch['headless'] = not gui
    if [launch['width'], launch['height']] != cfg['camera']['resolution']:
        raise ValueError('SimulationApp and model camera resolution differ')
    return cfg, launch, {'historical_episode_identity': previous,
                        'current_episode_identity': current,
                        'source_semantics_equal': True,
                        'render_equivalence_validated': False,
                        'historical_renderer': 'RayTracedLighting',
                        'current_robot_source': source}


def source_files():
    paths = set()
    for folder in ('research', 'DynaNav', 'ticvla'):
        paths.update(p for p in (ROOT/folder).rglob('*.py')
                     if not any(x in p.parts for x in ('__pycache__', 'logs', 'tmp', 'benchmark_results')))
    paths.update((ROOT/'scripts').glob('*.sh'))
    paths.update(p for p in (ROOT/BUNDLE).iterdir() if p.is_file())
    return sorted(paths)


def prepare(session, run_id, gui=False):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', run_id):
        raise ValueError('A simple run ID is required')
    run = ROOT/'outputs'/run_id
    if run.exists() or session.exists() or run == session:
        raise FileExistsError('Fresh session AND run directories required')
    cfg, launch, changes = current_config(gui)
    historical_runtime = read(ROOT/BASELINE/'simulation-runtime.json')
    controller = Path(historical_runtime['differential_controller_source'])
    sim_root = next(p for p in controller.parents if (p/'VERSION').is_file())
    if not (sim_root/'VERSION').read_text().startswith('6.0.1'):
        raise ValueError('Expected the preserved Isaac6.0.1 installation')
    inf = cfg['inference']
    if not (Path(inf['python']).parent/'VERSION').read_text().startswith('5.0.0'):
        raise ValueError('Expected the separate Isaac5 inference interpreter')
    external = {controller, sim_root/'VERSION', sim_root/'python.sh',
                Path(inf['python']), Path(inf['python']).parent/'VERSION', Path(inf['checkpoint'])}
    external.update(p for p in Path(inf['base_model']).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    print('Hashing current sources, reference data and existing model files; no model load.', flush=True)
    ext_hashes = {str(p): digest(p) for p in sorted(external)}
    expected_weights = {
        'base': (Path(inf['base_model'])/'model.safetensors', 'a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484'),
        'checkpoint': (Path(inf['checkpoint']), '376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036')}
    for path, expected in expected_weights.values():
        if ext_hashes[str(path)] != expected:
            raise ValueError('Released weight identity mismatch: ' + str(path))
    references = [ROOT/SELECTION, ROOT/'DynaNav/configs/benchmark_full.yaml',
                  ROOT/'results/ticvla-2hz-cart-human-preflight.json']
    references += [p for p in (ROOT/BASELINE).rglob('*') if p.is_file()]
    source_hashes = {str(p.relative_to(ROOT)): digest(p) for p in source_files()}
    reference_hashes = {str(p.relative_to(ROOT)): digest(p) for p in references}
    session.mkdir(parents=True, exist_ok=False)
    write(session/'config.json', cfg)
    receipt = {'schema': 1, 'scope': SCOPE, 'prepared_repo': str(ROOT),
        'created_utc': datetime.now(timezone.utc).isoformat(), 'run_id': run_id,
        'run_relative': str(run.relative_to(ROOT)), 'config_sha256': digest(session/'config.json'),
        'source_sha256': source_hashes, 'reference_sha256': reference_hashes,
        'external_sha256': ext_hashes, 'launch': launch, 'isaac6_root': str(sim_root),
        'controller_source': str(controller), 'controller_sha256': digest(controller),
        'isaac6_version': (sim_root/'VERSION').read_text().strip(),
        'weights': {k: {'path': str(p), 'sha256': h} for k, (p, h) in expected_weights.items()},
        'human_source_sha256': read(ROOT/'results/ticvla-2hz-cart-human-preflight.json')['selected']['human_source_sha256'],
        'changes': changes, 'simulation_started': False, 'model_calls': 0,
        'git_required': False, 'copy_policy': 'Relative code/reference paths; external model/runtime paths retained. Reprepare after code edits.'}
    write(session/'receipt.json', receipt)
    for name, phase in [('run.sh', 'launch'), ('analyze.sh', 'analyze'), ('verify.sh', 'verify')]:
        text = ('#!/usr/bin/env bash\nset -euo pipefail\n'
                'session="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n'
                'repo="$(cd "$session/../.." && pwd)"\n'
                'cd "$repo"\n'
                'exec python3 -B tools/h05_api_smoke/runner.py '+phase+' --session "$session"\n')
        with (session/name).open('x') as stream:
            stream.write(text)
    print('PREPARED_ONLY', session, flush=True)
    return receipt


def verify(session):
    receipt = read(session/'receipt.json')
    if receipt.get('schema') != 1 or receipt.get('scope') != SCOPE:
        raise ValueError('Not a modified-copy smoke receipt')
    actual_sources = {str(p.relative_to(ROOT)) for p in source_files()}
    if actual_sources != set(receipt['source_sha256']):
        raise ValueError('Prepared source inventory changed; prepare a new session')
    for group in ('source_sha256', 'reference_sha256'):
        for relative, expected in receipt[group].items():
            if digest(local_path(relative)) != expected:
                raise ValueError('Prepared file changed; prepare a new session: ' + relative)
    for absolute, expected in receipt['external_sha256'].items():
        if digest(absolute) != expected:
            raise ValueError('Prepared external file changed: ' + absolute)
    if digest(session/'config.json') != receipt['config_sha256']:
        raise ValueError('Prepared configuration changed')
    cfg, launch, _ = current_config(gui=not receipt['launch']['headless'])
    if cfg != read(session/'config.json') or launch != receipt['launch']:
        raise ValueError('Current-copy configuration mismatch')
    run = local_path(receipt['run_relative'])
    if run.parent != ROOT/'outputs' or run.name != receipt['run_id'] or run == session:
        raise ValueError('Invalid run destination')
    return receipt, cfg, run


def run(session):
    receipt, cfg, destination = verify(session)
    if destination.exists():
        raise FileExistsError('Run already exists; prepare a fresh session')
    write(session/'attempt.json', {'run_id': receipt['run_id'], 'scope': SCOPE,
                                 'started_utc': datetime.now(timezone.utc).isoformat()})
    from research.nova_evidence import NovaRecords, validate_continuous
    from research.records import clocks
    records = NovaRecords(destination, {'scope': SCOPE, 'config': cfg, 'mode': 'static',
        'scenario': 'H05 modified-copy smoke', 'command_variant': 'direct',
        'receipt_relative': str(session.relative_to(ROOT)), 'command_argv': sys.argv,
        'repository': str(ROOT), 'source_sha256': receipt['source_sha256'],
        'revision_kind': 'content_snapshot; Git unavailable/not required'})
    app, sim, code = None, None, 1
    try:
        from isaacsim import SimulationApp
        app = SimulationApp(receipt['launch'])
        print('API_SMOKE_SIMULATION_APP_READY', flush=True)
        import inspect
        import numpy as np
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.stationary_human import StationaryHuman
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        controller = Path(inspect.getfile(DifferentialController))
        runtime = {'python': sys.version, 'numpy': np.__version__,
            'isaac_version': (Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),
            'differential_controller_source': str(controller), 'controller_sha256': digest(controller),
            'renderer': receipt['launch']['renderer'], 'scope': SCOPE}
        write(destination/'simulation-runtime.json', runtime)
        if runtime['isaac_version'] != receipt['isaac6_version'] or runtime['controller_sha256'] != receipt['controller_sha256']:
            raise ValueError('Prepared simulator/controller identity changed')
        sim = HospitalLightsSimulation(cfg, records, actor_factory=lambda s: StationaryHuman(s, cfg['stationary_human']))
        sim.phase = 'h05_api_smoke'
        pre_model_gate(sim, cfg, receipt)
        for path, expected in receipt['source_sha256'].items():
            if digest(local_path(path)) != expected:
                raise ValueError('Source changed before model load: ' + path)
        print('API_SMOKE_PRE_MODEL_PASS', flush=True)
        result = run_model_loop(sim, cfg, records, 'static', evidence_analyzer=validate_continuous)
        write(destination/'summary.json', {'status': 'PASS', 'status_scope': 'Runtime only; not navigation success',
                                          'scope': SCOPE, **result})
        code = 0
    except Exception as exc:
        traceback.print_exc()
        write(destination/'failure.json', {'status': 'IMPLEMENTATION_FAILURE', 'error': repr(exc), **clocks()})
    finally:
        if sim:
            sim.close()
        write(destination/'raw-sha256.json', {str(p.relative_to(destination)): digest(p)
            for folder in ('raw', 'diagnostics') for p in (destination/folder).rglob('*') if p.is_file()})
        if app:
            app.close(exit_code=code)
    return code


def launch(session):
    receipt, _, destination = verify(session)
    if destination.exists() or (session/'attempt.json').exists():
        raise FileExistsError('This session was already attempted; prepare a fresh one')
    timeout = shutil.which('timeout')
    if not timeout:
        raise RuntimeError('GNU timeout is required for the bounded launch')
    command = [timeout, '--signal=INT', '--kill-after=30s', '900s', 'bash',
               str(ROOT/'scripts/isaac6_python.sh'), str(ROOT/BUNDLE/'runner.py'),
               'run', '--session', str(session)]
    env = os.environ.copy()
    env.update(TICVLA_ISAAC6_ROOT=receipt['isaac6_root'], CUDA_VISIBLE_DEVICES='0',
               PYTHONDONTWRITEBYTECODE='1')
    with (session/'console.log').open('xb') as log:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            for line in process.stdout:
                log.write(line); log.flush()
                sys.stdout.buffer.write(line); sys.stdout.buffer.flush()
            rc = process.wait()
        except KeyboardInterrupt:
            process.terminate()
            try:
                rc = process.wait(timeout=35)
            except subprocess.TimeoutExpired:
                process.kill(); rc = process.wait()
    completed = ((destination/'summary.json').is_file()
                 and read(destination/'summary.json').get('status') == 'PASS')
    write(session/'process-result.json', {'argv': command, 'returncode': rc,
        'runtime_summary_pass': completed, 'scope': SCOPE,
        'success_note': 'Exit status alone does not prove model inference, movement or navigation'})
    return 0 if rc == 0 and completed else (rc if rc > 0 else 1)


def analyze(session):
    receipt, _, destination = verify(session)
    if not (destination/'summary.json').is_file() or read(destination/'summary.json').get('status') != 'PASS':
        raise ValueError('No completed runtime PASS. Inspect this session console.log/failure.json first')
    for relative, expected in read(destination/'raw-sha256.json').items():
        if digest(destination/relative) != expected:
            raise ValueError('Raw result changed: ' + relative)
    from research.nova_evidence import load_nova
    from research.analyze_hospital_turn import nonfloor_contacts
    import numpy as np
    data = load_nova(destination)
    xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in data['robot']])
    goal = np.array(data['cfg']['scene']['goal'][:2])
    contacts = [json.loads(line) for line in (destination/'raw/robot_contacts.jsonl').read_text().splitlines()]
    structural = nonfloor_contacts(contacts, data['cfg']['robot']['prim'])
    human = data['cfg']['stationary_human']['prim'] + '/'
    human_contacts = [r for r in contacts if r['contacts'] and any(r[k].startswith(human) for k in ('collider0', 'collider1'))]
    result = {'scope': SCOPE, 'run_id': receipt['run_id'], 'runtime_pass': True,
        'predictions': len(data['events']), 'measured_motion': bool(np.max(np.linalg.norm(xy-xy[0], axis=1)) > .01),
        'maximum_displacement_from_start_m': float(np.max(np.linalg.norm(xy-xy[0], axis=1))),
        'closest_goal_distance_m': float(np.min(np.linalg.norm(xy-goal, axis=1))),
        'goal_entered_strict_2m': bool(np.min(np.linalg.norm(xy-goal, axis=1)) < 2.),
        'nonfloor_contact_spans': len(structural), 'human_contact_reports': len(human_contacts),
        'world_axes': 'Original Hospital origin, metres, +Z up; XY distances only',
        'limitations': ['Runtime smoke only; no general avoidance qualification',
                       'Modified renderer equivalence to the historical run has not been established']}
    output = destination/'derived/api-smoke'
    output.mkdir(parents=True, exist_ok=False)
    write(output/'summary.json', result)
    print(json.dumps(result, indent=2))
    return 0


def pre_model_gate(sim, cfg, receipt):
    # Retain the historical 17 checks; bind source identity to this modified copy.
    import numpy as np
    from research.two_hz_preflight import RADIUS, HUMAN_RADIUS
    import carb
    import omni.client
    from omni.physx import get_physx_scene_query_interface
    from pxr import UsdGeom, UsdPhysics
    from isaacsim.core.utils.semantics import get_labels
    from research.blind_corner_queries import Queries
    h = cfg['stationary_human']
    root = sim.stage.GetPrimAtPath(h['prim'])
    capsule = UsdGeom.Capsule(sim.stage.GetPrimAtPath(h['prim']+'/Collider'))
    actual = sim.pedestrian.snapshot()
    authored = np.asarray(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0))
    source_result, _, data = omni.client.read_file(h['asset'])
    asset_sha = hashlib.sha256(bytes(data)).hexdigest() if source_result == omni.client.Result.OK else None
    labels = get_labels(root)
    query = Queries.__new__(Queries)
    query.carb = carb
    query.q = get_physx_scene_query_interface()
    def floor(xy):
        hits = []
        def hit(item):
            if str(item.collision).startswith('/Root/'):
                hits.append((float(item.distance), list(item.position), list(item.normal), str(item.collision)))
            return True
        query.q.raycast_all(carb.Float3(*map(float, xy[:2]), .3), carb.Float3(0, 0, -1), .6, hit, True)
        if not hits:
            return False, ''
        _, position, normal, path = min(hits)
        return abs(position[2]) < .02 and normal[2] > .9, path
    query.floor = floor
    query.floor_prim = floor([19., 0.])[1]
    geometry = query.human(h['position'], h['radius_m'], h['yaw'])
    clearance = float(np.linalg.norm(np.array(sim.state()['position'][:2])-h['position'][:2])-RADIUS-HUMAN_RADIUS)
    before = sim.state()
    _, observation, _ = sim.capture('preflight.png')
    robot = read(sim.records.path/'robot_asset.json')
    baseline_robot = read(ROOT/PRIMARY/'robot_asset.json')
    lights = read(sim.records.path/'hospital_lighting_applied.json')
    checks = {
        'asset_hash': asset_sha == receipt['human_source_sha256'],
        'actual_asset_reference': read(sim.records.path/'human_asset.json')['asset'] == h['asset'],
        'authored_position': bool(np.allclose(authored[3, :3], h['position'], rtol=0, atol=1e-9)),
        'actual_PhysX_position': bool(np.allclose(actual['position'], h['position'], rtol=0, atol=1e-6)),
        'yaw_zero': bool(np.allclose(authored[:3, :3], np.eye(3), rtol=0, atol=1e-9)),
        'capsule': capsule.GetAxisAttr().Get() == 'Z' and capsule.GetRadiusAttr().Get() == .25 and capsule.GetHeightAttr().Get() == 1.2,
        'kinematic': bool(UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get()),
        'semantic_label': labels.get('class') == ['occluded_human'] and sim.segmentation is not None,
        'floor_support': geometry['floor_ok'], 'no_structure_overlap': not geometry['overlap_paths'],
        'no_robot_overlap': clearance > 0,
        'robot_camera_wheels': all(robot[k] == baseline_robot[k] for k in [
            'cameras', 'camera_body_transform', 'verified_drive_wheels', 'verified_wheel_base', 'controller_radius', 'controller_base']),
        'robot_source_matches_prepared_copy': robot['source'] == receipt['changes']['current_robot_source'],
        'spawn_valid': read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting_unchanged': lights == read(ROOT/PRIMARY/'hospital_lighting_applied.json'),
        'brightness': read(sim.records.path/'raw/brightness/preflight.png.json')['pass'],
        'capture_no_physics_advance': before['tick'] == observation['tick'] == 0 and before['position'] == observation['position'],
    }
    record = {'checks': checks, 'pass': all(checks.values()), 'model_calls': 0,
        'actual_human': actual, 'authored_world_row_matrix': authored.tolist(), 'human_asset_sha256': asset_sha,
        'semantic_labels': labels, 'geometry': geometry, 'initial_human_surface_clearance_m': clearance,
        'visibility': read(sim.records.path/'raw/visibility/preflight.png.json'),
        'note': 'One same-pose render gate; no inference, no candidate search, no bypass forcing; C1 need not be CLEAR'}
    write_json(sim.records.path/'pre-model-gate.json', record)
    if not record['pass']:
        raise RuntimeError('H05_PREREQUISITE_FAILED: '+str([k for k, v in checks.items() if not v]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('prepare', 'verify', 'launch', 'run', 'analyze'))
    parser.add_argument('--session')
    parser.add_argument('--run-id')
    parser.add_argument('--gui', action='store_true', help='Prepare a GUI run instead of the historical headless mode')
    args = parser.parse_args()
    if args.phase == 'prepare':
        run_id = args.run_id or ('h05-api-smoke-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
        session = session_path(args.session or ROOT/'outputs'/(run_id+'-setup'))
        prepare(session, run_id, args.gui)
        return 0
    if not args.session:
        parser.error('--session is required')
    session = session_path(args.session)
    if args.phase == 'verify':
        receipt, _, destination = verify(session)
        print(json.dumps({'status': 'PREPARATION_PASS', 'scope': SCOPE, 'run_id': receipt['run_id'],
                          'run_directory_exists': destination.exists(), 'simulation_started_by_verify': False}))
        return 0
    return {'launch': launch, 'run': run, 'analyze': analyze}[args.phase](session)


if __name__ == '__main__':
    raise SystemExit(main())
