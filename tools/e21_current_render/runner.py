"""Official E21 no-added-obstacle RGB and right-turn validation in the current renderer.

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
BASELINE = Path('outputs/blind-e21-baseline-20261010-01')
BUNDLE = Path('tools/e21_current_render')
SCOPE = 'E21_CURRENT_RENDER_RGB_AND_RIGHT_TURN_NO_ADDED_OBSTACLES; modified Isaac6 wrapper, not official Isaac5 reproduction'


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


def current_config(gui=False, max_v=1.5):
    from research.hospital_episode import official_episode, validate_config
    from research.nova_source import source_audit
    if max_v not in (1.5, .5):
        raise ValueError('Reviewed E21 speeds are 1.5 or 0.5 m/s')
    cfg = copy.deepcopy(read(ROOT/BASELINE/'metadata.json')['config'])
    current = official_episode('episode_21')
    if clean_identity(cfg['official_episode']) != clean_identity(current):
        raise ValueError('Official episode semantics changed')
    previous = cfg['official_episode']
    cfg['official_episode'] = current
    source = source_audit('episode_21')
    if clean_identity(cfg['nova_source']) != clean_identity(source):
        raise ValueError('Historical robot/camera/controller semantics changed')
    cfg['nova_source'] = source
    cfg['simulation']['headless'] = not gui
    cfg['controller']['max_v'] = max_v
    cfg['simulation']['predictions'] = round(48 * 1.5 / max_v)
    validate_config(cfg)
    tree = ast.parse((ROOT/'research/h05_stationary_run.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == 'SimulationApp']
    if len(calls) != 1:
        raise ValueError('Inspect changed current renderer launch')
    launch = ast.literal_eval(calls[0].args[0])
    if launch['renderer'] != 'RealTimePathTracing':
        raise ValueError('Current renderer differs from reviewed profile')
    launch['headless'] = not gui
    if [launch['width'], launch['height']] != cfg['camera']['resolution']:
        raise ValueError('Camera resolution differs')
    return cfg, launch, {'historical_episode_identity': previous,
        'current_episode_identity': current, 'source_semantics_equal': True,
        'current_robot_source': source, 'render_equivalence_validated': False,
        'added_obstacles': 0, 'speed_m_s': max_v,
        'baseline_gate': 'Existing E21 right-turn region and following observation; unchanged RGB rules'}


def source_files():
    paths = set()
    for folder in ('research', 'DynaNav', 'ticvla'):
        paths.update(p for p in (ROOT/folder).rglob('*.py')
                     if not any(x in p.parts for x in ('__pycache__', 'logs', 'tmp', 'benchmark_results')))
    paths.update((ROOT/'scripts').glob('*.sh'))
    paths.update(p for p in (ROOT/BUNDLE).glob('*.py'))
    return sorted(paths)


def prepare(session, run_id, gui=False, preflight=False, prerequisite=None, max_v=1.5, provenance_path=None):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', run_id):
        raise ValueError('A simple run ID is required')
    run = ROOT/'outputs'/run_id
    if run.exists() or session.exists() or run == session:
        raise FileExistsError('Fresh session AND run directories required')
    if provenance_path is None:
        raise ValueError('Source Git/content provenance is required')
    provenance = read(provenance_path)
    if not all(provenance.get(k) for k in ('local_sha', 'upstream_sha', 'new_sources')):
        raise ValueError('Incomplete source provenance')
    cfg, launch, changes = current_config(gui, max_v)
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
    references = [ROOT/'DynaNav/configs/benchmark_full.yaml']
    references += [p for p in (ROOT/BASELINE).rglob('*') if p.is_file()]
    source_hashes = {str(p.relative_to(ROOT)): digest(p) for p in source_files()}
    reference_hashes = {str(p.relative_to(ROOT)): digest(p) for p in references}
    prerequisite_hashes = {}
    if not preflight:
        if prerequisite is None:
            raise ValueError('A completed E21 camera/placement preflight is required before model preparation')
        prerequisite = local_path(prerequisite)
        if not read(prerequisite/'pre-model-gate.json')['pass'] or read(prerequisite/'summary.json').get('status_scope') != 'PREFLIGHT_ONLY; no model inference':
            raise ValueError('E21 camera/placement preflight did not pass')
        if read(prerequisite/'metadata.json')['config'] != cfg:
            raise ValueError('Preflight configuration differs')
        prerequisite_hashes = {str(p.relative_to(ROOT)): digest(p) for p in prerequisite.rglob('*') if p.is_file()}
    session.mkdir(parents=True, exist_ok=False)
    write(session/'config.json', cfg)
    receipt = {'schema': 1, 'scope': SCOPE, 'preflight_only': preflight, 'origin_provenance': provenance, 'prepared_repo': str(ROOT),
        'created_utc': datetime.now(timezone.utc).isoformat(), 'run_id': run_id,
        'run_relative': str(run.relative_to(ROOT)), 'config_sha256': digest(session/'config.json'),
        'source_sha256': source_hashes, 'reference_sha256': {**reference_hashes, **prerequisite_hashes},
        'external_sha256': ext_hashes, 'launch': launch, 'isaac6_root': str(sim_root),
        'controller_source': str(controller), 'controller_sha256': digest(controller),
        'isaac6_version': (sim_root/'VERSION').read_text().strip(),
        'weights': {k: {'path': str(p), 'sha256': h} for k, (p, h) in expected_weights.items()},
        'changes': changes, 'simulation_started': False, 'model_calls': 0,
        'git_required': False,
        'copy_policy': 'Relative code/reference paths; external model/runtime paths retained. Reprepare after code edits.'}
    write(session/'receipt.json', receipt)
    for name, phase in [('run.sh', 'launch'), ('analyze.sh', 'analyze'), ('verify.sh', 'verify')]:
        text = ('#!/usr/bin/env bash\nset -euo pipefail\n'
                'session="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n'
                'repo="$(cd "$session/../.." && pwd)"\n'
                'cd "$repo"\n'
                'exec python3 -B tools/e21_current_render/runner.py '+phase+' --session "$session"\n')
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
    cfg, launch, _ = current_config(gui=not receipt['launch']['headless'], max_v=read(session/'config.json')['controller']['max_v'])
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
        'scenario': 'Official E21 no added obstacles; room exit and right turn', 'command_variant': 'direct',
        'receipt_relative': str(session.relative_to(ROOT)), 'command_argv': sys.argv,
        'repository': str(ROOT), 'source_sha256': receipt['source_sha256'],
        'revision_kind': 'source Git SHA plus content snapshot of modified runtime copy',
        'origin_provenance': receipt['origin_provenance']})
    app, sim, code = None, None, 1
    try:
        from isaacsim import SimulationApp
        app = SimulationApp(receipt['launch'])
        print('API_SMOKE_SIMULATION_APP_READY', flush=True)
        import inspect
        import numpy as np
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from tools.e21_current_render.scene import pre_model_gate
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
        sim = HospitalLightsSimulation(cfg, records)
        sim.phase = 'e21_current_render'
        pre_model_gate(sim, cfg, receipt, ROOT/BASELINE)
        for path, expected in receipt['source_sha256'].items():
            if digest(local_path(path)) != expected:
                raise ValueError('Source changed before model load: ' + path)
        print('API_SMOKE_PRE_MODEL_PASS', flush=True)
        if receipt['preflight_only']:
            write(destination/'summary.json', {'status': 'PASS', 'status_scope': 'PREFLIGHT_ONLY; no model inference', 'model_calls': 0, 'navigation_steps_after_settling': 0})
        else:
            result = run_model_loop(sim, cfg, records, 'static', evidence_analyzer=validate_continuous)
            write(destination/'summary.json', {'status': 'PASS', 'status_scope': 'Runtime only; not navigation success', 'scope': SCOPE, **result})
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
    if receipt['preflight_only']:
        raise ValueError('Preflight has no navigation to analyze')
    from tools.e21_current_render.analysis import evaluate
    evaluate(destination, ROOT/BASELINE)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('prepare', 'verify', 'launch', 'run', 'analyze'))
    parser.add_argument('--session')
    parser.add_argument('--run-id')
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--prerequisite', help='Repository-relative completed preflight run')
    parser.add_argument('--provenance', help='Git/content origin receipt required for prepare')
    parser.add_argument('--max-v', type=float, choices=(1.5, .5), default=1.5)
    parser.add_argument('--gui', action='store_true', help='Prepare a GUI run instead of the historical headless mode')
    args = parser.parse_args()
    if args.phase == 'prepare':
        run_id = args.run_id or ('e21-current-render-' + ('preflight-' if args.preflight else 'run-')+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
        session = session_path(args.session or ROOT/'outputs'/(run_id+'-setup'))
        prepare(session, run_id, args.gui, args.preflight, args.prerequisite, args.max_v, args.provenance)
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
