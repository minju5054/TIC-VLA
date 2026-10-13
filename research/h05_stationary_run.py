"""One authorized H05 trial; additive configuration adapter and pre-model gate.

No controller/model/scheduler implementation. All records are exclusive-created.
"""
import argparse
import copy
import hashlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import clocks, provenance, write_json
from research.two_hz_preflight import read, digest, references, PRIMARY, RADIUS, HUMAN_RADIUS, MARGIN
from research.hospital_episode import validate_config
from research.h05_stationary_analysis import RULES

SELECTION = ROOT/'configs/research/ticvla_2hz_stationary_human_selected.json'
SELECTION_SHA = '1b9d2beec527fdede71f3b33984cfd3e11a0d75dcf01da1ac4b1544d57da434e'
PREFLIGHT = ROOT/'outputs/ticvla-2hz-human-preflight-20261013-01'


def adapt(selected):
    cfg = copy.deepcopy(selected['baseline_config'])
    human = copy.deepcopy(selected['human'])
    human['position'] = copy.deepcopy(human['start_position'])
    cfg['stationary_human'] = human
    cfg['occlusion'] = {'visibility_rule': {
        'clearly_visible_min_fraction': .002, 'clearly_visible_min_bbox_width': 20,
        'clearly_visible_min_bbox_height': 80, 'hidden_max_pixels': 0}}
    validate_config(cfg)
    return cfg


def freeze(out, run_id):
    if Path(run_id).name != run_id or (ROOT/'outputs'/run_id).exists():
        raise ValueError('Fresh simple scientific run ID required')
    selected = read(SELECTION)
    if digest(SELECTION) != SELECTION_SHA or selected['candidate_id'] != 'H05' or selected['experiment_authorized']:
        raise ValueError('Historical H05 selection changed')
    baseline = ROOT/PRIMARY/'runtime-config.json'
    if digest(baseline) != selected['baseline_config_sha256'] or read(baseline) != selected['baseline_config']:
        raise ValueError('Frozen successful baseline mismatch')
    cfg = adapt(selected)
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/'expanded-config.json', cfg)
    protected = [f for p in [PREFLIGHT, *[ROOT/p for p in references()]] for f in p.rglob('*') if f.is_file()]
    protected += [SELECTION, ROOT/'configs/research/ticvla_2hz_cart_human_preflight.json',
        ROOT/'docs/TICVLA_2HZ_CART_AND_HUMAN_PREFLIGHT.md', ROOT/'results/ticvla-2hz-cart-human-preflight.json',
        ROOT/'outputs/nova-reveal-window-search-20261010-04/human_asset.json']
    tracked = subprocess.check_output(['git', 'ls-files', 'DynaNav'], cwd=ROOT, text=True).splitlines()
    protected += [ROOT/p for p in tracked if (ROOT/p).is_file()]
    protected += [p for p in Path(cfg['inference']['base_model']).rglob('*') if p.is_file()]
    protected += [Path(cfg['inference']['checkpoint'])]
    source = [*ROOT.joinpath('research').rglob('*.py'), *ROOT.joinpath('tests').glob('test_h05*.py'),
              ROOT/'scripts/isaac6_python.sh']
    runtime = read(ROOT/PRIMARY/'simulation-runtime.json')
    source += [Path(runtime['differential_controller_source'])]
    weights = {k: {'path': str(p), 'sha256': digest(p)} for k, p in [
        ('base', Path(cfg['inference']['base_model'])/'model.safetensors'),
        ('checkpoint', Path(cfg['inference']['checkpoint']))]}
    # The authoritative checkpoint identity is the prior frozen preflight manifest.
    old = read(PREFLIGHT/'protected-sha256.json')
    for w in weights.values():
        if old.get(w['path']) != w['sha256']:
            raise ValueError('Weight not identical to H05 preflight: '+w['path'])
    if weights['base']['sha256'] != 'a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484':
        raise ValueError('Unexpected base model')
    write_json(out/'authorization.json', {
        'experiment_authorized': True, 'scientific_runs_authorized': 1, 'scientific_retries': False,
        'run_id': run_id, 'selection_sha256': digest(SELECTION),
        'baseline_config_sha256': digest(baseline), 'expanded_config_sha256': digest(out/'expanded-config.json'),
        'human_source_sha256': read(ROOT/'results/ticvla-2hz-cart-human-preflight.json')['selected']['human_source_sha256'],
        'human_hash_origin': 'Prior H05 compact result / physical source receipt; selected file binds its pose and URL',
        'source_sha256': {str(p): digest(p) for p in source},
        'protected_sha256': {str(p): digest(p) for p in protected}, 'weights': weights,
        'rules': RULES, 'baseline_runtime': runtime, 'provenance': provenance(ROOT), **clocks()})


def verify(out):
    receipt = read(out/'authorization.json')
    for group in ['source_sha256', 'protected_sha256']:
        for p, expected in receipt[group].items():
            if digest(p) != expected:
                raise ValueError('Frozen file changed: '+p)
    if digest(out/'expanded-config.json') != receipt['expanded_config_sha256'] or receipt['rules'] != RULES:
        raise ValueError('Frozen configuration/evaluation changed')
    cfg = read(out/'expanded-config.json')
    if cfg != adapt(read(SELECTION)):
        raise ValueError('Adapter mismatch')
    return receipt, cfg


def pre_model_gate(sim, cfg, receipt):
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
            'source', 'cameras', 'camera_body_transform', 'verified_drive_wheels', 'verified_wheel_base', 'controller_radius', 'controller_base']),
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


def run(out):
    receipt, cfg = verify(out)
    if (out/'scientific-claim.json').exists():
        raise RuntimeError('Single scientific attempt already claimed; retry forbidden')
    from research.nova_evidence import NovaRecords, validate_continuous
    records = NovaRecords(ROOT/'outputs'/receipt['run_id'], {
        'mode': 'static', 'scenario': 'H05 stationary human', 'config': cfg, 'command_variant': 'direct',
        'command_argv': sys.argv, 'authorization_receipt': str(out/'authorization.json'),
        'authorization_sha256': digest(out/'authorization.json'), **provenance(ROOT)})
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RayTracedLighting', 'width': 1920, 'height': 1080})
    sim = None
    code = 1
    try:
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.stationary_human import StationaryHuman
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        sim = HospitalLightsSimulation(cfg, records, actor_factory=lambda s: StationaryHuman(s, cfg['stationary_human']))
        sim.phase = 'h05_stationary'
        controller = Path(inspect.getfile(DifferentialController))
        runtime = {'python': sys.version, 'numpy': np.__version__,
            'isaac_version': (Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),
            'differential_controller_source': str(controller), 'controller_sha256': digest(controller)}
        write_json(records.path/'simulation-runtime.json', runtime)
        if runtime != receipt['baseline_runtime']:
            raise RuntimeError('Simulator/controller runtime changed')
        pre_model_gate(sim, cfg, receipt)
        # Recheck sources after simulator initialization, before the only model load.
        for path, expected in receipt['source_sha256'].items():
            if digest(path) != expected:
                raise RuntimeError('Runtime source changed: '+path)
        write_json(out/'scientific-claim.json', {'run_id': receipt['run_id'], 'attempt': 1,
            'retry_permitted': False, 'claim_before_model_load': True, **clocks()})
        print('H05_PRE_MODEL_PASS_SINGLE_SCIENTIFIC_ATTEMPT_CLAIMED', flush=True)
        result = run_model_loop(sim, cfg, records, 'static', evidence_analyzer=validate_continuous)
        write_json(records.path/'summary.json', {'status': 'PASS', 'status_scope': 'Runtime only; navigation evaluated separately', **result})
        code = 0
    except Exception as exc:
        traceback.print_exc()
        write_json(records.path/'failure.json', {'status': 'IMPLEMENTATION_FAILURE', 'error': repr(exc),
            'scientific_attempt_claimed': (out/'scientific-claim.json').exists(), **clocks()})
    finally:
        if sim:
            sim.close()
        # Immutable inputs for saved-only analysis, including failure prefixes.
        write_json(records.path/'raw-sha256.json', {str(p.relative_to(records.path)): digest(p)
            for folder in ['raw', 'diagnostics'] for p in (records.path/folder).rglob('*') if p.is_file()})
        app.close(exit_code=code)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['freeze', 'run', 'verify'], required=True)
    parser.add_argument('--receipt-dir', type=Path, required=True)
    parser.add_argument('--run-id')
    args = parser.parse_args()
    if args.phase == 'freeze':
        freeze(args.receipt_dir.resolve(), args.run_id)
    elif args.phase == 'run':
        run(args.receipt_dir.resolve())
    else:
        verify(args.receipt_dir.resolve())
        print('FROZEN_SOURCES_AND_PRIOR_ARTIFACTS_PASS')
