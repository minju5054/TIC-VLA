"""One scoped C26 speed diagnostic; existing runner/controller remain unchanged."""
import argparse
import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.hospital_episode import digest, validate_config
from research.records import write_json, provenance
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination

SCOPE = 'ONE_C26_MAX_V_1_2_RUN_DESPITE_BYPASS_FAILURE'
EXPECTED_DIFF = [{'path': 'controller.max_v', 'old': 1.5, 'new': 1.2}]


def config_diff(old, new, prefix=''):
    if isinstance(old, dict) and isinstance(new, dict):
        result = []
        for k in sorted(old.keys() | new.keys()):
            path = prefix+'.'+k if prefix else k
            if k not in old or k not in new:
                result.append(dict(path=path, old=old.get(k), new=new.get(k),
                                   old_present=k in old, new_present=k in new))
            else:
                result.extend(config_diff(old[k], new[k], path))
        return result
    return [] if old == new else [{'path': prefix, 'old': old, 'new': new}]


def validate_ablation(old, new, frozen):
    if config_diff(old, new) != EXPECTED_DIFF:
        raise ValueError('Exactly controller.max_v 1.5 -> 1.2 is required')
    if old['stationary_cart'] != frozen or frozen.get('intervention') != 'EXACT_C26':
        raise ValueError('Original frozen C26 cart required')
    if old['simulation']['predictions'] != 48:
        raise ValueError('48 prediction budget required')
    validate_config(new)


def validate_authorization(auth, run_id, preflight, reference, config_sha):
    expected = dict(scope=SCOPE, run_id=run_id, reference_run=str(reference),
                    reference_metadata_sha256=digest(reference/'metadata.json'),
                    preflight_freeze_sha256=digest(preflight/'freeze.json'),
                    config_sha256=config_sha, max_v=1.2, predictions=48)
    if any(auth.get(k) != v for k, v in expected.items()) or not auth.get('user_instruction'):
        raise ValueError('New speed/run/source-bound user authorization required')
    physical = json.loads((preflight/'physical.json').read_text())
    summary = json.loads((preflight/'summary.json').read_text())
    required = ['floor_support', 'no_static_overlap', 'no_initial_collision', 'baseline_intersects_inflated_cart']
    if (physical['pass'] or physical['checks']['local_bypass'] or
            not all(physical['checks'][k] for k in required) or summary['cart_visible_at_C1'] or
            set(summary['stop_reasons']) != {'PHYSICAL_PLACEMENT_INVALID', 'NO_FEASIBLE_BYPASS'}):
        raise ValueError('Original bypass-only failure must remain unchanged')


def freeze(reference, preflight, out, run_id, instruction_file, startup_result):
    if Path(run_id).name != run_id or not run_id:
        raise ValueError('Run ID must be a directory basename')
    validate_destination(out, [reference, preflight])
    if (ROOT/'outputs'/run_id).exists():
        raise FileExistsError('Run already exists; no retry')
    if json.loads(startup_result.read_text()).get('pass') is not True:
        raise ValueError('Current Isaac 6 startup PASS required')
    old = json.loads((reference/'metadata.json').read_text())['config']
    cfg = copy.deepcopy(old)
    cfg['controller']['max_v'] = 1.2
    frozen = json.loads((preflight/'freeze.json').read_text())
    validate_ablation(old, cfg, frozen)
    runtime = json.loads((reference/'inference-runtime.json').read_text())
    weights = [Path(cfg['inference']['base_model'])/'model.safetensors', Path(cfg['inference']['checkpoint'])]
    expected = runtime['artifact_sha256']
    if [digest(p) for p in weights] != [expected['base'], expected['checkpoint']]:
        raise ValueError('Model weights changed')
    out.mkdir(parents=True)
    write_json(out/'config.json', cfg)
    auth = dict(scope=SCOPE, run_id=run_id, reference_run=str(reference),
                reference_metadata_sha256=digest(reference/'metadata.json'),
                preflight_freeze_sha256=digest(preflight/'freeze.json'),
                config_sha256=digest(out/'config.json'), max_v=1.2, predictions=48,
                user_instruction=instruction_file.read_text(), user_instruction_sha256=digest(instruction_file))
    validate_authorization(auth, run_id, preflight, reference, auth['config_sha256'])
    write_json(out/'authorization.json', auth)
    code = evidence_hashes([ROOT/'research', ROOT/'scripts/isaac6_python.sh', ROOT/'DynaNav'])
    code = {p: h for p, h in code.items() if Path(p).suffix in ('.py', '.sh')}
    write_json(out/'receipt.json', dict(authorized_run_id=run_id, config_sha256=auth['config_sha256'],
        physical_pass=False, visibility_diagnostic_only=True, explicit_user_authorization=auth,
        reference_run=str(reference), preflight_dir=str(preflight), exact_config_difference=EXPECTED_DIFF,
        code_sha256=code, source_evidence_sha256=evidence_hashes([
            reference, preflight, *weights, out/'authorization.json', startup_result, instruction_file]),
        single_run_no_retry=True, **provenance(ROOT)))


def execute(folder):
    receipt = json.loads((folder/'receipt.json').read_text())
    reference, preflight = (Path(receipt[k]) for k in ['reference_run', 'preflight_dir'])
    cfg = json.loads((folder/'config.json').read_text())
    validate_authorization(receipt['explicit_user_authorization'], receipt['authorized_run_id'],
                           preflight, reference, digest(folder/'config.json'))
    validate_ablation(json.loads((reference/'metadata.json').read_text())['config'], cfg,
                      json.loads((preflight/'freeze.json').read_text()))
    verify_hashes(receipt['source_evidence_sha256'])
    # The original exclusive run reservation, asset hashes and physics/model path apply.
    from research.e16_cart_run import execute as original_execute
    original_execute(folder)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--phase', choices=['freeze', 'run'], required=True)
    p.add_argument('--receipt-dir', type=Path, required=True)
    for name in ['reference-run', 'preflight-dir', 'instruction-file', 'startup-result']:
        p.add_argument('--'+name, type=Path)
    p.add_argument('--run-id')
    a = p.parse_args()
    if a.phase == 'freeze':
        freeze(a.reference_run.resolve(), a.preflight_dir.resolve(), a.receipt_dir.resolve(),
               a.run_id, a.instruction_file.resolve(), a.startup_result.resolve())
    else:
        execute(a.receipt_dir.resolve())
