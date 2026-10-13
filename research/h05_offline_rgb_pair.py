"""Two independent, teacher-forced C1-C14 inference sessions; no simulation.

Historical observation clocks/poses are inputs, not newly executed robot states.
Only PNG sequences differ at the replay boundary; actual internal state can differ.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import RunRecords, clocks, provenance, write_json
from research.loop import InferenceClient
from research.control import rotation_wxyz, validate_chunk, waypoint_command
from research.two_hz_preflight import read, digest, canonical, PRIMARY
from research.nova_occlusion import segmentation_stats, visibility_state
from research.nova_bright import image_stats, bright_pass

BASE = ROOT/PRIMARY
PREFLIGHT = ROOT/'outputs/ticvla-2hz-human-preflight-20261013-01'
OLD_SETUP = ROOT/'outputs/h05-avoidance-setup-20261013-01'
ORDER = ['ABSENT', 'PRESENT']
RULES = {
    'conditions': ORDER, 'requests_each': 14, 'maximum_new_inferences': 28,
    'independent_model_loads': 2, 'retries': False,
    'navigation_trials': 0, 'physics_steps': 0, 'renders': 0,
    'primary_window': [12, 13, 14], 'previsible_window': list(range(1, 11)), 'marginal': 11,
    'scheduling': 'After service ready: nominal_i=monotonic_epoch+(reference_sim_i-reference_sim_1); dispatch no earlier than nominal_i and previous_dispatch+reference_interval_i. Synchronous one outstanding request; no catch-up bursts. Record nominal/guard/actual/drift.',
    'teacher_forcing': 'Both receive exact same saved observation, instruction and previous_waypoints_text. Never update pose/history from new actions. Reset service and controller filter between conditions.',
    'rgb_changed_pixel': 'Any RGB channel absolute difference >10 on original uint8 PNG; distribution is mean absolute RGB channel difference per pixel',
    'previsible_significant_operational': 'XY point RMSE>=0.05 m OR abs lookahead-left difference>=0.05 m OR abs angular-command difference>=0.05 rad/s; descriptive threshold, not statistical significance',
    'native_direction_deadband_m': .02, 'angular_direction_deadband_rad_s': .03,
    'primary_response': 'Opposite LEFT/RIGHT native lookahead or angular-command signs, outside declared deadbands, at any C12-C14; report which channel and all confounds',
    'native_rmse': 'sqrt(mean(sum((B-A)**2,axis=XY))) over all 30 corresponding native points; no interpolation/stitching',
    'world_transform': 'World metres, original Hospital origin, +Z up. Body forward/left [dx,dy,0] mapped by observation wxyz body-to-world rotation and XYZ translation, then XY projection. Positive angular command is yaw-CCW LEFT. No native yaw/time.',
    'context_parity': 'Compare actual float32 six-value state sent to predict_async, delay/reference, sampled IDs, generation-start step, completion-poll step and cache availability. Exact consumed KV identity unavailable.',
    'interpretation': 'Exploratory archived RGB-sequence association only; stopped-render/background and stochastic/asynchronous context confounds; original asynchronous state not reproduced; repeatability unresolved',
}


def shared_request(event):
    return copy.deepcopy({k: event[k] for k in ['type', 'request_id', 'observation', 'instruction', 'previous_waypoints_text']})


def effective_deadline(epoch, reference_offset, previous_dispatch, interval):
    nominal = epoch + reference_offset
    return nominal, max(nominal, previous_dispatch+interval) if previous_dispatch is not None else nominal


def difference_stats(a, b, mask):
    delta = np.abs(a.astype(np.int16)-b.astype(np.int16))
    pixel = delta.mean(axis=2)
    significant = delta.max(axis=2) > 10
    def region(selected):
        if not np.any(selected):
            return None
        values = pixel[selected]
        return {'pixel_count': int(selected.sum()), 'rgb_mae_0_255': float(values.mean()),
                'pixel_mae_quantiles_0_25_50_75_90_95_99_100': np.percentile(values, [0,25,50,75,90,95,99,100]).tolist(),
                'fraction_any_channel_difference_gt10': float(significant[selected].mean())}
    return {'whole': region(np.ones(mask.shape, dtype=bool)), 'inside_human': region(mask), 'outside_human': region(~mask)}


def audit_inputs(cfg, historical_manifest):
    render = PREFLIGHT/'stage-b/renders'
    asset = read(BASE/'robot_asset.json')
    camera = next(c for c in asset['cameras'] if c['path'] == cfg['camera']['prim'])
    attributes = read(render/'camera-attributes.json')
    for saved, key in [('focalLength', 'focal_length'), ('horizontalAperture', 'horizontal_aperture'), ('verticalAperture', 'vertical_aperture')]:
        if abs(float(attributes[saved])-camera[key]) > 1e-9:
            raise ValueError('Camera intrinsics changed')
    clipping = [float(x) for x in attributes['clippingRange'].strip('()').split(',')]
    np.testing.assert_allclose(clipping, camera['clipping_range'], rtol=0, atol=1e-8)
    if read(render/'complete.json')['status'] != 'PASS':
        raise ValueError('Historical render incomplete')
    pairs, references, hashes = [], [], {}
    for rid in range(1, 15):
        paths = {'ABSENT': BASE/f'diagnostics/rgb_{rid:06d}.png',
            'PRESENT': render/f'H05/C{rid:02d}-rgb.png',
            'reference': BASE/f'raw/requests/request_{rid:06d}.json',
            'present_observation': render/f'H05/C{rid:02d}-observation.json',
            'mask': render/f'H05/C{rid:02d}-mask.png', 'semantic': render/f'H05/C{rid:02d}-semantic.npy',
            'labels': render/f'H05/C{rid:02d}-labels.json',
            'baseline_camera': BASE/f'raw/scene_observations/rgb_{rid:06d}.png.json'}
        for path in paths.values():
            sha = digest(path)
            if historical_manifest.get(str(path)) != sha:
                raise ValueError('Missing/changed historical input: '+str(path))
            hashes[str(path)] = sha
        event = read(paths['reference']); obs = event['observation']; present = read(paths['present_observation'])
        if (event['request_id'] != rid or present['request_id'] != rid or present['saved_observation'] != obs
                or present['observation_sim_time'] != obs['sim_time'] or event['instruction'] != cfg['instruction']
                or present['source_rgb_sha256'] != digest(paths['ABSENT']) or not present['render_valid']
                or present['timeline_playing'] or present['timeline_time'] != 0.):
            raise ValueError('Reference pose/state/time/instruction/render mismatch')
        body = np.eye(4); body[:3, :3] = rotation_wxyz(obs['quaternion_wxyz']).T; body[3, :3] = obs['position']
        expected = np.asarray(asset['camera_body_transform'])@body
        error = float(np.max(np.abs(expected-np.asarray(present['camera_world_matrix_row_vector']))))
        if error > 1e-5:
            raise ValueError('Saved full camera pose mismatch')
        np.testing.assert_allclose(read(paths['baseline_camera'])['camera_world_xyz'], expected[3, :3], rtol=0, atol=1e-5)
        images = {condition: np.array(Image.open(paths[condition])) for condition in ORDER}
        for image in images.values():
            if image.shape != (cfg['camera']['resolution'][1], cfg['camera']['resolution'][0], 3) or image.dtype != np.uint8 or image.std() < 1.:
                raise ValueError('Invalid archived RGB')
        stats, mask = segmentation_stats(np.load(paths['semantic'], allow_pickle=False), read(paths['labels']), 'preflight_human')
        if not np.array_equal(mask, np.array(Image.open(paths['mask'])) > 0):
            raise ValueError('Human mask/semantic mismatch')
        state = visibility_state(stats)
        state = 'CLEAR' if state == 'VISIBLE' else state
        if state != present['state'] or any(stats[k] != present[k] for k in stats):
            raise ValueError('Saved visibility mismatch')
        brightness = {c: image_stats(im) for c, im in images.items()}
        validity = {c: bright_pass(s, cfg['bright_validation']) for c, s in brightness.items()}
        np.testing.assert_allclose(brightness['PRESENT']['mean'], present['brightness']['mean'], rtol=0, atol=1e-9)
        if not validity['PRESENT'] or not present['brightness_pass']:
            raise ValueError('Invalid H05 brightness')
        references.append(shared_request(event))
        pairs.append({'request_id': rid, 'reference_sim_time': obs['sim_time'], 'visibility': state,
            'human_pixels': stats['human_visible_pixel_count'], 'human_bbox': stats['human_bbox_if_visible'],
            'camera_max_abs_error': error, 'paths': {k: str(p) for k, p in paths.items()},
            'sha256': {k: hashes[str(p)] for k, p in paths.items()},
            'reference_request_sha256': canonical(references[-1]),
            'rgb_difference': difference_stats(images['ABSENT'], images['PRESENT'], mask),
            'luminance': brightness, 'brightness_pass': validity})
    if [p['visibility'] for p in pairs] != ['HIDDEN']*10+['MARGINAL']+['CLEAR']*3:
        raise ValueError('Unexpected historical H05 visibility sequence')
    return pairs, references, hashes


def prepare(out):
    out.mkdir(parents=True, exist_ok=False)
    old = read(OLD_SETUP/'authorization.json')
    cfg = read(BASE/'runtime-config.json')
    selected = read(ROOT/'configs/research/ticvla_2hz_stationary_human_selected.json')
    if cfg != selected['baseline_config'] or digest(BASE/'runtime-config.json') != selected['baseline_config_sha256']:
        raise ValueError('Frozen 2 Hz baseline changed')
    pairs, references, inputs = audit_inputs(cfg, old['protected_sha256'])
    write_json(out/'rgb-audit.json', pairs)
    write_json(out/'reference-requests.json', references)
    write_json(out/'config.json', cfg)
    protected = dict(old['protected_sha256'])
    for directory in [ROOT/'outputs/nova-e16-h05-stationary-20261013-01', OLD_SETUP]:
        protected.update({str(p): digest(p) for p in directory.rglob('*') if p.is_file()})
    for p, expected in protected.items():
        if digest(p) != expected:
            raise ValueError('Historical protected evidence changed: '+p)
    sources = [*ROOT.joinpath('research').glob('*.py'), ROOT/'tests/test_h05_offline_rgb_pair.py',
               ROOT/'tests/test_research.py', ROOT/'scripts/isaac6_python.sh']
    write_json(out/'protocol.json', {'rules': RULES, 'order': ORDER, 'input_sha256': inputs,
        'reference_requests_sha256': digest(out/'reference-requests.json'), 'rgb_audit_sha256': digest(out/'rgb-audit.json'),
        'config_sha256': digest(out/'config.json'), 'weights': old['weights'],
        'baseline_config_sha256': selected['baseline_config_sha256'],
        'source_sha256': {str(p): digest(p) for p in sources}, 'protected_sha256': protected,
        'exact_command': ' '.join(sys.argv), 'provenance': provenance(ROOT), **clocks()})
    print('SAVED_INPUT_AUDIT_PASS; frozen 2 services / 28 calls; no inference yet', flush=True)


def verify(out):
    protocol = read(out/'protocol.json')
    if protocol['rules'] != RULES or protocol['order'] != ORDER:
        raise ValueError('Protocol changed')
    for group in ['input_sha256', 'source_sha256', 'protected_sha256']:
        for p, sha in protocol[group].items():
            if digest(p) != sha:
                raise ValueError('Frozen file changed: '+p)
    for name, key in [('reference-requests.json', 'reference_requests_sha256'), ('rgb-audit.json', 'rgb_audit_sha256'), ('config.json', 'config_sha256')]:
        if digest(out/name) != protocol[key]:
            raise ValueError('Frozen input snapshot changed')
    return protocol


def replay_condition(condition, out, cfg, references, pairs):
    records = RunRecords(out/'raw'/condition, {'mode': 'offline_teacher_forced', 'condition': condition,
        'config': cfg, 'protocol_sha256': digest(out/'protocol.json'), 'reference_clocks': 'historical, not replay wall time',
        'command_application': False, 'navigation_trials': 0, 'physics_steps': 0, 'renders': 0, **provenance(ROOT)})
    pngs = [Path(p['paths'][condition]).read_bytes() for p in pairs]
    write_json(records.path/'model-load-claim.json', {'condition': condition, 'retry_permitted': False, **clocks()})
    client = InferenceClient(cfg, records)
    try:
        epoch = time.monotonic(); previous = None; yaw_filter = None
        write_json(records.path/'session.json', {'service_pid': client.process.pid,
            'wall_schedule_epoch_monotonic_s': epoch, 'fresh_service': True, 'initial_controller_filter': None, **clocks()})
        for i, (reference, png) in enumerate(zip(references, pngs)):
            interval = reference['observation']['sim_time']-references[i-1]['observation']['sim_time'] if i else 0.
            offset = reference['observation']['sim_time']-references[0]['observation']['sim_time']
            nominal, guard = effective_deadline(epoch, offset, previous, interval)
            while True:
                remaining = guard-time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(.05, remaining))
            request = copy.deepcopy(reference)
            write_json(records.path/f'raw/claim_{i+1:02d}.json', {'request_id': i+1, 'max_requests': 14, **clocks()})
            dispatch = clocks(); previous = dispatch['monotonic_ns']/1e9
            response = client.predict(request, png)
            received = clocks()
            chunk = validate_chunk(np.asarray(response.pop('raw_action'), dtype=np.float32))
            before = yaw_filter
            command, yaw_filter, index = waypoint_command(chunk, yaw_filter, **cfg['controller'])
            records.request(i+1, chunk, {**request, **response, 'replay_dispatch': dispatch, 'replay_received': received,
                'nominal_dispatch_monotonic_s': nominal, 'guard_dispatch_monotonic_s': guard,
                'schedule_drift_seconds': previous-nominal, 'reference_interval_seconds': interval,
                'reference_request_sha256': canonical(reference), 'rgb_source': pairs[i]['paths'][condition],
                'rgb_sha256': pairs[i]['sha256'][condition],
                'actual_predict_async_state_float32': np.asarray(response['robot_state_input'], dtype=np.float32).tolist(),
                'counterfactual_controller_command': list(command), 'lookahead_index': index,
                'lookahead_native_fl_m': chunk[index].tolist(), 'yaw_filter_before': before, 'yaw_filter_after': yaw_filter,
                'application': None, 'command_applied': False, 'new_physics_steps': 0})
            print('OFFLINE_PREDICTION', condition, i+1, 'drift', round(previous-nominal, 4), flush=True)
        write_json(records.path/'summary.json', {'status': 'PASS', 'inferences': 14, 'command_applications': 0,
            'navigation_trials': 0, 'physics_steps': 0, 'renders': 0})
    finally:
        client.close()
        write_json(records.path/'service-closed.json', {'pid': client.process.pid,
            'returncode': client.process.returncode, 'wall_time_ns': clocks()['wall_time_ns']})
    if client.process.returncode != 0:
        raise RuntimeError('Inference service failed during shutdown; no next condition')


def run(out):
    verify(out)
    write_json(out/'experiment-claim.json', {'model_load_limit': 2, 'request_limit': 28, 'no_retry': True, **clocks()})
    cfg, references, pairs = read(out/'config.json'), read(out/'reference-requests.json'), read(out/'rgb-audit.json')
    if len(references) != 14 or len(pairs) != 14:
        raise ValueError('Exactly 14 common references required')
    try:
        for condition in ORDER:
            replay_condition(condition, out, cfg, references, pairs)
        write_json(out/'summary.json', {'status': 'PASS', 'model_loads': 2, 'inferences': 28,
            'navigation_trials': 0, 'physics_steps': 0, 'new_renders': 0, 'no_retry': True})
    except BaseException as exc:
        write_json(out/'failure.json', {'status': 'IMPLEMENTATION_FAILURE', 'error': repr(exc), **clocks()})
        traceback.print_exc()
        raise
    finally:
        write_json(out/'raw-sha256.json', {str(p.relative_to(out)): digest(p) for p in (out/'raw').rglob('*') if p.is_file()})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['prepare', 'run', 'verify'], required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    {'prepare': prepare, 'run': run, 'verify': verify}[args.phase](args.output_dir.resolve())
