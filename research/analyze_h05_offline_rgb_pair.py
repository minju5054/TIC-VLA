"""Saved-only paired native/steering/context analysis. No model or simulator."""
import argparse
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json
from research.two_hz_preflight import read, digest, canonical
from research.control import validate_chunk, waypoint_command, rotation_wxyz
from research.h05_offline_rgb_pair import RULES, ORDER, shared_request


def direction(value, deadband):
    return 'LEFT' if value > deadband else 'RIGHT' if value < -deadband else 'NEAR_ZERO'


def opposed(a, b):
    return {a, b} == {'LEFT', 'RIGHT'}


def project(chunk, observation):
    local = np.column_stack([chunk, np.zeros(len(chunk))])
    return (local@rotation_wxyz(observation['quaternion_wxyz']).T + observation['position'])[:, :2]


def load_condition(out, condition, references, audit):
    source = out/'raw'/condition
    runtime = read(source/'inference-runtime.json')
    if not runtime['strict_checkpoint'] or runtime['simulation_app_started']:
        raise ValueError('Expected unchanged strict model service without SimulationApp')
    if read(source/'summary.json')['inferences'] != 14 or read(source/'service-closed.json')['returncode'] != 0:
        raise ValueError('Incomplete/failed condition; no qualification from partial data')
    files = sorted((source/'raw/requests').glob('*.json'))
    if len(files) != 14 or len(list((source/'raw/requests').glob('*.npy'))) != 14 or len(list((source/'raw').glob('claim_*.json'))) != 14:
        raise ValueError('Exactly 14 unique requests required')
    events, chunks = [], []
    state = None
    for i, f in enumerate(files):
        e = read(f); chunk = validate_chunk(np.load(f.with_suffix('.npy'), allow_pickle=False))
        if e['request_id'] != i+1 or shared_request(e) != references[i] or e['reference_request_sha256'] != canonical(references[i]):
            raise ValueError('Teacher-forced input changed')
        if e['command_applied'] or e['application'] is not None or e['new_physics_steps'] != 0:
            raise ValueError('Offline-only contract violated')
        if e['rgb_sha256'] != audit[i]['sha256'][condition] or digest(e['rgb_source']) != e['rgb_sha256']:
            raise ValueError('Wrong archived RGB input')
        np.testing.assert_array_equal(e['actual_predict_async_state_float32'], np.asarray(e['robot_state_input'], dtype=np.float32))
        if state != e['yaw_filter_before']:
            raise ValueError('Controller filter was not independently initialized/carried')
        command, state, index = waypoint_command(chunk, state, **read(out/'config.json')['controller'])
        np.testing.assert_allclose(command, e['counterfactual_controller_command'], rtol=0, atol=1e-12)
        if state != e['yaw_filter_after'] or index != e['lookahead_index']:
            raise ValueError('Original controller parity failed')
        stamps = [e[k]['monotonic_ns'] for k in ['replay_dispatch', 'inference_start', 'ready', 'replay_received']]
        if stamps != sorted(stamps):
            raise ValueError('Invalid replay/service clocks')
        dispatch = e['replay_dispatch']['monotonic_ns']/1e9
        if dispatch+1e-9 < e['nominal_dispatch_monotonic_s'] or dispatch+1e-9 < e['guard_dispatch_monotonic_s']:
            raise ValueError('Early dispatch')
        if i and dispatch-events[-1]['replay_dispatch']['monotonic_ns']/1e9+1e-9 < e['reference_interval_seconds']:
            raise ValueError('Catch-up burst')
        expected_ids = []
        for offset in [9., 6., 3., 0.]:
            available = [r['request_id'] for r in references[:i+1] if r['observation']['sim_time'] <= e['observation']['sim_time']-offset+1e-8]
            if available and available[-1] not in expected_ids:
                expected_ids.append(available[-1])
        if e['sampled_image_request_ids'] != expected_ids:
            raise ValueError('Image sampling policy changed')
        chunks.append(chunk); events.append(e)
    return events, chunks


def analyze(out):
    protocol = read(out/'protocol.json')
    if protocol['rules'] != RULES or read(out/'summary.json')['inferences'] != 28:
        raise ValueError('Frozen complete paired experiment required')
    raw_manifest = read(out/'raw-sha256.json')
    for p, expected in raw_manifest.items():
        if digest(out/p) != expected:
            raise ValueError('Raw input/output changed: '+p)
    references, audit = read(out/'reference-requests.json'), read(out/'rgb-audit.json')
    datasets = {c: load_condition(out, c, references, audit) for c in ORDER}
    sessions = {c: read(out/'raw'/c/'session.json') for c in ORDER}
    if sessions['ABSENT']['service_pid'] == sessions['PRESENT']['service_pid']:
        raise ValueError('Conditions did not use independent service PIDs')
    if read(out/'raw/ABSENT/service-closed.json')['wall_time_ns'] >= read(out/'raw/PRESENT/model-load-claim.json')['wall_time_ns']:
        raise ValueError('First service not closed before second initialized')
    rows, arrays = [], []
    for i in range(14):
        a, b = [datasets[c][0][i] for c in ORDER]
        ca, cb = [datasets[c][1][i] for c in ORDER]
        la, lb = ca[a['lookahead_index']], cb[b['lookahead_index']]
        va, wa = a['counterfactual_controller_command']; vb, wb = b['counterfactual_controller_command']
        da, db = [direction(float(v), RULES['native_direction_deadband_m']) for v in [la[1], lb[1]]]
        sa, sb = [direction(v, RULES['angular_direction_deadband_rad_s']) for v in [wa, wb]]
        native_delta = cb.astype(float)-ca; world_a = project(ca, a['observation']); world_b = project(cb, b['observation'])
        rmse = float(np.sqrt(np.mean(np.sum(native_delta**2, axis=1))))
        context = {'sampled_ids_equal': a['sampled_image_request_ids'] == b['sampled_image_request_ids'],
            'state_float32_equal': bool(np.array_equal(a['actual_predict_async_state_float32'], b['actual_predict_async_state_float32'])),
            'delay_equal': a['delay_seconds_input'] == b['delay_seconds_input'],
            'delay_reference_equal': a['delay_reference'] == b['delay_reference'],
            'generation_start_equal': a['reasoning_new_start_step'] == b['reasoning_new_start_step'],
            'completion_poll_equal': a['reasoning_completion_poll_step'] == b['reasoning_completion_poll_step'],
            'kv_available_equal': a['kv_cache_available'] == b['kv_cache_available']}
        row = {'request_id': i+1, 'reference_sim_time': references[i]['observation']['sim_time'],
            'visibility': audit[i]['visibility'], 'human_pixels': audit[i]['human_pixels'],
            'rgb_mae': audit[i]['rgb_difference']['whole']['rgb_mae_0_255'],
            'native_lookahead_A': la.tolist(), 'native_lookahead_B': lb.tolist(),
            'lookahead_B_minus_A': (lb.astype(float)-la).tolist(),
            'counterfactual_vw_A': [va, wa], 'counterfactual_vw_B': [vb, wb], 'angular_B_minus_A': wb-wa,
            'native_direction_A': da, 'native_direction_B': db, 'angular_direction_A': sa, 'angular_direction_B': sb,
            'native_direction_opposed': opposed(da, db), 'angular_direction_opposed': opposed(sa, sb),
            'raw_native_left_sign_agrees': bool(np.sign(la[1]) == np.sign(lb[1])),
            'raw_angular_sign_agrees': bool(np.sign(wa) == np.sign(wb)),
            'native_xy_point_rmse_m': rmse,
            'world_xy_point_rmse_m': float(np.sqrt(np.mean(np.sum((world_b-world_a)**2, axis=1)))),
            'significant_operational_difference': rmse >= .05 or abs(float(lb[1]-la[1])) >= .05 or abs(wb-wa) >= .05,
            'sampled_ids_A': a['sampled_image_request_ids'], 'sampled_ids_B': b['sampled_image_request_ids'],
            'state_float32_A': a['actual_predict_async_state_float32'], 'state_float32_B': b['actual_predict_async_state_float32'],
            'delay_A_s': a['delay_seconds_input'], 'delay_B_s': b['delay_seconds_input'],
            'generation_start_A': a['reasoning_new_start_step'], 'generation_start_B': b['reasoning_new_start_step'],
            'completion_poll_A': a['reasoning_completion_poll_step'], 'completion_poll_B': b['reasoning_completion_poll_step'],
            'kv_available_A': a['kv_cache_available'], 'kv_available_B': b['kv_cache_available'], 'context_parity': context}
        for c, e in [('A', a), ('B', b)]:
            row['action_call_'+c+'_wall_s'] = (e['ready']['monotonic_ns']-e['inference_start']['monotonic_ns'])/1e9
            row['dispatch_to_ready_'+c+'_wall_s'] = (e['ready']['monotonic_ns']-e['replay_dispatch']['monotonic_ns'])/1e9
            row['schedule_drift_'+c+'_s'] = e['schedule_drift_seconds']
        rows.append(row)
        arrays.append({'request_id': i+1, 'native_B_minus_A': native_delta.tolist(),
            'world_A': world_a.tolist(), 'world_B': world_b.tolist(), 'world_B_minus_A': (world_b-world_a).tolist()})
    primary = rows[11:14]
    directional = any(r['native_direction_opposed'] or r['angular_direction_opposed'] for r in primary)
    flags = ['HUMAN_RGB_CONDITIONED_RESPONSE_OBSERVED' if directional else 'NO_CLEAR_DIRECTIONAL_RESPONSE']
    if any(r['significant_operational_difference'] for r in rows[:10]):
        flags.append('PREVISIBLE_DIVERGENCE')
    if any(not all(r['context_parity'].values()) for r in rows):
        flags.append('ASYNC_CONTEXT_CONFOUNDED')
    flags.append('REPLAY_VARIABILITY_UNRESOLVED')
    summary = {'status': 'PASS', 'interpretation': flags, 'model_loads': 2, 'new_inferences': 28,
        'requests_per_condition': 14, 'navigation_trials': 0, 'physics_steps': 0, 'new_renders': 0, 'retries': 0,
        'service_pids': {c: s['service_pid'] for c, s in sessions.items()},
        'first_native_opposite_direction_request': next((r['request_id'] for r in rows if r['native_direction_opposed']), None),
        'first_angular_opposite_direction_request': next((r['request_id'] for r in rows if r['angular_direction_opposed']), None),
        'first_raw_native_sign_disagreement': next((r['request_id'] for r in rows if not r['raw_native_left_sign_agrees']), None),
        'first_operational_difference_request': next((r['request_id'] for r in rows if r['significant_operational_difference']), None),
        'context_mismatch_requests': {key: [r['request_id'] for r in rows if not r['context_parity'][key]] for key in rows[0]['context_parity']},
        'previsible_rgb_mae_range': [min(r['rgb_mae'] for r in rows[:10]), max(r['rgb_mae'] for r in rows[:10])],
        'raw_manifest_sha256': digest(out/'raw-sha256.json'), 'protocol_sha256': digest(out/'protocol.json'),
        'exact_consumed_KV_identity': 'NOT_AVAILABLE',
        'causal_limit': 'Archived sequences are not a human-only pixel intervention. Background render differences and any internal context differences are retained; no attribution of previous navigation failure and no repeatability claim.',
        'rows': rows}
    derived = out/'derived'; derived.mkdir(exist_ok=False)
    write_json(derived/'comparison.json', summary)
    write_json(derived/'native-and-world-differences.json', arrays)
    plot(derived, rows, datasets)
    for p, expected in raw_manifest.items():
        if digest(out/p) != expected:
            raise ValueError('Analysis changed raw evidence')
    print(summary['interpretation'], flush=True)
    return summary


def plot(out, rows, datasets):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ids = [r['request_id'] for r in rows]
    for c, color in [('A', '#2471a3'), ('B', '#ce5b23')]:
        axes[0].plot(ids, [r['native_lookahead_'+c][1] for r in rows], 'o-', c=color, label=c+' '+('ABSENT' if c == 'A' else 'H05 PRESENT'))
        axes[1].plot(ids, [r['counterfactual_vw_'+c][1] for r in rows], 'o-', c=color)
    for ax in axes:
        ax.axhline(0, c='.5', lw=.7); ax.axvspan(11.5, 14.4, alpha=.12, color='green')
        ax.axvspan(10.5, 11.5, alpha=.1, color='orange'); ax.grid(alpha=.2)
    axes[0].set(ylabel='Native lookahead left (m)', title='Offline RGB-sequence comparison; counterfactual steering only')
    axes[0].legend(); axes[1].set(ylabel='Angular command (rad/s)', xlabel='Common reference observation C'); axes[1].set_xticks(ids)
    fig.tight_layout(); fig.savefig(out/'steering-comparison.png', dpi=170); plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for rid, ax in zip([12, 13, 14], axes):
        for condition, color in [('ABSENT', '#2471a3'), ('PRESENT', '#ce5b23')]:
            event, chunk = datasets[condition][0][rid-1], datasets[condition][1][rid-1]
            ax.plot(chunk[:, 0], chunk[:, 1], c=color, label=condition)
            ax.scatter(*chunk[event['lookahead_index']], c=color, s=35)
        ax.scatter(0, 0, marker='+', c='black'); ax.grid(alpha=.2)
        ax.set(title=f'C{rid}: common observation body frame', xlabel='Forward (m)', ylabel='Left (m)', aspect='equal')
        ax.legend(fontsize=8)
    fig.suptitle('Individual native (30,2) curves; dots = each original controller lookahead; no executed motion')
    fig.tight_layout(); fig.savefig(out/'primary-native-curves.png', dpi=170); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    analyze(args.output_dir.resolve())
