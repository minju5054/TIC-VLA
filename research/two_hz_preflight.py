"""Saved-data geometry and immutable receipts for the model-free 2 Hz preflight.

World metres, Z up. West travel coordinates are s=entrance_x-x, l=center_y-y.
No native action conversion, model import, inference, or navigation execution.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json
from research.cart_geometry import segment_minimum

PRIMARY = 'outputs/nova-e16-hospital-lights-baseline-20261010-01'
SECONDARY = 'outputs/nova-e16-no-cart-15-repro-20261011-01'
RECEIPT = 'outputs/e16-zero-repeatability-plan-20261011-01/receipt.json'
RADIUS = .6069825421953869
HUMAN_RADIUS = .25
MARGIN = .05
SEPARATION = RADIUS + HUMAN_RADIUS + MARGIN
POLICY = {
    'scope': 'MODEL-FREE HUMAN PLACEMENT PREFLIGHT — NO HUMAN-CONDITIONED NAVIGATION',
    'goal_cutoff': 'First measured dense pose with XY distance strictly <2 m; include that row only',
    'near_cart_surface_distance_m': 3.,
    'meaningful_cart_robot_clearance_max_m': 2.,
    'passage_windows_progress_relative_to_cart_m': {'approach': [-3., -1.], 'local': [-1., 1.], 'post': [1., 3.]},
    'trend': 'OLS lateral offset versus west progress in approach window; extrapolate unchanged',
    'excursion_threshold_m': .20, 'persistence_progress_m': .50,
    'return': 'Absolute median post residual <=0.10 m and <=half local peak absolute residual',
    'passage_validity': 'Entire windows inside goal-cut west corridor; cart footprint intersects physical corridor; valid cross-sections; meaningful clearance',
    'grid_downstream_m': [3., 5., 7., 9.], 'grid_left_offsets_m': [0., .25, -.25],
    'left_definition': 'West forward=(-1,0), left=(0,-1); positive left decreases world Y',
    'human_root_yaw_rad': 0., 'human_orientation': 'Same Z-up source rest pose, root RotateZ=0; no anatomical facing inferred',
    'human_radius_m': HUMAN_RADIUS, 'robot_radius_m': RADIUS, 'static_margin_m': MARGIN,
    'required_center_separation_m': SEPARATION,
    'bypass': 'Primary-route connectors at person west progress +/-2.5 m; side strip at +/-1.1 m longitudinal, left/right offset 1.0 m from human; full path tested',
    'max_sample_spacing_m': .05,
    'structural_proxy': 'Enabled original collider world AABB intersecting robot sphere vertical range [0.70-(radius+margin),0.70+(radius+margin)]; floor excluded; continuous segment distance',
    'human_proxy_limit': '0.25 m capsule does not enclose outstretched rest-pose arms; separate full visual bounds placement overlap check',
    'visibility': {'fraction_min': .002, 'width_min_px': 20, 'height_min_px': 80, 'early_along_track_min_m': 5.},
    'render_sequence': 'All primary saved observations before first continuous path conflict, up to goal cutoff; exact full position/quaternion; stopped timeline',
    'valid_early': 'CLEAR, original brightness PASS, correct camera/asset/settings, >=5 m remaining measured path arc length to closest human station, before first conflict; corner observations allowed and straight-corridor flag reported',
    'ranking': ['maximum structural clearance of passing side', 'maximum valid early CLEAR remaining distance', 'minimum absolute corridor-left offset', 'ascending candidate ID'],
    'stop': 'At most 12 frozen sites; no relocation, relaxation, inference or navigation; select exactly one or NO_QUALIFIED_STATIONARY_HUMAN_SITE',
}


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def references():
    trials = read(RECEIPT)['trials']
    if len(trials) != 6 or [t['index'] for t in trials] != list(range(6)):
        raise ValueError('Actual six-trial receipt required')
    return [PRIMARY, SECONDARY] + ['outputs/' + t['run_id'] for t in trials]


def load_reference(path):
    from research.analyze_hospital_turn import nonfloor_contacts
    p = Path(path)
    cfg = read(p / 'runtime-config.json')
    with (p / 'robot_state.csv').open() as f:
        all_rows = list(csv.DictReader(f))
    xy = np.array([[float(r['x']), float(r['y'])] for r in all_rows])
    hits = np.flatnonzero(np.linalg.norm(xy - np.array(cfg['scene']['goal'][:2]), axis=1) < 2.)
    if not len(hits):
        raise ValueError('No historical goal entry: ' + str(p))
    cutoff = int(hits[0]); rows = all_rows[:cutoff + 1]; xy = xy[:cutoff + 1]
    times = np.array([float(r['sim_time']) for r in rows])
    if not np.isfinite(xy).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Bad measured trace')
    entries = [json.loads(s) for s in (p / 'raw/robot_contacts.jsonl').read_text().splitlines()]
    contacts = nonfloor_contacts(entries, cfg['robot']['prim'])
    if contacts:
        raise ValueError('Recorded nonfloor contact in successful reference: ' + str(p))
    gate = cfg['spatial_turn_gate']
    inside = (xy[:, 0] <= gate['exit_center_x_max_m']) & (xy[:, 1] >= gate['center_y_min_m']) & (xy[:, 1] <= gate['center_y_max_m'])
    if not np.any(inside[:-1] & inside[1:]):
        raise ValueError('No measured west gate passage')
    events = [read(f) for f in sorted((p / 'raw/requests').glob('*.json'))]
    if len(events) != 48 or read(p / 'summary.json')['status'] != 'PASS':
        raise ValueError('Incomplete successful source')
    for f in sorted((p / 'raw/requests').glob('*.npy')):
        a = np.load(f, allow_pickle=False)
        if a.shape != (30, 2) or not np.isfinite(a).all():
            raise ValueError('Bad historical native evidence')
    return {'path': str(p), 'cfg': cfg, 'rows': rows, 'xy': xy, 'times': times,
            'arc': np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))],
            'events': [e for e in events if e['observation']['sim_time'] <= times[-1]],
            'validation': {'source_run': str(p), 'full_rows': len(all_rows), 'goal_cut_rows': len(rows),
                'goal_entry_tick': int(rows[-1]['tick']), 'goal_entry_sim_time': float(times[-1]),
                'goal_entry_xy': xy[-1].tolist(), 'nonfloor_contacts': contacts,
                'west_gate_pass': True, 'request_count': len(events), 'config_canonical_sha256': canonical(cfg),
                'historical_metadata': read(p / 'metadata.json'),
                'simulation_runtime': read(p / 'simulation-runtime.json'),
                'inference_runtime': read(p / 'inference-runtime.json')}}


def point_path(xy, point, radius=0., arc=None):
    xy = np.asarray(xy, float); p = np.asarray(point, float)
    d = np.diff(xy, axis=0); denom = np.sum(d*d, axis=1)
    u = np.clip(np.sum((p - xy[:-1])*d, axis=1) / np.maximum(denom, 1e-30), 0., 1.)
    dist = np.linalg.norm(xy[:-1] + u[:, None]*d - p, axis=1)
    i = int(np.argmin(dist)); first = None
    for j in np.flatnonzero(dist < radius):
        a = xy[j] - p; v = d[j]; A = v@v; B = 2*a@v; C = a@a - radius*radius
        entry = 0. if C <= 0 else (-B - np.sqrt(max(0., B*B-4*A*C))) / (2*A)
        if 0 <= entry <= 1:
            first = {'segment': int(j), 'fraction': float(entry)}; break
    return {'distance_m': float(dist[i]), 'segment': i, 'fraction': float(u[i]),
            'point_xy': (xy[i]+u[i]*d[i]).tolist(),
            'closest_arc_m': float(arc[i]+u[i]*(arc[i+1]-arc[i])) if arc is not None else None,
            'conflict': bool(dist[i] < radius), 'first_conflict': first}


def rect_path(xy, lo, hi, radius=0.):
    center = (np.array(lo[:2])+hi[:2])/2; half = (np.array(hi[:2])-lo[:2])/2
    best = None
    for i, (a, b) in enumerate(zip(np.asarray(xy)[:-1]-center, np.asarray(xy)[1:]-center)):
        d, u = segment_minimum(a, b, half)
        if best is None or d-radius < best['clearance_m']:
            best = {'clearance_m': d-radius, 'segment': i, 'fraction': u}
    return best


def sample_path(vertices, spacing=.05):
    if spacing <= 0 or spacing > .05:
        raise ValueError('Spacing must be in (0, .05] m')
    parts = [np.linspace(a, b, max(1, int(np.ceil(np.linalg.norm(b-a)/spacing)))+1)[:-1]
             for a, b in zip(np.asarray(vertices)[:-1], np.asarray(vertices)[1:])]
    return np.vstack([*parts, np.asarray(vertices)[-1:]])


def excursion(progress, lateral, station, valid):
    u = np.asarray(progress)-station; lateral = np.asarray(lateral)
    windows = {k: (u >= a)&(u <= b) for k, (a, b) in POLICY['passage_windows_progress_relative_to_cart_m'].items()}
    result = {'classification': 'INSUFFICIENT_GEOMETRIC_EVIDENCE'}
    if any(np.sum(m) < 2 for m in windows.values()) or min(u) > -3. or max(u) < 3.:
        return {**result, 'reason': 'Incomplete approach/local/post windows before first goal entry'}
    total = (u >= -3)&(u <= 3)
    if not np.all(np.asarray(valid)[total]):
        return {**result, 'reason': 'Invalid or ambiguous structural cross-section in passage window'}
    coef = np.polyfit(u[windows['approach']], lateral[windows['approach']], 1)
    residual = lateral - np.polyval(coef, u); local = residual[windows['local']]
    positive, negative = float(max(local)), float(min(local)); peak = max(abs(positive), abs(negative))
    persistence = 0.
    for sign in [-1, 1]:
        start = None
        for x, v, eligible in zip(u, sign*residual, total):
            if eligible and v >= .2:
                if start is None: start = x
                persistence = max(persistence, float(x-start))
            else: start = None
    post = float(np.median(residual[windows['post']]))
    returns = abs(post) <= .1 and abs(post) <= peak/2
    observed = peak >= .2 and persistence >= .5 and returns
    return {'classification': 'LOCAL_LATERAL_EXCURSION_OBSERVED' if observed else 'PASSIVE_CORRIDOR_FOLLOWING_CONSISTENT',
            'approach_trend_slope': float(coef[0]), 'approach_trend_intercept_m': float(coef[1]),
            'pre_offset_m': float(np.median(lateral[windows['approach']])),
            'post_offset_m': float(np.median(lateral[windows['post']])),
            'maximum_positive_residual_m': positive, 'maximum_negative_residual_m': negative,
            'maximum_absolute_residual_m': peak, 'persistence_m': persistence,
            'post_median_residual_m': post, 'returned_toward_trend': bool(returns)}


def candidates(geometry):
    x0, y0 = geometry['entrance_x_m'], geometry['center_y_m']
    return [{'id': f'H{i+1:02d}', 'downstream_m': s, 'left_offset_m': l,
             'position_world_xyz': [x0-s, y0-l, 0.], 'yaw_rad': 0.}
            for i, (s, l) in enumerate((s, l) for s in POLICY['grid_downstream_m'] for l in POLICY['grid_left_offsets_m'])]


def bypass_vertices(reference, candidate, side):
    if side not in ['LEFT', 'RIGHT']: raise ValueError('Unknown side')
    x, y = candidate['position_world_xyz'][:2]; xy = reference['xy']
    def crossing(target):
        for a, b in zip(xy[:-1], xy[1:]):
            if a[0] >= target >= b[0] and a[0] != b[0]:
                return (a+(target-a[0])/(b[0]-a[0])*(b-a)).tolist()
        raise ValueError('No goal-cut reference connector')
    dy = -1. if side == 'LEFT' else 1.
    return np.array([crossing(x+2.5), [x+1.1, y+dy], [x-1.1, y+dy], crossing(x-2.5)])


def early_clear(row):
    b = row.get('human_bbox_if_visible')
    return bool(row['human_visible_fraction'] >= .002 and b and b[2]-b[0] >= 20 and b[3]-b[1] >= 80
                and row['remaining_along_track_m'] >= 5. and row['before_conflict']
                and row['render_valid'] and row['brightness_pass'])


def select(rows):
    passing = [r for r in rows if r['qualified']]
    return min(passing, key=lambda r: (-r['best_structural_clearance_m'], -r['early_clear_distance_m'], abs(r['left_offset_m']), r['id'])) if passing else None


def prepare(out):
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/'analysis-policy.json', POLICY)  # Before analyzing geometry or any candidate.
    commands = ['pwd','git status --short','git branch --show-current','git rev-parse HEAD','git remote -v',
                'git rev-parse origin/hardcase-probe','git log -5 --oneline',
                'git rev-list --left-right --count origin/hardcase-probe...HEAD']
    repo = {c: subprocess.check_output(c.split(), text=True).strip() for c in commands}
    if repo['git branch --show-current'] != 'hardcase-probe': raise ValueError('Wrong branch')
    write_json(out/'repository-start.json', {'commands': repo, 'fetch': 'git fetch --all --prune completed before prepare; no changed remote SHA', 'wall_time_ns': time.time_ns()})
    previous = read('outputs/e16-zero-repeatability-analysis-20261011-02/metadata.json')['source_sha256']
    old_raw = {p: h for p, h in previous.items() if '/outputs/' in p}
    mismatch = [p for p, h in old_raw.items() if not Path(p).is_file() or digest(p) != h]
    write_json(out/'historical-hash-verification.json', {'verified_count': len(old_raw), 'mismatches': mismatch,
               'manifest': 'outputs/e16-zero-repeatability-analysis-20261011-02/metadata.json',
               'manifest_sha256': digest('outputs/e16-zero-repeatability-analysis-20261011-02/metadata.json')})
    if mismatch: raise ValueError('Historical raw/source evidence changed')
    refs = [load_reference(p) for p in references()]
    base = refs[0]['cfg']
    for r in refs:
        for k in ['scene','robot','camera','instruction','seed','controller','simulation','hospital_lighting','inference']:
            if r['cfg'][k] != base[k]: raise ValueError('Reference condition mismatch: '+k)
    write_json(out/'reference-validation.json', [r['validation'] for r in refs])
    files = {Path(p) for p in subprocess.check_output(['git','ls-files'], text=True).splitlines() if Path(p).is_file()}
    for r in refs: files.update(p for p in Path(r['path']).rglob('*') if p.is_file())
    files.add(Path(RECEIPT))
    files.update(p for p in Path(base['inference']['base_model']).rglob('*') if p.is_file())
    files.add(Path(base['inference']['checkpoint']))
    write_json(out/'protected-sha256.json', {str(p): digest(p) for p in sorted(files)})
    write_json(out/'prepare-complete.json', {'status':'PASS','model_calls':0,'checkpoint_loads':0,'navigation_trials':0,'physics_steps':0,'wall_time_ns':time.time_ns()})
    print('PREPARE_PASS', out, flush=True)


def launch(out, name, command):
    folder = out/'attempts'/name; folder.mkdir(parents=True, exist_ok=False)
    sources = {str(p):digest(p) for p in sorted(Path('research').glob('*.py'))}
    write_json(folder/'start.json', {'argv':command,'sources':sources,'wall_time_ns':time.time_ns(),
               'local_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
               'dirty_diff_sha256':hashlib.sha256(subprocess.check_output(['git','diff','HEAD'])).hexdigest(),
               'model_calls':0,'checkpoint_loads':0,'navigation_trials':0})
    try:
        with (folder/'stdout.log').open('x') as f:
            result = subprocess.run(command, stdout=f, stderr=subprocess.STDOUT)
    except OSError as exc:
        write_json(folder/'end.json', {'exit_code':None,'launch_error':repr(exc),'wall_time_ns':time.time_ns()})
        raise
    write_json(folder/'end.json', {'exit_code':result.returncode,'wall_time_ns':time.time_ns()})
    print('ATTEMPT_COMPLETE',name,result.returncode,flush=True)
    return result.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['prepare','launch'], required=True)
    parser.add_argument('--output-dir', required=True); parser.add_argument('--attempt')
    parser.add_argument('command', nargs=argparse.REMAINDER); a=parser.parse_args()
    out=Path(a.output_dir)
    if a.phase=='prepare': prepare(out)
    else:
        command=a.command[1:] if a.command[:1]==['--'] else a.command
        if not a.attempt or not command: parser.error('Attempt and command required')
        sys.exit(launch(out,a.attempt,command))


if __name__=='__main__': main()
