"""Pure saved-evidence reveal rules. Nominal target times are derived, not native."""
import json
from pathlib import Path
import numpy as np
from research.nova_dynamic_geometry import remaining_curve, polyline_clearance
from research.nova_occlusion import visibility_state
from research.hospital_episode import digest
from research.records import write_json


def state(stats, rules):
    value = visibility_state(stats, rules)
    return 'CLEAR' if value == 'VISIBLE' else value


def path_distance(point, path):
    return polyline_clearance(path, point, 0.)


def coarse_positions(path, step=.5, band=4.):
    path = np.asarray(path)
    low = np.floor((path.min(0)-band)/step).astype(int)
    high = np.ceil((path.max(0)+band)/step).astype(int)
    return [[float(x*step), float(y*step), 0.] for x in range(low[0], high[0]+1)
            for y in range(low[1], high[1]+1)
            if path_distance([x*step, y*step], path) <= band]


def first_clear(rows):
    # One-based saved request IDs, ordered complete observations only.
    if [r['request_id'] for r in rows] != list(range(1, len(rows)+1)):
        raise ValueError('Visibility history must begin at C1 without gaps')
    first = next((i for i, r in enumerate(rows) if r['state'] == 'CLEAR'), None)
    if first is None:
        return None
    prior = rows[max(0, first-3):first]
    hidden = 0
    for r in reversed(rows[:first]):
        if r['state'] != 'HIDDEN':
            break
        hidden += 1
    return {'fresh_request_id': first+1, 'old_request_id': first if first else None,
            'three_prior_hidden': len(prior) == 3 and all(r['state'] == 'HIDDEN' for r in prior),
            'immediate_old_hidden': first > 0 and rows[first-1]['state'] == 'HIDDEN',
            'consecutive_hidden_history_count': hidden,
            'previous_visibility_history': rows[:first], 'first_clear': rows[first]}


def conflict_time(times, points, human, radius):
    """Earliest closed-circle entry on a continuous piecewise linear curve."""
    t, p, h = np.asarray(times, float), np.asarray(points, float), np.asarray(human, float)[:2]
    if len(t) != len(p) or len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('At least two increasing finite targets required')
    if not np.isfinite(np.r_[t, p.ravel(), h, radius]).all() or radius <= 0:
        raise ValueError('Invalid geometry')
    for i, (a, b) in enumerate(zip(p[:-1], p[1:])):
        d, offset = b-a, a-h
        c = float(offset@offset-radius**2)
        if c <= 1e-12:
            return {'time': float(t[i]), 'point': a.tolist(), 'segment': i, 'segment_fraction': 0.}
        aa, bb = float(d@d), float(2*offset@d)
        if aa <= 1e-24:
            continue
        disc = bb*bb-4*aa*c
        if disc < -1e-12:
            continue
        u = (-bb-np.sqrt(max(0., disc)))/(2*aa)
        if -1e-12 <= u <= 1+1e-12:
            u = float(np.clip(u, 0, 1))
            return {'time': float(t[i]+u*(t[i+1]-t[i])), 'point': (a+u*d).tolist(),
                    'segment': i, 'segment_fraction': u}
    return None


def temporal_metrics(run, fresh_id, human, config):
    fresh = run['events'][fresh_id-1]
    obs = fresh['observation']
    radius = config['robot_radius_m']+config['human_radius_m']
    out = {'robot_pose_at_reveal': obs, 'reveal_sim_time': obs['sim_time'],
           'baseline_application_sim_time': fresh['application']['sim_time'],
           'robot_current_clearance_at_reveal_m': float(np.linalg.norm(np.asarray(obs['position'])[:2]-human[:2])-radius),
           'old_min_clearance_m': None, 'conflict': None, 'reveal_lead_s': None,
           'switch_to_conflict_margin_s': None, 'old_horizon_end_s': None,
           'remaining_times': [], 'remaining_world_xy': []}
    if fresh_id < 2:
        return out
    old = run['events'][fresh_id-2]
    t, p = remaining_curve(run['worlds'][fresh_id-2], old['observation']['sim_time'], obs['sim_time'], config['native_waypoint_dt'])
    out['old_horizon_end_s'] = old['observation']['sim_time']+3.
    if t is None:
        return out
    hit = conflict_time(t, p, human, radius)
    out.update(old_min_clearance_m=polyline_clearance(p, human, radius), conflict=hit,
               remaining_times=t.tolist(), remaining_world_xy=p.tolist())
    if hit:
        out.update(reveal_lead_s=hit['time']-obs['sim_time'],
                   switch_to_conflict_margin_s=hit['time']-fresh['application']['sim_time'])
    return out


def classify(candidate, rows, run, config):
    result = dict(candidate)
    failures = []
    g = candidate['geometry']
    if not g['floor_ok'] or g['overlap_paths']:
        failures.append('E')
    if not candidate['bypass']['pass'] and not candidate['bypass'].get('not_evaluated'):
        failures.append('D')
    result.update(rendered=rows is not None, visibility=rows, selection_rank=None)
    if rows is not None:
        if len(rows) != len(run['events']):
            raise ValueError('Incomplete render cohort')
        event = first_clear(rows)
        if event is None:
            failures.append('F')
        else:
            result.update(event)
            if not event['immediate_old_hidden'] and event['old_request_id']:
                failures.append('B')
            if not event['three_prior_hidden']:
                failures.append('H')
            result.update(temporal_metrics(run, event['fresh_request_id'], candidate['position'], config))
            if result['robot_current_clearance_at_reveal_m'] <= 0:
                failures.append('C')
            if result['conflict'] is None:
                failures.append('A')
            elif result['reveal_lead_s'] <= 0 or result['switch_to_conflict_margin_s'] <= 0:
                failures.append('G')
    elif not failures:
        if 'prefilter' in candidate:
            failures.append('P_RAY' if candidate['prefilter']['temporal_pass_pair_count'] else 'P_TEMPORAL')
        else:failures.append('A_prefilter')
    result['failures'] = failures
    result['strict_qualified'] = rows is not None and not failures
    result['selection_rationale'] = config['ranking'] if result['strict_qualified'] else None
    result['primary_failure'] = next((f for f in ['E','D','F','B','H','C','A','G','P_TEMPORAL','P_RAY','A_prefilter'] if f in failures), None)
    return result


def ranked(candidates):
    ordered = sorted([c for c in candidates if c['strict_qualified']], key=lambda c: (
        c['switch_to_conflict_margin_s'], -c['first_clear']['human_visible_fraction'],
        -c['bypass']['clearance_m'], *c['position']))
    return [{**c, 'selection_rank': i+1} for i, c in enumerate(ordered)]


def freeze_selected(path, candidates):
    order = ranked(candidates)
    selected = order[0] if order else None
    write_json(Path(path), selected)  # Exclusive creation; never replaces a selection.
    return selected


def evidence_hashes(paths):
    files = [p for root in map(Path, paths) for p in (root.rglob('*') if root.is_dir() else [root]) if p.is_file()]
    return {str(p.resolve()): digest(p) for p in sorted(set(files))}


def validate_destination(output, sources):
    output = Path(output).resolve()
    for source in map(Path, sources):
        source = source.resolve()
        if source == output or source in output.parents:
            raise ValueError('Output must be separate from source evidence')
    if output.exists():
        raise FileExistsError(output)
    return output


def verify_hashes(hashes):
    changed = [p for p, value in hashes.items() if digest(p) != value]
    if changed:
        raise ValueError('Source changed: '+str(changed))


class CandidateNavigation:
    """UI-independent selection and saved-observation navigation."""
    def __init__(self, candidates):
        self.all = candidates
        self.items = candidates
        self.index = 0
        self.request_id = 1

    @property
    def current(self):
        return self.items[self.index] if self.items else None

    def filter(self, mode):
        self.items = [c for c in self.all if mode == 'all' or c['strict_qualified'] == (mode == 'strict')]
        self.index = 0
        return self.current

    def move(self, delta):
        if self.items:
            self.index = (self.index+delta) % len(self.items)
        return self.current

    def top(self):
        self.filter('all')
        order = ranked(self.all)
        target = order[0]['id'] if order else next((c['id'] for c in self.all if c.get('first_clear')), self.all[0]['id'])
        self.index = next(i for i,c in enumerate(self.items) if c['id'] == target)
        return self.current

    def jump(self, kind):
        if self.current:
            self.request_id = self.current.get('old_request_id' if kind == 'old' else 'fresh_request_id') or 1
        return self.request_id
