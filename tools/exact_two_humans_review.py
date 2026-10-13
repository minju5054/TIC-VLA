"""Saved-only review of the exact C10/C24 run; preserves frozen analysis.

The experiment's individual semantic-label matcher produced empty masks even
when the combined human mask was nonempty. Do not interpret these as absence.
This supplement uses measured poses/contacts independently of visibility.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(path.read_text())


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def visibility_quality(combined, individual):
    bad = []
    for rid, total, people in zip(range(1, len(combined)+1), combined, individual):
        if total['human_visible_pixel_count'] != sum(p['human_visible_pixel_count'] for p in people['people'].values()):
            bad.append(rid)
    return {'status': 'UNUSABLE' if bad else 'COUNTS_CONSISTENT_ONLY',
            'combined_individual_count_mismatch_requests': bad,
            'note': 'Empty per-person masks are not evidence of absence. Counts alone do not verify identity. Full semantic ID rasters/maps were not archived; no individual mask is reconstructed.'}


def evaluate(run, output):
    protected = read(run/'raw-sha256.json')
    # Also preserve every prior derived artifact, including the flawed summary.
    protected.update({str(p.relative_to(run)): digest(p) for p in (run/'derived').rglob('*') if p.is_file()})
    for p, h in protected.items():
        if digest(run/p) != h:
            raise ValueError('Changed evidence: '+p)
    cfg = read(run/'metadata.json')['config']
    with (run/'robot_state.csv').open() as stream:
        robot = list(csv.DictReader(stream))
    with (run/'raw/humans_state.csv').open() as stream:
        actors = list(csv.DictReader(stream))
    t = np.array([float(r['sim_time']) for r in robot])
    xy = np.array([[float(r[k]) for k in ('x', 'y')] for r in robot])
    contacts = [json.loads(s) for s in (run/'raw/robot_contacts.jsonl').read_text().splitlines()]
    events = [read(p) for p in sorted((run/'raw/requests').glob('*.json'))]
    combined = [read(run/'raw/visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    individual = [read(run/'raw/individual_visibility'/f'rgb_{e["request_id"]:06d}.png.json') for e in events]
    people = {}
    for h in cfg['fixed_humans']:
        samples = [r for r in actors if r['id'] == h['id']]
        if len(samples) != len(robot) or any(a['tick'] != b['tick'] or a['sim_time'] != b['sim_time'] for a, b in zip(samples, robot)):
            raise ValueError('Human/robot samples differ')
        xyz = np.array([[float(r[k]) for k in ('x', 'y', 'z')] for r in samples])
        quaternion = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in samples])
        motion = float(np.linalg.norm(xyz-xyz[0], axis=1).max())
        turn = float(np.abs(quaternion-quaternion[0]).max())
        radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']+h['radius_m']
        forward = np.array([np.cos(h['reference_tangent_rad']), np.sin(h['reference_tangent_rad'])])
        passed = np.flatnonzero((xy-xyz[0, :2])@forward > radius)
        reports = [e for e in contacts if e['contacts'] and any(e[k] == h['prim'] or e[k].startswith(h['prim']+'/') for k in ('collider0', 'collider1'))]
        points = [p for e in reports for p in e['contacts']]
        people[h['id']] = {
            'exact_reference_xy_m': h['reference_observation']['pose'][:2],
            'measured_xy_error_m': float(np.linalg.norm(xyz[0, :2]-h['reference_observation']['pose'][:2])),
            'maximum_motion_m': motion, 'maximum_quaternion_component_change': turn,
            'measured_passage_sim_s': float(t[passed[0]]) if len(passed) else None,
            'contact_free_passage': bool(len(passed) and not reports and motion <= 1e-6 and turn <= 1e-7),
            'contact_reports': len(reports),
            'first_contact_sim_s': min((e['sim_time'] for e in reports), default=None),
            'maximum_contact_impulse_norm_Ns': max((float(np.linalg.norm(p['impulse'])) for p in points), default=0.),
            'minimum_contact_separation_m': min((p['separation'] for p in points), default=None),
            'visibility': 'UNAVAILABLE_FROM_INDIVIDUAL_AUTOMATIC_LOG'}
    distances = np.linalg.norm(xy-np.array(cfg['scene']['goal'][:2]), axis=1)
    approach = [{k: e[k] for k in ('request_id', 'controller_command', 'old_controller_command', 'reasoning_new_start_step', 'reasoning_completion_poll_step')}
                | {'observation_sim_s': e['observation']['sim_time'], 'application_sim_s': e['application']['sim_time']}
                for e in events if 18 <= e['request_id'] <= 24]
    result = {'status': 'FAIL_THIS_SCENARIO' if any(p['contact_reports'] for p in people.values()) or not np.any(distances < 2.) else 'REVIEW_REQUIRED',
              'run_dir': str(run), 'predictions': len(events), 'people': people,
              'goal_entered_strict_2m': bool(np.any(distances < 2.)),
              'closest_goal_distance_m': float(distances.min()), 'final_goal_distance_m': float(distances[-1]),
              'visibility_log': visibility_quality(combined, individual), 'approach_commands': approach,
              'supersedes': 'Only visibility-dependent interpretations in derived/exact-two-humans/summary.json. Its first_visible/first_clear nulls do not establish absence; C10 passed_and_stationary=false is invalid. Original file retained.',
              'coordinates': 'Original Hospital origin, world XY metres, +Z up. Passage uses world robot-minus-human XY projected onto the original adjacent-observation tangent. Human quaternions xyzw. Command v is m/s; w is rad/s, positive left. Times are simulation seconds since navigation start, not wall time or absolute Isaac time.',
              'limitations': 'One 48-prediction simulation; no causal avoidance claim. Conservative passage plane is not full-body swept clearance. RGB identity is reviewed manually; no automated first per-person visibility time is claimed.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    (output/'protected-input-sha256.json').write_text(json.dumps(protected, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    fig, axes = plt.subplots(2, 3, figsize=(15, 7))
    for ax, rid in zip(axes.flat, (1, 8, 12, 20, 24, 48)):
        event = next(e for e in events if e['request_id'] == rid)
        with Image.open(run/event['rgb_observation_reference']) as image:
            ax.imshow(image)
        ax.set_title(f'Observation C{rid} | sim {event["observation"]["sim_time"]:.3f} s')
        ax.axis('off')
    fig.suptitle('Exact original C10 + C24 | actual saved RGB (no rerender)', fontsize=16)
    fig.tight_layout()
    fig.savefig(output/'rgb-review.png', dpi=140)
    plt.close(fig)
    if any(digest(run/p) != h for p, h in protected.items()):
        raise ValueError('Evidence changed during review')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    evaluate(args.run_dir.resolve(), args.output_dir.resolve())
