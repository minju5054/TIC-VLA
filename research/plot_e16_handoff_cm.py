"""Display-only cm closeups of the already audited E16 handoff examples.

All polylines keep their observation-to-world anchors. Display transform only:
q_cm = 100 * (p_world_m - B_world_m), with no rotation, world +Z up.
B is the measured application pose. Native outputs have no yaw or timestamps;
the green nominal point is the prior audit's interpolation, not an observation.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator
import numpy as np

from research.audit_e16_wheelchair_handoffs import (
    digest, read, snapshot, to_world, verify_snapshot, write,
)


def render(audit, out, root):
    verdict = read(audit/'final-verdict.json')
    source = root/'outputs'/verdict['source_run']
    for protected in (source, audit):
        if out == protected or out.is_relative_to(protected) or protected.is_relative_to(out):
            raise ValueError('New output must be separate from source and prior audit')
    if out.exists():
        raise FileExistsError('New output directory required')
    git = lambda *args: subprocess.check_output(['git', *args], cwd=root, text=True).strip()
    if git('branch', '--show-current') != 'TIC-VLA-TEST':
        raise ValueError('Only TIC-VLA-TEST authorized')
    before = read(audit/'source-before.json')
    verify_snapshot(source, before)
    audit_before = snapshot(audit)
    metrics = read(audit/'pair-metrics.json')
    assert [m['fresh_id'] for m in metrics] == list(range(2, 91))
    with (source/'robot_state.csv').open() as f:
        robot = list(csv.DictReader(f))
    xy = np.array([[float(row[k]) for k in ('x', 'y')] for row in robot])
    examples = []
    for name in ('C10', 'C24', 'control'):
        rid = verdict['representatives'][name]['fresh_id']
        m = metrics[rid-2]
        old, fresh = [read(source/'raw/requests'/f'request_{i:06d}.json') for i in (rid-1, rid)]
        world = [to_world(np.load(source/'raw/requests'/f'request_{e["request_id"]:06d}.npy', allow_pickle=False),
                          e['agent_pose_at_observation']) for e in (old, fresh)]
        B = np.array(m['boundary_pose'][:2])
        nominal = np.array(m['nominal_boundary_xy'])
        assert np.allclose(B, fresh['application']['pose'][:2], rtol=0, atol=1e-12)
        assert abs(np.linalg.norm(nominal-B)-m['boundary_position_m']) < 1e-12
        a, b = fresh['observation']['tick'], fresh['application']['tick']
        cm = lambda points: 100*(np.asarray(points)-B)
        examples.append(dict(name=name, metrics=m, old=cm(world[0]), fresh=cm(world[1]),
            committed=cm(xy[old['application']['tick']:b+1]), pending=cm(xy[a:b+1]),
            observation=cm(fresh['observation']['pose'][:2]), nominal=cm(nominal),
            connector=cm([B, world[1][m['first_native_future_index']]]),
            display_origin_world_xy_cm=(100*B).tolist()))
    out.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'svg.fonttype': 'none'})
    legend = [Line2D([], [], color='#cd8823', ls='--', lw=2, label='OLD predicted future'),
              Line2D([], [], color='#2568ae', lw=2, label='FRESH predicted future'),
              Line2D([], [], color='#333c48', lw=3, label='Committed OLD (measured)'),
              Line2D([], [], color='#ee6575', lw=3, label='Pending OLD (measured)'),
              Line2D([], [], marker='X', color='#bb252e', ls='', ms=9, label='Actual switch B'),
              Line2D([], [], marker='D', color='#15936d', ls='', ms=6, label='FRESH nominal switch position'),
              Line2D([], [], color='#b840ac', ls=':', lw=2, label='Virtual RAW connector'),
              Line2D([], [], marker='o', mfc='white', mec='#2568ae', color='none', ls='', label='FRESH observation anchor')]
    for span, filename in ((6, 'handoff-closeup-6cm'), (80, 'handoff-context-80cm')):
        fig, axes = plt.subplots(1, 3, figsize=(16, 6.8))
        for ax, ex in zip(axes, examples):
            m = ex['metrics']
            ax.plot(*ex['old'].T, '--', color='#cd8823', lw=2, zorder=2)
            ax.plot(*ex['fresh'].T, color='#2568ae', lw=2, zorder=3)
            ax.plot(*ex['committed'].T, color='#333c48', lw=3, zorder=4)
            ax.plot(*ex['pending'].T, color='#ee6575', lw=3, zorder=5)
            ax.plot(*ex['connector'].T, ':', color='#b840ac', lw=2.2, zorder=6)
            ax.scatter(*ex['observation'], s=70, facecolor='white', edgecolor='#2568ae', zorder=8)
            ax.scatter(0, 0, s=100, marker='X', color='#bb252e', edgecolor='white', linewidth=.6, zorder=10)
            ax.scatter(*ex['nominal'], s=55, marker='D', color='#15936d', edgecolor='white', linewidth=.5, zorder=9)
            if span == 6:
                ax.plot([0, ex['nominal'][0]], [0, ex['nominal'][1]], color='#15936d', lw=1.3, zorder=8)
            ax.text(.04, .05, f'Boundary gap: {100*m["boundary_position_m"]:.3f} cm\n'
                    f'Pending OLD travel: {100*m["pending_path_m"]:.3f} cm',
                    transform=ax.transAxes, va='bottom', fontsize=11,
                    bbox=dict(boxstyle='round,pad=.4', facecolor='white', edgecolor='#d5dbe0', alpha=.97), zorder=12)
            ax.set_title(f'{ex["name"]}  |  C{m["old_id"]} -> C{m["fresh_id"]}\n'
                         f'Switch {m["switch_sim_s"]:.3f} sim s', fontsize=12, pad=10)
            ax.set(xlim=(-span/2, span/2), ylim=(-span/2, span/2),
                   xlabel='World X relative to switch B [cm]', ylabel='World Y relative to switch B [cm]')
            ax.set_aspect('equal', adjustable='box')
            ax.xaxis.set_major_locator(MultipleLocator(1 if span == 6 else 20))
            ax.yaxis.set_major_locator(MultipleLocator(1 if span == 6 else 20))
            ax.xaxis.set_minor_locator(MultipleLocator(.5 if span == 6 else 10))
            ax.yaxis.set_minor_locator(MultipleLocator(.5 if span == 6 else 10))
            ax.grid(which='major', alpha=.23); ax.grid(which='minor', alpha=.10)
            ax.axhline(0, color='#818b95', lw=.6, alpha=.5); ax.axvline(0, color='#818b95', lw=.6, alpha=.5)
        fig.suptitle(f'E16 handoff | {span} x {span} cm view at each measured switch', fontsize=19, y=.98)
        fig.legend(handles=legend, loc='upper center', bbox_to_anchor=(.5, .92), ncol=4, frameon=False, fontsize=10)
        fig.text(.5, .055, 'Display only: 100 x (world XY - measured switch XY). World axes preserved; no rotation or re-anchoring.\n'
                 'Identical scale in all three panels. Curves are clipped to the view; green point uses nominal timing.\n'
                 'Virtual connector is diagnostic, not the executed control path. Obstacle clearance remains in the full audit.',
                 ha='center', va='center', fontsize=10, color='#4b5560', linespacing=1.5)
        fig.subplots_adjust(left=.055, right=.985, top=.75, bottom=.18, wspace=.25)
        for suffix in ('png', 'svg'):
            fig.savefig(out/f'{filename}.{suffix}', dpi=180, facecolor='white')
        plt.close(fig)
    verify_snapshot(source, before)
    verify_snapshot(audit, audit_before)
    write(out/'display-provenance.json', dict(
        scope='DISPLAY_ONLY_CM_ZOOM; no new analysis, thresholds or classification',
        argv=sys.argv, branch=git('branch', '--show-current'), head=git('rev-parse', 'HEAD'),
        dirty_status=git('status', '--short'), source=str(source), prior_audit=str(audit),
        script_sha256=digest(Path(__file__)), audit_input_hashes=audit_before,
        protected_source_files=len(before), prior_audit_files_unchanged=len(audit_before),
        display_transform='q_cm = 100*(p_world_m - B_world_m); translation/unit scale only, world axes unchanged',
        native_transform='p_world_m = observation_xy_m + R(observation_yaw_CCW_rad) @ native_forward_left_m; +Z up',
        spans_cm=[6, 80], verdict_unchanged=verdict['verdict'],
        examples=[dict(name=e['name'], fresh_id=e['metrics']['fresh_id'],
                       boundary_gap_cm=100*e['metrics']['boundary_position_m'],
                       pending_path_cm=100*e['metrics']['pending_path_m'],
                       display_origin_world_xy_cm=e['display_origin_world_xy_cm']) for e in examples],
        artifacts={p.name: digest(p) for p in out.iterdir() if p.is_file()}))
    print(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.audit.resolve(), args.output.resolve(), Path(__file__).resolve().parents[1])


if __name__ == '__main__':
    main()
