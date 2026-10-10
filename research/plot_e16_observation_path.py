"""Saved-only measured E16 path: C markers are observation poses, not waypoints."""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.hospital_spatial_gate import wall_inventory
from research.records import provenance, write_json
from research.reveal_window import evidence_hashes, validate_destination, verify_hashes

FIELDS = ['request_id', 'sim_time', 'x_world_m', 'y_world_m', 'yaw_rad', 'yaw_deg']
CONVENTION = ('Hospital world XY in metres; yaw unchanged in radians, counterclockwise '
              'from world +X about +Z. Arrow = [cos(yaw), sin(yaw)]. '
              'C marker = measured robot SE(2) observation pose, not a native waypoint.')


def load_observations(source):
    source = Path(source)
    summary = json.loads((source / 'summary.json').read_text())
    events = [json.loads(p.read_text()) for p in sorted((source / 'raw/requests').glob('*.json'))]
    count = summary['successful_predictions']
    if summary['status'] != 'PASS' or not summary['all_finite']:
        raise ValueError('A completed finite baseline is required')
    if [e['request_id'] for e in events] != list(range(1, count + 1)):
        raise ValueError('Request count/IDs do not match the saved summary')
    if len(list((source / 'raw/requests').glob('*.npy'))) != count:
        raise ValueError('Native prediction count mismatch')
    with (source / 'robot_state.csv').open() as f:
        robot = list(csv.DictReader(f))
    ticks = {int(r['tick']): r for r in robot}
    rows = []
    for e in events:
        rid, obs = e['request_id'], e['observation']
        native = np.load(source / f'raw/requests/request_{rid:06d}.npy', allow_pickle=False)
        if native.shape != (30, 2) or not np.isfinite(native).all():
            raise ValueError(f'Invalid native prediction C{rid}')
        pose = e['agent_pose_at_observation']
        if pose != obs['pose'] or pose[:2] != obs['position'][:2] or not np.isfinite(pose).all():
            raise ValueError(f'Observation pose disagreement C{rid}')
        tick = ticks[obs['tick']]
        if pose != [float(tick[k]) for k in ['x', 'y', 'yaw']] or obs['sim_time'] != float(tick['sim_time']):
            raise ValueError(f'Observation does not equal measured tick C{rid}')
        rows.append(dict(zip(FIELDS, [rid, obs['sim_time'], *pose, float(np.degrees(pose[2]))])))
    dense = np.array([[float(r[k]) for k in ['tick', 'sim_time', 'x', 'y', 'yaw']] for r in robot])
    if not np.isfinite(dense).all() or np.any(np.diff(dense[:, 1]) <= 0):
        raise ValueError('Invalid measured robot path')
    return rows, dense


def draw_map(rows, dense, walls, zoom=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Patch
    plt.rcParams.update({'pdf.fonttype': 42, 'font.size': 11})
    fig, ax = plt.subplots(figsize=(16, 8.5) if not zoom else (13, 9))
    xy = dense[:, 2:4]
    bounds = [-5.2, 9., 5., 14.] if zoom else [xy[:, 0].min()-1.7, xy[:, 0].max()+2., xy[:, 1].min()-1.5, xy[:, 1].max()+2.]
    ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='World X [m]', ylabel='World Y [m]')
    used_walls, seen = [], set()
    for wall in walls:
        lo, hi = wall['world_aabb_min'], wall['world_aabb_max']
        if hi[0] < bounds[0] or lo[0] > bounds[1] or hi[1] < bounds[2] or lo[1] > bounds[3]:
            continue
        used_walls.append(wall['path'])
        key = tuple(lo[:2] + hi[:2])
        if key in seen:
            continue
        seen.add(key)
        ax.add_patch(Rectangle(lo[:2], hi[0]-lo[0], hi[1]-lo[1], facecolor='#e1e5e9', edgecolor='#b5bec7', lw=.45, zorder=0))
    ax.plot(*xy.T, color='#216e9d', lw=1.7, label='Actual executed robot path', zorder=2)
    visible = [r for r in rows if bounds[0] <= r['x_world_m'] <= bounds[1] and bounds[2] <= r['y_world_m'] <= bounds[3]]
    points = np.array([[r['x_world_m'], r['y_world_m']] for r in visible])
    yaws = np.array([r['yaw_rad'] for r in visible])
    # C1/C2 are only 2 mm apart: both remain at exact coordinates, with nested
    # marker styles (hollow C1 ring and filled C2 disk), never display jitter.
    marker = ax.scatter(*points.T, s=24 if not zoom else 36, color='#d66a16', edgecolors='white', lw=.7, zorder=5, label='C# observation pose')
    if not zoom:
        ax.scatter(*points[0], s=85, facecolors='none', edgecolors='#d66a16', lw=1.1, zorder=6)
    length = .32 if not zoom else .36
    ax.quiver(*points.T, length*np.cos(yaws), length*np.sin(yaws), angles='xy', scale_units='xy', scale=1,
              color='#514879', width=.002, headwidth=3.5, zorder=4, label='Saved observation heading')
    annotations = []
    for r in visible:
        rid, yaw = r['request_id'], r['yaw_rad']
        sign = 1 if rid % 2 else -1
        offset = np.array([-np.sin(yaw), np.cos(yaw)]) * sign * (17 if not zoom else 20)
        if not zoom and rid == 1:
            offset = [-25, -20]
        elif not zoom and rid == 2:
            offset = [28, -8]
        elif not zoom and rid == 3:
            offset = [28, 10]
        annotations.append(ax.annotate(f'C{rid}', (r['x_world_m'], r['y_world_m']), xytext=offset,
            textcoords='offset points', ha='center', va='center', fontsize=9 if not zoom else 11,
            color='#202c36', bbox=dict(boxstyle='round,pad=.12', fc='white', ec='none', alpha=.95),
            arrowprops=dict(arrowstyle='-', lw=.55, color='#66727c'), zorder=7))
    for pos, name, color, style, offset in [(xy[0], 'Start', '#19815e', 's', (0, -44)), (xy[-1], 'Final', '#b93242', 'X', (-7, 31))]:
        if bounds[0] <= pos[0] <= bounds[1] and bounds[2] <= pos[1] <= bounds[3]:
            ax.scatter(*pos, s=105, marker=style, facecolors='none' if style=='s' else color, edgecolors=color, lw=1.3, zorder=3, label=name)
            ax.annotate(name, pos, xytext=offset, textcoords='offset points', ha='center', color=color, weight='bold',
                        arrowprops=dict(arrowstyle='-', color=color, lw=.7), zorder=8)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(color='#b5bec7', alpha=.22, lw=.5)
    if zoom:
        from matplotlib.ticker import MultipleLocator
        ax.xaxis.set_major_locator(MultipleLocator(1))
        ax.yaxis.set_major_locator(MultipleLocator(1))
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Patch(facecolor='#e1e5e9', edgecolor='#b5bec7')); labels.append('Structural collider XY bounds')
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(.5, .9), ncol=3, frameon=False, fontsize=10)
    fig.suptitle('Hospital E16 lights-on baseline | '+('Original left turn / corner' if zoom else f'All {len(rows)} observation poses'), fontsize=17, y=.98)
    fig.text(.5, .93, 'C# = actual robot pose at request observation time (not a native trajectory waypoint)', ha='center', fontsize=11)
    footer = ('Saved measured path and yaw; no human or hypothetical trajectory. Collider bounds are map context, not exact surface clearance.'
              + ('\nC1 and C2 are approximately 2 mm apart; both markers use their exact saved coordinates.' if not zoom else '\nEvery observation inside this zoom is marked and labelled; all 48 are in the full map.'))
    fig.text(.5, .03, footer, ha='center', fontsize=10, color='#46535e')
    fig.subplots_adjust(left=.07, right=.97, bottom=.14, top=.8)
    fig.canvas.draw()
    boxes = [a.get_bbox_patch().get_window_extent() for a in annotations]
    overlaps = [(annotations[i].get_text(), annotations[j].get_text()) for i in range(len(boxes)) for j in range(i) if boxes[i].overlaps(boxes[j])]
    return fig, marker, {'request_ids': [r['request_id'] for r in visible], 'bounds_world_xy': bounds,
                         'label_overlap_pairs': overlaps, 'structural_collider_paths': used_walls}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', default='outputs/nova-e16-hospital-lights-baseline-20261010-01')
    parser.add_argument('--colliders', default='outputs/hospital-native-light-audit-20261010-01/collision_geometry.json')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    source, inventory = Path(args.run_dir).resolve(), Path(args.colliders).resolve()
    out = validate_destination(args.output_dir, [source, inventory.parent])
    hashes = evidence_hashes([source, inventory])
    rows, dense = load_observations(source)
    config = json.loads((source/'metadata.json').read_text())['config']
    if config['official_episode']['episode_id'] != 'episode_16' or 'stationary_human' in config or 'pedestrian' in config:
        raise ValueError('This plot requires the no-human E16 baseline')
    out.mkdir(parents=True, exist_ok=False)
    with (out/'e16_c_observation_poses.csv').open('x') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS); writer.writeheader(); writer.writerows(rows)
    with (out/'e16_executed_robot_path.csv').open('x') as f:
        writer = csv.writer(f); writer.writerow(['tick', 'sim_time', 'x_world_m', 'y_world_m', 'yaw_rad', 'yaw_deg'])
        writer.writerows([int(r[0]), *r[1:], float(np.degrees(r[-1]))] for r in dense)
    with (out/'e16_c_observation_poses.csv').open() as f:
        exported = list(csv.DictReader(f))
    assert [[float(r[k]) for k in FIELDS] for r in exported] == [[r[k] for k in FIELDS] for r in rows]
    walls = wall_inventory(json.loads(inventory.read_text()))
    figures = {}
    for zoom in [False, True]:
        stem = 'e16_c_observation_path' + ('_turn_zoom' if zoom else '')
        fig, marker, info = draw_map(rows, dense, walls, zoom)
        expected = np.array([[rows[i-1]['x_world_m'], rows[i-1]['y_world_m']] for i in info['request_ids']])
        np.testing.assert_array_equal(marker.get_offsets(), expected)
        if info['label_overlap_pairs']:
            raise ValueError('Adjust label offsets before delivery: '+str(info['label_overlap_pairs']))
        fig.savefig(out/(stem+'.png'), dpi=240)
        fig.savefig(out/(stem+'.pdf'))
        import matplotlib.pyplot as plt
        plt.close(fig)
        figures[stem] = info
    verify_hashes(hashes)
    if hashes != evidence_hashes([source, inventory]):
        raise ValueError('Source file inventory changed')
    write_json(out/'metadata.json', {**provenance(ROOT), 'source_run': str(source), 'source_evidence_sha256': hashes,
        'script_sha256': evidence_hashes([Path(__file__)]), 'coordinate_convention': CONVENTION,
        'dense_source': str(source/'robot_state.csv'), 'request_count': len(rows), 'dense_pose_count': len(dense),
        'figures': figures, 'model_calls': 0, 'physics_reexecution': False,
        'validation': {'finite_prediction_count_matches': True, 'exact_saved_observation_and_tick_pose': True,
                       'csv_roundtrip_exact': True, 'plotted_marker_coordinates_exact': True, 'source_files_unchanged': True}})
    print(json.dumps({'output_dir': str(out), 'request_count': len(rows), 'dense_pose_count': len(dense), 'figures': figures}, indent=2))


if __name__ == '__main__':
    main()
