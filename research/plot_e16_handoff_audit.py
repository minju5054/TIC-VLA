"""Static scientific figures from the saved E16 audit; no simulator rendering."""
import numpy as np
from research.hospital_spatial_gate import wall_inventory

COLORS = {'historical': '#2078a5', 'reproduction': '#b44480'}
LABELS = {'historical': 'A: historical', 'reproduction': 'B: reproduction'}
PHASE_COLORS = dict(initial_travel='#999999', corner_approach='#deac29', left_turn='#d44f40',
                    hallway_straight='#278cb2', near_goal='#6a51a3')


def figures(out, runs, results, cases, inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'font.size': 10})
    walls = wall_inventory(inventory)

    def save(fig, stem):
        fig.savefig(out/(stem+'.png'), dpi=175, bbox_inches='tight')
        plt.close(fig)

    def context(ax):
        for wall in walls:
            a, b = np.array(wall['world_aabb_min']), np.array(wall['world_aabb_max'])
            ax.add_patch(Rectangle(a[:2], *(b-a)[:2], fc='#eeeeee', ec='#cccccc', lw=.4, zorder=0))
        ax.set(xlabel='World X [m]', ylabel='World Y [m]')
        ax.set_aspect('equal'); ax.grid(alpha=.15)

    fig, axes = plt.subplots(2, 1, figsize=(15, 11), layout='constrained')
    for ax, (name, run) in zip(axes, runs.items()):
        context(ax); xy = run['xy']; obs = np.array([e['agent_pose_at_observation'][:2] for e in run['events']])
        ax.plot(*xy.T, c=COLORS[name], lw=1.8, label='Measured executed path')
        artist = ax.scatter(*obs.T, s=18, c=COLORS[name], zorder=4, label='C = actual observation pose')
        np.testing.assert_array_equal(artist.get_offsets(), obs)
        for i, point in enumerate(obs, 1):
            offset = (4, 9) if i % 2 else (-8, -15)
            if i == 1: offset = (25, -15)
            if i == 2: offset = (-26, 8)
            ax.annotate(f'C{i}', point, xytext=offset, textcoords='offset points', fontsize=7,
                        arrowprops=dict(arrowstyle='-', lw=.4, color='#888888'))
        ax.scatter(*xy[0], marker='s', facecolors='none', edgecolors='black', s=70, label='Start')
        ax.scatter(*xy[-1], marker='X', c='black', s=55, label='End')
        goal = run['cfg']['scene']['goal'][:2]
        ax.scatter(*goal, marker='*', c='#6a51a3', s=90, label='Goal')
        lo, hi = xy.min(0)-1.1, xy.max(0)+1.4
        ax.set(xlim=[lo[0], hi[0]], ylim=[lo[1], hi[1]], title=LABELS[name]+' | 48 saved observation poses, no added actor')
        ax.legend(fontsize=8, loc='lower left', ncol=2)
    save(fig, 'world_paths')

    def ecdf(ax, name, key, label=None, style='-'):
        values = sorted(r[key] for r in results[name]['rows'] if r[key] is not None)
        if values:
            ax.step(values, np.arange(1, len(values)+1)/len(values), where='post',
                    c=COLORS[name], ls=style, label=label or LABELS[name])
        ax.set_ylabel('Within-run fraction'); ax.grid(alpha=.2)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout='constrained')
    for ax, key, label in zip(axes,
            ['boundary_position_m', 'boundary_robot_heading_deg', 'boundary_executed_heading_deg'],
            ['Position seam [m]', 'Robot yaw vs FRESH tangent [deg]', 'Executed direction vs FRESH tangent [deg]']):
        for name in runs: ecdf(ax, name, key)
        ax.set_xlabel(label); ax.legend(fontsize=8)
    fig.suptitle('Boundary distributions | 47 adjacent handoffs per run; tangents are derived')
    save(fig, 'boundary_distributions')

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout='constrained')
    for ax, key, label in zip(axes, ['rmse_m', 'mean_abs_lateral_m', 'mean_abs_tangent_deg'],
            ['Overlap RMSE [m]', 'Mean absolute lateral revision [m]', 'Mean derived tangent difference [deg]']):
        for name in runs:
            ecdf(ax, name, 'full_'+key, LABELS[name]+' full')
            ecdf(ax, name, 'editable_'+key, LABELS[name]+' post-switch', '--')
        ax.set_xlabel(label); ax.legend(fontsize=7)
    fig.suptitle('Future disagreement | own observation frames, same nominal absolute target time')
    save(fig, 'future_distributions')

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout='constrained')
    for name in runs:
        ecdf(axes[0], name, 'delta_w'); ecdf(axes[1], name, 'measured_next_tick_delta_yaw_rad_s')
    axes[0].set_xlabel('Command delta_w [rad/s]')
    axes[1].set_xlabel('Measured yaw-rate change at next physics tick [rad/s]')
    for ax in axes: ax.legend(fontsize=8)
    fig.suptitle('Command jump and measured response are different quantities; delta_v = 0 throughout')
    save(fig, 'command_distributions')

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
    for ax, name in zip(axes, runs):
        for phase, color in PHASE_COLORS.items():
            rows = [r for r in results[name]['rows'] if r['phase'] == phase]
            ax.scatter([r['boundary_position_m'] for r in rows], [r['full_rmse_m'] for r in rows],
                       c=color, s=35, label=phase.replace('_', ' '), alpha=.8)
        ax.set(xlabel='Actual B vs nominal FRESH position seam [m]', ylabel='Full OLD/FRESH overlap RMSE [m]', title=LABELS[name])
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle('Boundary vs future | phase assigned at measured switch pose/progress; no severity cutoff')
    save(fig, 'boundary_vs_future')

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for ax, metric, unit in zip(axes.flat, ['boundary_position_m', 'boundary_robot_heading_deg', 'full_rmse_m', 'abs_delta_w'], ['m', 'deg', 'm', 'rad/s']):
        for name in runs:
            offset = -.14 if name == 'historical' else .14
            for j, phase in enumerate(PHASE_COLORS):
                values = [r[metric] for r in results[name]['rows'] if r['phase'] == phase and r[metric] is not None]
                ax.scatter(np.full(len(values), j+offset), values, c=COLORS[name], s=16, alpha=.5)
                if values: ax.plot([j+offset-.09, j+offset+.09], [np.median(values)]*2, c=COLORS[name], lw=3)
            ax.plot([], [], c=COLORS[name], label=LABELS[name])
        ax.set_xticks(range(5), ['Initial', 'Approach', 'Turn', 'Hallway', 'Goal'])
        ax.set_ylabel(metric.replace('_', ' ')+f' [{unit}]'); ax.grid(axis='y', alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('Physical-progress phases | all samples + median bars; two dependent within-run series')
    save(fig, 'phase_comparison')

    unique = {(c['run'], c['pair']['old_request_id']) for c in cases if c['pair']}
    for name, old_id in sorted(unique):
        d = results[name]['details'][str(old_id)]; r = results[name]['rows'][old_id-1]; run = runs[name]
        fig, axes = plt.subplots(1, 2, figsize=(13, 6), layout='constrained', gridspec_kw={'width_ratios': [1.6, 1]})
        b = np.array(d['switch']['position'][:2])
        expected = d['boundary']['fresh_expected_xy']
        expected = np.array(expected) if expected is not None else None
        st = r['switch_sim_time']; mask = (run['times'] <= st) & (run['times'] >= st-3.)
        history = run['xy'][mask]
        for ax in axes:
            context(ax)
            ax.plot(*history.T, c='#303030', lw=2, label='Measured history (last 3 sim s)')
            ax.plot(*d['old_full_world'].T, 'o--', c='#b98320', mfc='white', ms=3, lw=1, label=f'OLD C{old_id}: projected native points')
            if d['old_remaining_world'] is not None:
                ax.plot(*d['old_remaining_world'].T, c='#b98320', lw=2.5, label='OLD remaining at B (derived clip)')
            ax.plot(*d['fresh_full_world'].T, '.-', c='#8646aa', ms=5, label=f'FRESH C{old_id+1}: projected native points')
            for key, color, label in [('old_observation', '#b98320', 'OLD observation pose'), ('fresh_observation', '#8646aa', 'FRESH observation pose')]:
                p = d[key]['pose']; ax.scatter(*p[:2], marker='D', c=color, s=38, zorder=5, label=label)
                ax.quiver(*p[:2], .2*np.cos(p[2]), .2*np.sin(p[2]), angles='xy', scale_units='xy', scale=1, color=color)
            ax.scatter(*b, c='black', marker='X', s=75, zorder=6, label='Actual switch B')
            if expected is not None:
                ax.scatter(*expected, c='#169e6b', marker='*', s=90, zorder=7, label='Nominal FRESH at switch (derived)')
                ax.plot(*np.vstack([b, expected]).T, c='#169e6b', ls=':', lw=1.5)
        points = np.vstack([history, d['old_full_world'], d['fresh_full_world']]); lo, hi = points.min(0)-.35, points.max(0)+.35
        axes[0].set(xlim=[lo[0], hi[0]], ylim=[lo[1], hi[1]], title='World geometry and executed history')
        center = (b+expected)/2 if expected is not None else b
        half = max(.16, np.linalg.norm(b-expected)*1.6) if expected is not None else .16
        axes[1].set(xlim=[center[0]-half, center[0]+half], ylim=[center[1]-half, center[1]+half], title='Boundary detail (equal aspect)')
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, fontsize=8, loc='lower center', bbox_to_anchor=(.5, -.13), ncol=3)
        def display(value):
            return f'{value:.4f}' if value is not None else 'undefined'
        categories = ', '.join(c['category'] for c in cases if c['run'] == name and c['pair'] and c['pair']['old_request_id'] == old_id)
        fig.suptitle(f"{LABELS[name]} C{old_id} -> C{old_id+1} | {r['phase']}\n"
            f"Position seam {display(r['boundary_position_m'])} m; robot/tangent {display(r['boundary_robot_heading_deg'])} deg; full RMSE {display(r['full_rmse_m'])} m\n"
            + categories.replace('_', ' '), fontsize=10)
        save(fig, f'{name}_pair_{old_id:02d}_{old_id+1:02d}')
