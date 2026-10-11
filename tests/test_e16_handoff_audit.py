import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from research.analyze_chunk_geometry import project_world, rotation, wrap
from research.analyze_e16_handoff_audit import (
    CONFIG, INVENTORY, analyze, command_jump, editable_overlap, exact_event_pose,
    overlap_metrics, phase_at, seam_metrics, select_cases,
)
from research.analyze_nova_dynamic_handoff import load_saved
from research.route_geometry import interpolate, temporal_overlap
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [ROOT/'outputs'/p for p in (
    'nova-e16-hospital-lights-baseline-20261010-01',
    'nova-e16-no-cart-15-repro-20261011-01')]


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(CONFIG.read_text())

    def test_projection_and_temporal_frame_invariance(self):
        np.testing.assert_allclose(project_world([[2, 1]], [3, 4, np.pi/2]), [[2, 6]])
        t = .1*np.arange(1, 31)
        world = lambda q: np.column_stack([2*q, .4*q+5])
        poses = [[4, 1, .8], [-3, 2, -1.3]]
        ow, fw = [] , []
        for absolute, pose in [(t, poses[0]), (t+.5, poses[1])]:
            local = (world(absolute)-pose[:2]) @ rotation(pose[2])
            restored = project_world(local, pose)
            if not len(ow): ow = restored
            else: fw = restored
        full = temporal_overlap(ow, fw, 0, .5)
        self.assertEqual(len(full['absolute_times']), 25)
        self.assertLess(full['rmse'], 1e-14)
        shifted = temporal_overlap(ow, fw + np.array([-np.sin(poses[1][2]), np.cos(poses[1][2])])*.2, 0, .5)
        metrics = overlap_metrics(shifted, poses[1][2], 1e-6)
        self.assertAlmostEqual(metrics['mean_signed_lateral_m'], .2)

    def test_editable_overlap_no_extrapolation_and_original_knots(self):
        t = .1*np.arange(1, 31)
        ow = np.column_stack([t, np.sin(t*9)])
        fw = np.column_stack([t+.55, t*0])
        full = temporal_overlap(ow, fw, 0, .55)
        clipped = editable_overlap(full, ow, fw, 0, .55, .72, .1)
        self.assertEqual(clipped['absolute_times'][0], .72)
        np.testing.assert_allclose(clipped['old'][0], interpolate(t, ow, .72))
        self.assertIs(editable_overlap(full, ow, fw, 0, .55, .6, .1), full)
        self.assertIsNone(editable_overlap(full, ow, fw, 0, .55, 3., .1))
        with self.assertRaises(ValueError): interpolate(t, ow, .09)
        with self.assertRaises(ValueError): interpolate(t, ow, 3.01)

    def test_correct_switch_pose_and_clock(self):
        rows = [dict(tick='0', sim_time='2.0', x='1', y='2', z='0', yaw='.3')]
        state = dict(tick=0, sim_time=2., position=[1., 2., 0.], pose=[1., 2., .3])
        event = dict(request_id=3, application=state, agent_pose_at_application=state['pose'])
        exact_event_pose(event, rows, 'application')
        for field, value in [('sim_time', 2.1), ('pose', [1, 2, .4]), ('position', [2, 2, 0])]:
            bad = copy.deepcopy(event); bad['application'][field] = value
            with self.assertRaises(ValueError): exact_event_pose(bad, rows, 'application')

    def test_boundary_own_observation_and_wrap(self):
        world = np.column_stack([-.1*np.arange(1, 31), -.001*np.arange(1, 31)])
        obs = dict(sim_time=0., position=[0, 0, 0], pose=[0, 0, 0])
        app = dict(sim_time=.15, position=[-.15, -.0015, 0], pose=[-.15, -.0015, np.pi-.01])
        event = dict(observation=obs, application=app)
        times = np.array([0., .1, .15]); xy = np.array([[0, 0], [-.1, -.001], [-.15, -.0015]])
        seam = seam_metrics(world, event, times, xy, self.config)
        self.assertLess(seam['raw_boundary_position_gap_m'], 1e-12)
        self.assertLess(seam['raw_robot_heading_to_fresh_tangent_gap_deg'], 1.2)
        self.assertAlmostEqual(np.degrees(wrap(np.deg2rad(-179-179))), 2)
        event['application']['sim_time'] = 3.1
        self.assertIsNone(seam_metrics(world, event, times, xy, self.config)['raw_boundary_position_gap_m'])

    def test_degenerate_tangents_have_reasons(self):
        event = dict(observation=dict(sim_time=0., position=[0, 0, 0], pose=[0, 0, 0]),
                     application=dict(sim_time=.15, position=[0, 0, 0], pose=[0, 0, 0]))
        world = np.zeros((30, 2))
        seam = seam_metrics(world, event, np.array([0., .15]), np.zeros((2, 2)), self.config)
        self.assertIsNone(seam['raw_robot_heading_to_fresh_tangent_gap_deg'])
        self.assertIsNotNone(seam['robot_heading_undefined_reason'])
        metric = overlap_metrics(temporal_overlap(world, world, 0, .5), 0, 1e-6)
        self.assertEqual(metric['valid_tangent_count'], 0)
        self.assertIsNone(metric['mean_abs_tangent_deg'])
        moving = np.column_stack([np.arange(1, 31)*.1, np.zeros(30)])
        seam = seam_metrics(moving, event, np.array([0., .15]), np.zeros((2, 2)), self.config)
        self.assertIsNone(seam['raw_executed_to_fresh_tangent_gap_deg'])
        self.assertIsNotNone(seam['executed_heading_undefined_reason'])

    def test_command_difference(self):
        old = dict(controller_command=[1.5, .4])
        fresh = dict(old_controller_command=[1.5, .4], controller_command=[1.5, -.2])
        d = command_jump(old, fresh)
        self.assertEqual(d['delta_v'], 0)
        self.assertAlmostEqual(d['delta_w'], -.6)
        fresh['old_controller_command'] = [1.5, .3]
        with self.assertRaises(ValueError): command_jump(old, fresh)

    def test_case_selection_keeps_undefined_headings_undefined(self):
        rows = [dict(old_request_id=1, fresh_request_id=2, phase='left_turn',
                     boundary_position_m=.1, boundary_robot_heading_deg=None,
                     full_rmse_m=.2, full_max_abs_lateral_m=.3)]
        stats = dict(all=dict(boundary_position_m=dict(median=.1), boundary_robot_heading_deg=dict(median=None)))
        selected = select_cases(dict(example=dict(rows=rows, stats=stats)))
        self.assertIsNone(selected[0]['descending_ranks']['boundary_robot_heading_deg'])
        self.assertIsNone(selected[1]['pair'])
        self.assertIsNone(selected[2]['pair'])

    def test_phases_use_physical_progress_and_do_not_stick_to_goal(self):
        cfg = dict(spatial_turn_gate=dict(center_y_min_m=9.5), controller=dict(max_v=1.5), scene=dict(goal=[-12, 10]))
        nav = dict(corner_entry=dict(sim_time=8.), corner_passage_completion_sim_time=12.)
        for t, xy, phase in [(2, [7, 2], 'initial_travel'), (6, [7, 6], 'corner_approach'),
                             (10, [4, 12], 'left_turn'), (14, [-4, 11], 'hallway_straight'),
                             (20, [-12, 10], 'near_goal'), (25, [-20, 10], 'hallway_straight')]:
            self.assertEqual(phase_at(dict(sim_time=t, position=xy), cfg, nav, 2., .1), phase)

    def test_output_cannot_overwrite_or_enter_raw(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)/'source'; source.mkdir(); (source/'raw.bin').write_bytes(b'original')
            before = evidence_hashes([source])
            for target in [source, source/'derived']:
                with self.assertRaises(ValueError): validate_destination(target, [source])
            output = validate_destination(Path(temp)/'analysis', [source]); output.mkdir()
            with self.assertRaises(FileExistsError): validate_destination(output, [source])
            verify_hashes(before)
            self.assertEqual(evidence_hashes([source]), before)


@unittest.skipUnless(all((p/'summary.json').exists() for p in SOURCES), 'Local saved evidence absent')
class SavedEvidenceTests(unittest.TestCase):
    def test_two_runs_48_47_exact_switches_and_raw_immutability(self):
        before = evidence_hashes(SOURCES)
        results = {}
        config = json.loads(CONFIG.read_text()); inventory = json.loads(INVENTORY.read_text())
        for name, path in zip(['historical', 'reproduction'], SOURCES):
            run = load_saved(path); result = analyze(run, config, inventory); results[name] = result
            self.assertEqual(len(run['chunks']), 48); self.assertEqual(len(result['rows']), 47)
            self.assertEqual([(r['old_request_id'], r['fresh_request_id']) for r in result['rows']], [(i, i+1) for i in range(1, 48)])
            for row in result['rows']:
                self.assertGreaterEqual(row['editable_start_sim_time'], row['switch_sim_time'])
                self.assertLessEqual(row['full_end_sim_time'], row['old_observation_sim_time']+3.0)
                self.assertAlmostEqual(row['delta_v'], 0)
                self.assertIsNone(row['model_ready_sim_time'])
        self.assertEqual(len(select_cases(results)), 10)
        verify_hashes(before); self.assertEqual(evidence_hashes(SOURCES), before)


if __name__ == '__main__': unittest.main()
