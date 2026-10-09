import copy
import math
from pathlib import Path
import unittest
import tempfile
import numpy as np
import yaml
from research.route_geometry import (route_side, temporal_overlap, boundary_seam, interpolate,
    feasibility, segment_intersects, rectangle, segment_clearance, static_route_gate)
from research.pedestrian import reveal_position, crossing_position

ROOT = Path(__file__).resolve().parents[1]
BLOCK = {"name": "central_blocker", "position": [4.8, 0, 1.2], "size": [1, 1.6, 2.4]}


class RouteTests(unittest.TestCase):
    def test_route_side_and_ambiguity(self):
        for y, side in [(1.5, "LEFT"), (-1.5, "RIGHT"), (0, "CENTER/BLOCKED"), (2.4, "CENTER/BLOCKED")]:
            result = route_side([[3, y], [6, y]], 4.8, BLOCK, .4, 2.5)
            self.assertEqual(result["route_side"], side)
            self.assertAlmostEqual(result["gate_y"], y)
        self.assertEqual(route_side([[0, 0], [3, 0]], 4.8, BLOCK, .4, 2.5)["route_side"], "UNKNOWN/DOES_NOT_REACH")
        self.assertEqual(route_side([[3, 1.5], [6, 1.5], [3, -1.5]], 4.8, BLOCK, .4, 2.5)["route_side"], "UNKNOWN/AMBIGUOUS")

    def test_arbitrary_time_boundary_and_no_extrapolation(self):
        obs = {"position": [0, 0, 0], "sim_time": 1.0}
        app = {"position": [.2, .1, 0], "pose": [.2, .1, 0], "sim_time": 1.15}
        world = np.column_stack([np.arange(1, 31)*.1, np.zeros(30)])
        expected = interpolate(np.arange(31)*.1, np.vstack([[0, 0], world]), .15)
        np.testing.assert_allclose(expected, [.15, 0])
        result = boundary_seam(world, obs, app, np.array([1., 1.05, 1.15]), np.array([[0, .1], [.1, .1], [.2, .1]]))
        self.assertAlmostEqual(result["raw_boundary_position_gap_m"], math.hypot(.05, .1))
        self.assertAlmostEqual(result["raw_executed_to_fresh_tangent_gap_deg"], 0)
        self.assertAlmostEqual(result["raw_robot_heading_to_fresh_tangent_gap_deg"], 0)
        with self.assertRaises(ValueError):
            interpolate([.1, .2], [[0, 0], [1, 0]], .3)
        app["sim_time"] = 4.1
        self.assertIsNone(boundary_seam(world, obs, app, np.array([1, 5]), np.array([[0, 0], [1, 0]]))["raw_boundary_position_gap_m"])

    def test_tangent_wrap_and_undefined_stationary(self):
        old_angle, fresh_angle = np.radians([179, -179])
        direction = np.array([np.cos(fresh_angle), np.sin(fresh_angle)])
        world = np.arange(1, 31)[:, None]*.1*direction
        obs = {"position": [0, 0, 0], "sim_time": 0.0}
        app = {"position": [0, 0, 0], "pose": [0, 0, old_angle], "sim_time": .15}
        executed = np.array([[0, 0], [.1*np.cos(old_angle), .1*np.sin(old_angle)]])
        result = boundary_seam(world, obs, app, np.array([0, .15]), executed)
        self.assertAlmostEqual(result["raw_executed_to_fresh_tangent_gap_deg"], 2)
        result = boundary_seam(world, obs, app, np.array([0, .15]), np.zeros((2, 2)))
        self.assertIsNone(result["raw_executed_to_fresh_tangent_gap_deg"])

    def test_variable_gap_alignment(self):
        dt, gap = .1, .537
        times = dt*np.arange(1, 31)
        old = np.column_stack([2*times, -times])
        fresh = np.column_stack([2*(times+gap), -(times+gap)])
        overlap = temporal_overlap(old, fresh, 0, gap)
        self.assertLess(overlap["rmse"], 1e-12)
        self.assertAlmostEqual(overlap["absolute_times"][0], gap+.1)
        self.assertAlmostEqual(overlap["absolute_times"][-1], 3.)
        np.testing.assert_allclose(temporal_overlap(old, fresh+[0, .3], 0, gap)["delta"], np.tile([0, .3], (len(overlap["delta"]), 1)), atol=1e-12)

    def test_obstacle_clearance_full_segments_and_occlusion(self):
        for y in [1.5, -1.5]:
            f = feasibility(np.array([[3, y], [6, y]]), [BLOCK], .4)
            self.assertFalse(f["predicted_static_obstacle_intersection"])
            self.assertAlmostEqual(f["minimum_static_obstacle_clearance_m"], .3)
        f = feasibility(np.array([[3, 0], [6, 0]]), [BLOCK], .4)
        self.assertTrue(f["predicted_static_obstacle_intersection"])
        self.assertAlmostEqual(f["minimum_static_obstacle_clearance_m"], -.9)
        self.assertTrue(segment_intersects([.26, 0], [5.8, 0], rectangle(BLOCK)))
        self.assertFalse(segment_intersects([.26, 0], [5.8, 1.6], rectangle(BLOCK)))
        self.assertAlmostEqual(segment_clearance([2, 2], [3, 3], [0, 0, 1, 1]), math.sqrt(2))

    def test_fixed_trigger_continuity_and_mirror(self):
        cfg = {"hidden_start_position": [5.8, 0, 0], "velocity": [0, 1, 0], "reveal_delay_s": 0., "max_displacement_m": 1.6}
        np.testing.assert_allclose(reveal_position(cfg, 100, None), [5.8, 0, 0])
        np.testing.assert_allclose(reveal_position(cfg, 3, 3), [5.8, 0, 0])
        np.testing.assert_allclose(reveal_position(cfg, 3.5, 3), [5.8, .5, 0])
        np.testing.assert_allclose(reveal_position(cfg, 30, 3), [5.8, 1.6, 0])
        mirror = {**cfg, "velocity": [0, -1, 0]}
        np.testing.assert_allclose(reveal_position(mirror, 3.5, 3), [5.8, -.5, 0])

    def test_mirrored_configs_and_static_gate(self):
        configs = [yaml.safe_load((ROOT/f"configs/research/route_switch_{name}.yaml").read_text()) for name in ["static", "block_left", "block_right"]]
        a, b = copy.deepcopy(configs[1]), copy.deepcopy(configs[2])
        a.pop("run_variant"); b.pop("run_variant")
        a["pedestrian"]["velocity"][1] *= -1
        a["pedestrian"]["yaw"] *= -1
        self.assertEqual(a, b)
        for key in ["scene", "robot", "camera", "controller", "inference", "instruction", "seed", "route_switch"]:
            self.assertEqual(configs[0][key], configs[1][key])
        contexts = [{"request_id": rid, "route_side": "LEFT", "predicted_static_obstacle_intersection": False} for rid in [3, 4]]
        self.assertTrue(static_route_gate(contexts, configs[0])["valid"])
        contexts[1]["predicted_static_obstacle_intersection"] = True
        self.assertFalse(static_route_gate(contexts, configs[0])["valid"])

    def test_clearance_against_dense_independent_sampling(self):
        rng = np.random.default_rng(11)
        rect = np.array([-1., -.5, 1., .5])
        for _ in range(20):
            a, b = rng.uniform(-2, 2, (2, 2))
            points = a+np.linspace(0, 1, 20001)[:, None]*(b-a)
            exterior = np.maximum(np.maximum(rect[:2]-points, points-rect[2:]), 0)
            distance = np.linalg.norm(exterior, axis=1)
            inside = (exterior == 0).all(axis=1)
            distance[inside] = -np.minimum(points[inside]-rect[:2], rect[2:]-points[inside]).min(axis=1)
            exact = segment_clearance(a, b, rect)
            self.assertLessEqual(exact, distance.min()+1e-10)
            self.assertLess(distance.min()-exact, .0003)

    def test_analysis_no_overwrite_or_source_output(self):
        from research.analyze_intent_handoff import analyze
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory)/"existing"
            existing.mkdir()
            sentinel = existing/"raw.txt"
            sentinel.write_text("immutable")
            with self.assertRaises(FileExistsError):
                analyze("not-read", None, None, None, existing)
            self.assertEqual(sentinel.read_text(), "immutable")
            with self.assertRaises(ValueError):
                analyze(existing, None, None, None, existing/"analysis")
            self.assertFalse((existing/"analysis").exists())


if __name__ == "__main__":
    unittest.main()
