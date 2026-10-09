"""Pure synthetic calibration tests; fixtures are not experimental evidence."""
import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from research.analyze_static_detour import (path_context, actual_context, consecutive_validity, analyze, sha, camera_context)
from research.route_geometry import rectangle, segment_intersects
from research.records import RunRecords, write_json

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT/"configs/research/asymmetric_offset_detour.yaml").read_text())


class StaticDetourTests(unittest.TestCase):
    def test_asymmetric_geometry(self):
        block = CFG["scene"]["obstacles"][0]
        np.testing.assert_allclose(rectangle(block, .4), [3, -1.7, 4.6, .5])
        self.assertTrue(segment_intersects([0, 0], [8, 0], rectangle(block)))
        self.assertGreater(2.45-rectangle(block)[3], .8)
        self.assertGreater(2.45-.4, rectangle(block, .4)[3])

    def test_valid_left_and_full_path_feasibility(self):
        valid = path_context([[0, 0], [2, 1], [5, 1], [6, 0]], CFG)
        self.assertEqual(valid["route_side"], "LEFT_VALID")
        self.assertAlmostEqual(valid["gate_y"], 1)
        self.assertAlmostEqual(valid["minimum_offset_blocker_clearance_m"], .5)
        invalid = path_context([[0, 0], [3.3, 0], [3.8, 1], [5, 1]], CFG)
        self.assertEqual(invalid["gate_route_side"], "LEFT")
        self.assertTrue(invalid["predicted_static_obstacle_intersection"])
        self.assertNotEqual(invalid["route_side"], "LEFT_VALID")

    def test_blocked_and_unreached_classification(self):
        blocked = path_context([[0, 0], [6, 0]], CFG)
        self.assertEqual(blocked["route_side"], "CENTER/BLOCKED")
        self.assertTrue(blocked["predicted_uninflated_blocker_intersection"])
        self.assertAlmostEqual(blocked["minimum_offset_blocker_clearance_m"], -.5)
        self.assertEqual(path_context([[0, 0], [2, .1]], CFG)["route_side"], "DOES_NOT_REACH")
        self.assertEqual(path_context([[2, 1], [5, 1], [2, 1.5]], CFG)["route_side"], "AMBIGUOUS")

    def test_actual_clearance_progress_and_positive_left(self):
        actual = actual_context([[0, 0], [2, 1], [5, 1], [6, 0]], CFG)
        self.assertAlmostEqual(actual["actual_min_clearance_to_blocker"], .5)
        self.assertTrue(actual["execution_static_detour_validated"])
        self.assertFalse(actual_context([[0, 0], [5, 0]], CFG)["execution_static_detour_validated"])
        self.assertFalse(actual_context([[0, 0], [2, 1], [4.6, 1]], CFG)["actual_passed_blocker"])

    def test_consecutive_rule_excludes_startup_and_missing_ids(self):
        def row(rid, valid=True):
            return {"request_id": rid, "route_side": "LEFT_VALID" if valid else "DOES_NOT_REACH",
                    "predicted_static_obstacle_intersection": False}
        result = consecutive_validity([row(1), row(2), row(3), row(4), row(5, False), row(6), row(7), row(8)])
        self.assertEqual(result["consecutive_valid_left_pairs"], [[6, 7], [7, 8]])
        self.assertEqual(result["longest_consecutive_valid_left_run"], 3)
        self.assertFalse(consecutive_validity([row(3), row(4)])["prediction_static_detour_validated"])
        self.assertFalse(consecutive_validity([row(4), row(6)])["prediction_static_detour_validated"])

    def test_goal_wall_is_separate_from_avoidance_failure(self):
        points = [[0, 0], [2, 1], [5, 1], [8, 0]]
        c = path_context(points, CFG)
        self.assertTrue(c["predicted_terminal_goal_wall_intersection"])
        self.assertTrue(c["predicted_any_static_including_goal_intersection"])
        self.assertFalse(c["predicted_static_obstacle_intersection"])
        self.assertEqual(c["route_side"], "LEFT_VALID")
        a = actual_context(points, CFG)
        self.assertTrue(a["actual_terminal_goal_wall_intersection"])
        self.assertTrue(a["execution_static_detour_validated"])

    def test_fixed_config_preserves_model_controller_continuous_policy(self):
        base = yaml.safe_load((ROOT/"configs/research/route_switch_static.yaml").read_text())
        for key in ["robot", "camera", "controller", "inference", "seed", "simulation", "instruction"]:
            self.assertEqual(CFG[key], base[key])
        self.assertNotIn("pedestrian", CFG)
        self.assertEqual(CFG["static_detour"]["first_primary_request_id"], 4)
        self.assertEqual(CFG["static_detour"]["minimum_consecutive_valid_left"], 2)

    def test_camera_containment_uses_full_body_transform(self):
        event = {"observation": {"position": [3.15, 0, .064], "quaternion_wxyz": [1, 0, 0, 0]}}
        self.assertTrue(camera_context(event, CFG)["camera_inside_physical_blocker"])
        event["observation"]["quaternion_wxyz"] = [0, 0, 0, 1]
        self.assertFalse(camera_context(event, CFG)["camera_inside_physical_blocker"])

    def test_saved_analysis_immutability_anchor_and_no_overwrite(self):
        # Reuse event/tick fixtures, with a complete synthetic timeline and explicit
        # fixture metadata. No simulator/model is loaded and no real run is read.
        from test_continuous import event_fixture, tick_fixture
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); source = root/"synthetic_fixture"
            cfg = copy.deepcopy(CFG); cfg["simulation"]["predictions"] = 4
            records = RunRecords(source, {"mode": "static", "config": cfg})
            write_json(source/"summary.json", {"status": "PASS", "continuous_handoff_validated": True})
            write_json(source/"inference-runtime.json", {"strict_checkpoint": True, "simulation_app_started": False})
            robot = []
            for rid in range(1, 5):
                event = event_fixture(rid); event["lookahead_index"] = 4
                event["observation"]["quaternion_wxyz"] = [np.cos(np.radians(179)/2), 0, 0, np.sin(np.radians(179)/2)]
                event["rgb_observation_reference"] = f"diagnostics/rgb_{rid}.png"
                (source/event["rgb_observation_reference"]).write_bytes(b"synthetic RGB fixture")
                records.request(rid, np.column_stack([np.arange(1, 31)*.1, np.zeros(30)]), event)
                for i, tick in enumerate(tick_fixture(event), 1):
                    tick.update(sim_time=str(event["observation"]["sim_time"]+i*.1), x=str(rid-1+i*.1), y="0")
                    robot.append(tick)
            with (source/"robot_state.csv").open("x") as f:
                writer = csv.DictWriter(f, fieldnames=list(robot[0])); writer.writeheader(); writer.writerows(robot)
            config = root/"config.yaml"; config.write_text(yaml.safe_dump(cfg))
            receipt = root/"receipt.json"
            write_json(receipt, {"config_path": str(config), "config_sha256": sha(config), "preflight_geometry_pass": True,
                "front_rgb_visual_review_pass": True, "authorized_run_id": source.name, "created_monotonic_ns": 0,
                "preflight_files_sha256": {}})
            before = {str(p): sha(p) for p in source.rglob("*") if p.is_file()}
            output = root/"analysis"
            with patch("research.static_detour_plots.plot_all"):
                result = analyze(source, output, receipt)
            self.assertTrue(result["source_immutability_verified"])
            self.assertEqual(before, {str(p): sha(p) for p in source.rglob("*") if p.is_file()})
            metadata = json.loads((output/"metadata.json").read_text())
            for path, digest in before.items():
                self.assertEqual(metadata["source_files_sha256"][path], digest)
            self.assertIn("research/analyze_static_detour.py", metadata["source_sha256"])
            world = np.load(output/"derived/request_000001_world.npy")
            np.testing.assert_allclose(world[0], [.1*np.cos(np.radians(179)), .1*np.sin(np.radians(179))])
            with self.assertRaises(FileExistsError):
                analyze(source, output, receipt)
            with self.assertRaises(ValueError):
                analyze(source, source/"forbidden", receipt)
            self.assertFalse((source/"forbidden").exists())


if __name__ == "__main__":
    unittest.main()
