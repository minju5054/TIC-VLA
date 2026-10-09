"""Pure unit fixtures test rules, not experimental navigation evidence."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from research.hospital_episode import official_episode, official_yaw_radians, validate_config, digest
from research.analyze_hospital_turn import (episode_local, native_metrics, phase_selection,
                                           nonfloor_contacts, analyze, characterize, rotation, load_hospital_run)

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT/"configs/research/dynanav_hospital_episode16_static.yaml").read_text())
RULES = CFG["hospital_validation"]


def turn(sign=1):
    a = np.linspace(.02, 1., 30)
    return np.column_stack([np.sin(a), sign*(1-np.cos(a))])


def row(rid, sign=1, degraded=False):
    return {"request_id": rid, "degraded_near_black": degraded,
            **native_metrics(turn(sign), 12, [1., sign*.2], RULES)}


class HospitalTests(unittest.TestCase):
    def test_official_episode_extraction_and_exact_config(self):
        source = validate_config(CFG)
        self.assertEqual(source, official_episode())
        self.assertEqual(source["episode"]["start"], [7.38, 1.49, .01])
        self.assertEqual(source["episode"]["num_people"], 0)
        changed = copy.deepcopy(CFG)
        changed["instruction"] += " turn earlier"
        with self.assertRaises(ValueError):
            validate_config(changed)
        changed = copy.deepcopy(CFG)
        changed["robot"]["start_yaw"] = 90
        with self.assertRaises(ValueError):
            validate_config(changed)

    def test_source_derived_yaw_conversion(self):
        # Known source case: runner RotateZ.Set(90), USD degrees -> +Y world.
        self.assertAlmostEqual(official_yaw_radians(90), math.pi/2)
        np.testing.assert_allclose(episode_local([[0, 1]], [0, 0], official_yaw_radians(90)), [[1, 0]], atol=1e-15)

    def test_episode_local_transform(self):
        np.testing.assert_allclose(episode_local([[7, 4], [5, 2]], [7, 2], math.pi/2), [[2, 0], [0, 2]], atol=1e-15)

    def test_left_right_and_straight_turn_sign(self):
        left, right = row(4, 1), row(4, -1)
        self.assertTrue(left["consistent_left_indicators"])
        self.assertFalse(right["weak_left_evidence"])
        self.assertAlmostEqual(left["derived_total_heading_change_deg"], -right["derived_total_heading_change_deg"])
        straight = native_metrics(np.column_stack([np.arange(1, 31)*.1, np.zeros(30)]), 9, [1, 0], RULES)
        self.assertTrue(straight["forward_dominant"])
        self.assertEqual(straight["derived_total_heading_change_deg"], 0)

    def test_stable_rule_requires_consecutive_and_all_indicators(self):
        self.assertIsNone(phase_selection([row(4), row(5, -1), row(6)], RULES)["first_stable_left_request"])
        result = phase_selection([row(4, -1), row(5), row(6), row(8)], RULES)
        self.assertEqual(result["first_stable_left_request"], 5)
        self.assertEqual(result["stable_left_groups"], [[5, 6]])
        inconsistent = row(6); inconsistent["consistent_left_indicators"] = False
        self.assertIsNone(phase_selection([row(5), inconsistent], RULES)["first_stable_left_request"])

    def test_startup_and_degraded_visibility_exclusion(self):
        self.assertIsNone(phase_selection([row(1), row(2), row(3), row(4)], RULES)["first_stable_left_request"])
        self.assertIsNone(phase_selection([row(4), row(5, degraded=True), row(6)], RULES)["first_stable_left_request"])
        self.assertEqual(phase_selection([row(i) for i in range(1, 6)], RULES)["first_stable_left_request"], 4)

    def test_unwrapped_derived_heading_and_degenerate_segments(self):
        angles = np.radians(np.linspace(170, 190, 29))
        p = np.vstack([[0, 0], np.cumsum(np.column_stack([np.cos(angles), np.sin(angles)]), axis=0)])
        self.assertAlmostEqual(native_metrics(p, 12, [1, .2], RULES)["derived_total_heading_change_deg"], 20)
        result = native_metrics(np.zeros((30, 2)), 12, [0, .2], RULES)
        self.assertIsNone(result["derived_total_heading_change_deg"])
        self.assertFalse(result["consistent_left_indicators"])

    def test_contact_sleep_and_lost_semantics(self):
        event = {"sim_time": 2., "type": "CONTACT_FOUND", "collider0": "/World/Jackal/base/collision",
                 "collider1": "/Root/wall", "contacts": [{"normal": [1, 0, 0]}]}
        spans = nonfloor_contacts([event], "/World/Jackal")
        self.assertEqual(spans[0]["start_sim_time"], 2.)
        self.assertIsNone(spans[0]["end_sim_time"])
        lost = {**event, "sim_time": 5., "type": "CONTACT_LOST", "contacts": []}
        self.assertEqual(nonfloor_contacts([event, lost], "/World/Jackal")[0]["end_sim_time"], 5.)

    def test_measured_execution_separate_from_prediction(self):
        local = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [4.5, .2], [4.7, 1], [4.8, 2]])
        xy = local@rotation(CFG["robot"]["start_yaw"]).T+CFG["robot"]["start_position"][:2]
        events, robot, chunks, diags = [], [], [], []
        for i, p in enumerate(xy):
            pose = [*p.tolist(), math.pi/2+max(0, i-3)*.3]
            events.append({"request_id": i+1, "lookahead_index": 12, "controller_command": [1., .2 if i >= 4 else 0.],
                "agent_pose_at_observation": pose, "observation": {"sim_time": float(i)},
                "application": {"sim_time": i+.1}, "rgb_observation_reference": "fixture.png"})
            robot.append({"sim_time": i, "x": p[0], "y": p[1], "yaw": pose[2], "command_v": 1.})
            chunks.append(turn() if i >= 4 else np.column_stack([np.arange(1, 31)*.1, np.zeros(30)]))
            diags.append({"observation_sim_time": float(i), "degraded_near_black": False,
                          "rgb_p95_0_255": 100, "camera_near_scene_collision_paths": []})
        run = {"cfg": CFG, "events": events, "robot": robot, "chunks": chunks, "path": Path("fixture")}
        latency = {k: .1 for k in ["action_wall_s", "observation_to_switch_wall_s", "robot_translation_m"]}
        with patch("research.analyze_hospital_turn.timing_metrics", return_value=latency):
            result = characterize(run, diags, [])[3]
            self.assertEqual(result["phases"]["first_stable_left_request"], 5)
            self.assertEqual(result["actual_left_interval_groups_by_fresh_id"], [[7, 8]])
            self.assertEqual(result["stage_2_decision"], "TIC-VLA LEFT-TURN BEHAVIOR VALIDATED")
            diags[-1]["degraded_near_black"] = True
            self.assertEqual(characterize(run, diags, [])[3]["stage_2_decision"],
                             "TIC-VLA LEFT-TURN PREDICTION OBSERVED; EXECUTION NOT VALIDATED")

    def test_source_immutability_and_no_overwrite(self):
        # Exercise the complete artifact-writing path with an explicitly mocked
        # characterization result; separate unit tests exercise its numerical rules.
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/"source", Path(directory)/"analysis"
            source.mkdir(); (source/"raw/scene_observations").mkdir(parents=True)
            (source/"sentinel.npy").write_bytes(b"immutable synthetic fixture")
            scene = {"custom_ground_walls_goal_obstacles_created": False,
                     "source_root_world_transform": [1], "composed_root_world_transform": [1],
                     "hospital_url": "fixture", "composed_meters_per_unit": 1,
                     "composed_up_axis": "Z", "source_default_prim": "/Root"}
            for name, data in [("hospital_scene.json", scene), ("hospital_start_audit.json", {"placement_valid": True}),
                               ("raw/scene_observations/rgb.png.json", {})]:
                (source/name).write_text(json.dumps(data))
            (source/"raw/robot_contacts.jsonl").write_text("")
            run = {"cfg": CFG, "events": [{"request_id": 1, "rgb_observation_reference": "rgb.png", "agent_pose_at_observation": [0, 0, 0]}],
                   "metadata": {"official_freeze_receipt": {"config": CFG, "visual_semantic_review_pass": True}},
                   "completion": {"status": "synthetic fixture"}}
            summary = {"phases": {k: None for k in ["first_forward_dominant_request", "last_forward_dominant_request_before_turn", "first_stable_left_request"]},
                       "actual_left_interval_groups_by_fresh_id": []}
            before = {str(p): digest(p) for p in source.rglob("*") if p.is_file()}
            with patch("research.analyze_hospital_turn.load_hospital_run", return_value=run), \
                 patch("research.analyze_hospital_turn.characterize", return_value=([{"request_id": 1}], [np.zeros((30, 2))], [], summary, None)), \
                 patch("research.hospital_turn_plots.plot_all"):
                analyze(source, output)
            self.assertEqual(before, {str(p): digest(p) for p in source.rglob("*") if p.is_file()})
            with self.assertRaises(FileExistsError):
                analyze(source, output)
            with self.assertRaises(ValueError):
                analyze(source, source/"derived")

    def test_partial_run_prefix_is_not_promoted_to_pass(self):
        from test_continuous import event_fixture, tick_fixture
        from research.records import RunRecords, write_json
        import csv
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"run"
            records = RunRecords(source, {"mode": "static", "config": CFG})
            write_json(source/"failure.json", {"status": "FAIL", "error": "RuntimeError('Invalid/blank RGB: fixture')"})
            write_json(source/"inference-runtime.json", {"strict_checkpoint": True, "simulation_app_started": False})
            ticks = []
            for rid in range(1, 4):
                e = event_fixture(rid)
                e["rgb_observation_reference"] = f"diagnostics/rgb_{rid:06d}.png"
                (source/e["rgb_observation_reference"]).write_bytes(b"synthetic unit fixture")
                records.request(rid, turn(), e)
                ticks += tick_fixture(e)
            with (source/"robot_state.csv").open("w") as f:
                writer = csv.DictWriter(f, fieldnames=list(ticks[0])); writer.writeheader(); writer.writerows(ticks)
            result = load_hospital_run(source)
            self.assertEqual(len(result["events"]), 3)
            self.assertEqual(result["completion"]["status"], "INCOMPLETE — ORIGINAL RUN FAILED")
            self.assertEqual(result["completion"]["planned_predictions"], 48)
            (source/"raw/requests/request_000002.npy").unlink()
            with self.assertRaises(ValueError):
                load_hospital_run(source)


if __name__ == "__main__":
    unittest.main()
