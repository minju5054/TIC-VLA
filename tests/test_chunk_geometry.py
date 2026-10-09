"""Synthetic fixtures validate mathematics only; they are not run evidence."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from research.analyze_chunk_geometry import (
    DEFAULT_CONFIG, characterize_pair, execute, human_at, load_run, manifest,
    nominal_alignment, project_world, tangent_changes, to_local,
)

CFG = json.loads(DEFAULT_CONFIG.read_text())


def request(rid, time, pose, points):
    return {"id": rid, "time": time, "pose": np.array(pose, float), "native": np.array(points, float),
            "command": np.array([1., .1]), "event": {"lookahead_index": 5, "yaw_filter_state": .1}}


def continuous_world(time):
    return np.column_stack([2*np.asarray(time), .3*np.asarray(time)**2])


def fixture(directory):
    """Create explicitly synthetic saved records ONLY under unittest's temp dir."""
    directory.mkdir(); (directory/"raw/requests").mkdir(parents=True); (directory/"diagnostics").mkdir()
    def write(path, data):
        path.write_text(json.dumps(data))
    write(directory/"metadata.json", {"mode": "dynamic", "config": {"simulation": {
        "pause_physics_during_inference": True}, "scene": {"goal": [8, 0, 0]}}})
    write(directory/"summary.json", {"mode": "dynamic", "status": "PASS", "successful_predictions": 8})
    with (directory/"robot_state.csv").open("w") as f:
        writer = csv.writer(f); writer.writerow(["tick", "sim_time", "x", "y", "yaw"])
        for rid in range(1, 9):
            t = (rid-1)*.5; pose = [t, 0, .1*t]; tick = (rid-1)*30
            writer.writerow([tick, t, *pose])
            stem = directory/"raw/requests"/f"request_{rid:06d}"
            image = f"diagnostics/rgb_{rid:06d}.png"
            (directory/image).write_bytes(b"synthetic test placeholder, not measured RGB")
            np.save(stem.with_suffix(".npy"), to_local(continuous_world(t+np.arange(1, 31)*.1), pose))
            write(stem.with_suffix(".json"), {"request_id": rid, "raw_action_shape": [30, 2],
                "native_frame": "observation_body", "native_axes": ["forward", "left"], "native_units": "meters",
                "native_yaw": None, "agent_pose_at_observation": pose, "observation": {"sim_time": t, "tick": tick, "pose": pose},
                "controller_command": [1, .1], "lookahead_index": 5, "yaw_filter_state": .1,
                "rgb_observation_reference": image})
    with (directory/"raw/pedestrian_state.csv").open("w") as f:
        writer = csv.writer(f); writer.writerow(["sim_time", "physx_x", "physx_y", "physx_z"])
        for i in range(1, 9): writer.writerow([i*.5, 2.5, 2-i*.5, 0])


class GeometryTests(unittest.TestCase):
    def test_world_projection(self):
        np.testing.assert_allclose(project_world([[1, 0], [0, 2]], [2, 3, np.pi/2]), [[2, 4], [0, 3]], atol=1e-14)

    def test_nominal_alignment(self):
        shift, residual = nominal_alignment(.5, 30, .1, 1e-6)
        self.assertEqual(shift, 5); self.assertEqual(30-shift, 25); self.assertEqual(residual, 0)
        old = np.arange(1, 31)*.1; fresh = .5+np.arange(1, 31)*.1
        np.testing.assert_allclose(old[shift:], fresh[:30-shift], atol=1e-14)
        with self.assertRaises(ValueError): nominal_alignment(.53, 30, .1, 1e-6)

    def test_frame_and_time_invariance(self):
        # Curved world path; nonzero translations AND different nonzero rotations.
        old_pose, fresh_pose = [3, -2, 2.8], [-1, 4, -.7]
        horizon = np.arange(1, 31)*.1
        old = request(1, 0, old_pose, to_local(continuous_world(horizon), old_pose))
        fresh = request(2, .5, fresh_pose, to_local(continuous_world(.5+horizon), fresh_pose))
        row, _ = characterize_pair(old, fresh, CFG)
        self.assertLess(row["aligned_overlap_rmse_m"], 1e-13)
        self.assertLess(row["mean_abs_tangent_change_deg"], 1e-11)

    def test_known_lateral_revision(self):
        horizon = np.arange(1, 31)*.1; pose = [4, -3, 1.2]
        old = request(1, 0, [0, 0, 0], continuous_world(horizon))
        fresh = request(2, .5, pose, to_local(continuous_world(.5+horizon), pose)+[0, -.4])
        row, _ = characterize_pair(old, fresh, CFG)
        self.assertAlmostEqual(row["aligned_overlap_rmse_m"], .4)
        self.assertAlmostEqual(row["mean_abs_lateral_revision_m"], .4)
        self.assertAlmostEqual(row["max_abs_lateral_revision_m"], .4)
        self.assertAlmostEqual(row["mean_signed_lateral_revision_m"], -.4)
        self.assertLess(row["max_abs_forward_revision_m"], 1e-13)

    def test_tangent_wrap_and_degenerate(self):
        def line(deg): return np.array([[0, 0], [np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))]])
        difference, valid = tangent_changes(line(179), line(-179), 1e-9)
        self.assertTrue(valid[0]); self.assertAlmostEqual(difference[0], 2)
        _, valid = tangent_changes([[0, 0], [0, 0]], line(0), 1e-9)
        self.assertFalse(valid[0])

    def test_raw_immutability_and_output_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); source = root/"synthetic_source"; fixture(source)
            before = manifest(source)
            result = execute(source, root/"analysis", make_figures=False)
            self.assertEqual(result["status"], "PASS")
            self.assertEqual(before, manifest(source))
            with self.assertRaises(FileExistsError): execute(source, root/"analysis", make_figures=False)
            with self.assertRaises(ValueError): execute(source, source/"derived_analysis", make_figures=False)
            self.assertEqual(before, manifest(source))

    def test_invalid_raw_blocks_all_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); source = root/"synthetic_source"; fixture(source)
            for mutation in ["nonfinite", "frame", "duplicate_id", "missing_pose"]:
                p = source/"raw/requests/request_000008.json"; original = p.read_bytes()
                a = source/"raw/requests/request_000008.npy"; original_array = a.read_bytes()
                event = json.loads(original)
                if mutation == "nonfinite": np.save(a, np.full((30, 2), np.nan))
                if mutation == "frame": event["native_frame"] = "application_body"
                if mutation == "duplicate_id": event["request_id"] = 7
                if mutation == "missing_pose": del event["agent_pose_at_observation"]
                p.write_text(json.dumps(event))
                with self.assertRaises((ValueError, KeyError)): execute(source, root/"analysis", make_figures=False)
                self.assertFalse((root/"analysis").exists())
                p.write_bytes(original); a.write_bytes(original_array)

    def test_human_time_matching_without_extrapolation(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"synthetic_source"; fixture(source)
            run = load_run(source, CFG, "dynamic")
            self.assertIsNone(human_at(run, 0)["position"])
            exact = human_at(run, .5)
            self.assertEqual(exact["method"], "exact_sim_time_match")
            self.assertEqual(exact["max_sample_time_offset_s"], 0)
            between = human_at(run, .75)
            np.testing.assert_allclose(between["position"], [2.5, 1.25, 0])
            self.assertEqual(between["max_sample_time_offset_s"], .25)


if __name__ == "__main__":
    unittest.main()
