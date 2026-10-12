"""Pure synthetic source-parity checks; fixtures are not navigation evidence."""
import ast
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from research.paperlike_control import (PaperlikeController, SimulationScheduler,
    termination, project_observation, human_path_metrics)
from research.nova_source import BEHAVIOR, slew
from research.records import RunRecords

ROOT = Path(__file__).resolve().parents[1]
CLASS = next(n for n in ast.parse(BEHAVIOR.read_text()).body if isinstance(n, ast.ClassDef) and n.name == "NovaCarterTICVLA")


def method(name):
    return copy.deepcopy(next(n for n in CLASS.body if isinstance(n, ast.FunctionDef) and n.name == name))


def oracle_target(chunk, state):
    # Compile the unchanged official target block directly, excluding only
    # tensor-to-NumPy conversion and subsequent state mutation / logging.
    body = method("_inference_task").body
    begin = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == "T")
    end = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == "should_skip_commands")
    env = {"np": np, "math": math, "self": state, "wps": chunk}
    exec(compile(ast.Module(body=body[begin:end], type_ignores=[]), str(BEHAVIOR), "exec"), env)
    return [env["v_cmd"], env["w_cmd"]], env["j"]


class PaperlikeTests(unittest.TestCase):
    def test_target_exact_source_sequence_and_native_immutability(self):
        c = PaperlikeController()
        reference = SimpleNamespace(_max_linear_velocity=1.5, _max_angular_velocity=1.2)
        for rid, curve in enumerate([0., .4, -.7, 2., -2., .01], 1):
            x = np.linspace(.1, 3, 30)
            chunk = np.column_stack([x, curve*x*x]).astype(np.float32)
            before = chunk.copy()
            expected, index = oracle_target(chunk, reference)
            result = c.accept(chunk, rid, rid*3, [0, 0, 0], rid*3, True)
            np.testing.assert_allclose(result["controller_command"], expected, rtol=0, atol=1e-15)
            self.assertEqual(result["lookahead_index"], index)
            self.assertEqual(c.yaw_filter, reference._yaw_err_filt)
            np.testing.assert_array_equal(chunk, before)

    def test_zero_lookahead_source_parity(self):
        c = PaperlikeController(); z = np.zeros((30, 2), np.float32)
        expected, idx = oracle_target(z, SimpleNamespace(_max_linear_velocity=1.5, _max_angular_velocity=1.2))
        actual = c.accept(z, 1, 0, [0, 0], 0, True)
        self.assertEqual(actual["controller_command"], expected); self.assertEqual(actual["lookahead_index"], idx)

    def test_slew_source_parity_signed_sequences(self):
        fn = method("_slew"); fn.decorator_list = []; env = {}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), str(BEHAVIOR), "exec"), env)
        for accel, decel, dead in [(2., 2.5, .001), (3., 3.5, .0005)]:
            actual = expected = 0.
            for target in [1.5]*20+[-.8]*25+[.8]*20+[0.]*30:
                actual = slew(actual, target, accel, decel, 1/30, dead)
                expected = env["_slew"](expected, target, accel, decel, 1/30, dead)
                self.assertEqual(actual, expected)

    def test_wait_stuck_backup_and_stale_rejection(self):
        c = PaperlikeController()
        self.assertEqual(c.mode, "WAIT_INITIAL"); np.testing.assert_array_equal(c.output(), [0, 0])
        chunk = np.column_stack([np.linspace(.1, 3, 30), np.zeros(30)]).astype(np.float32)
        c.accept(chunk, 1, 0, [0, 0, 0], 0, True)
        c.before_update(149, [0, 0, 0]); self.assertEqual(c.mode, "NORMAL")
        c.before_update(150, [0, 0, 0]); self.assertEqual(c.mode, "BACKUP")
        np.testing.assert_array_equal(c.output(), [-.5, 0])
        c.before_update(449, [0, 0, 0]); self.assertEqual(c.mode, "BACKUP")
        c.before_update(450, [0, 0, 0]); self.assertEqual(c.mode, "WAIT_FRESH")
        np.testing.assert_array_equal(c.output(), [0, 0])
        r = c.accept(chunk, 2, 450, [0, 0, 0], 450, True)
        self.assertTrue(r["stale_rejected"]); self.assertEqual(c.mode, "WAIT_FRESH")
        r = c.accept(chunk, 3, 453, [0, 0, 0], None, True)
        self.assertFalse(r["stale_rejected"]); self.assertEqual(c.mode, "NORMAL")

    def test_stuck_tracking_resets_with_source_threshold(self):
        c = PaperlikeController(); c.first_complete = True
        c.before_update(0, [0, 0]); c.before_update(149, [.1, 0]); c.before_update(150, [.1, 0])
        self.assertEqual(c.mode, "NORMAL"); self.assertEqual(c.last_movement_frame, 149)

    def test_state_constants_from_unchanged_upstream(self):
        values = {ast.unparse(n.targets[0]): ast.literal_eval(n.value) for n in method("on_init").body
                  if isinstance(n, ast.Assign) and isinstance(n.value, (ast.Constant, ast.UnaryOp))}
        self.assertEqual(values["self._stuck_time_threshold_frames"], 150)
        self.assertEqual(values["self._backup_duration_frames"], 300)
        self.assertEqual(values["self._stuck_position_threshold"], .1)
        self.assertEqual(values["self._backup_speed"], -.5)

    def test_scheduler_10hz_no_catchup(self):
        s = SimulationScheduler()
        self.assertEqual([i for i in range(31) if s.due(i)], list(range(0, 31, 3)))
        self.assertTrue(s.due(300)); self.assertFalse(s.due(300)); self.assertFalse(s.due(301))
        self.assertTrue(s.due(303))

    def test_history_index_selection_upstream(self):
        fn = method("_get_sampled_image_paths"); env = {"os": SimpleNamespace(path=SimpleNamespace(exists=lambda p: True, isfile=lambda p: True))}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), str(BEHAVIOR), "exec"), env)
        for length in [1, 30, 31, 60, 61, 91, 200]:
            history = list(range(length))
            expected = env["_get_sampled_image_paths"](SimpleNamespace(_image_history=history))
            self.assertEqual(expected, [history[length-1-n] for n in [90, 60, 30, 0] if length>n])

    def test_official_wheel_constants_installed_controller(self):
        path = Path('/home/gpuadmin/isaacsim/extsDeprecated/isaacsim.robot.wheeled_robots/isaacsim/robot/wheeled_robots/controllers/differential_controller.py')
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
        class Base:
            def __init__(self, name): pass
        env = {"np": np, "BaseController": Base, "ArticulationAction": lambda **kw: SimpleNamespace(**kw)}
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), "exec"), env)
        d = env["DifferentialController"]("oracle", .152, .413)
        for v, w in [(1.5, 0), (0, 1.2), (-.5, 0), (.3, -.7)]:
            actual = d.forward(np.array([v, w], dtype=np.float32)).joint_velocities
            vf, wf = np.float32(v), np.float32(w)
            np.testing.assert_allclose(actual, [(vf-.413*wf/2)/.152, (vf+.413*wf/2)/.152], rtol=1e-7)

    def test_goal_timeout_boundaries_and_upstream_timeout(self):
        self.assertEqual(termination([1.5, 0], [0, 0], 1)[0], "GOAL_REGION")
        self.assertIsNone(termination([1.50001, 0], [0, 0], 84)[0])
        self.assertEqual(termination([2, 0], [0, 0], 85)[0], "TIMEOUT")
        tree = ast.parse((ROOT/"DynaNav/benchmark.py").read_text())
        extension = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == "TIMEOUT_EXTENSION_SECONDS")
        self.assertEqual(extension, 0)

    def test_projection_never_uses_application_pose(self):
        chunk = np.tile([1., 0.], (30, 1)).astype(np.float32)
        e = {"agent_pose_at_observation": [2., 3., math.pi/2], "agent_pose_at_application": [99, 99, -1],
             "native_frame": "observation_body", "native_axes": ["forward", "left"]}
        np.testing.assert_allclose(project_observation(chunk, e), np.tile([2., 4.], (30, 1)))
        e["agent_pose_at_application"] = None
        np.testing.assert_allclose(project_observation(chunk, e), np.tile([2., 4.], (30, 1)))

    def test_human_clearance_checks_segments_not_lateral_endpoint(self):
        m = human_path_metrics([[-2, 0], [2, 2]], [0, 0], 1., 0.)
        self.assertLess(m["minimum_clearance_m"], 0)  # Endpoint alone would appear clear.
        m = human_path_metrics([[-2, 1.2], [3, 1.2]], [0, 0], 1., 0.)
        self.assertAlmostEqual(m["minimum_clearance_m"], .2); self.assertEqual(m["side"], "LEFT")

    def test_existing_bypass_checker_margin_and_blocked_case(self):
        from research.blind_corner_queries import Queries
        q = Queries.__new__(Queries)
        radii = []
        def check(xy, radius):
            radii.append(radius)
            return [{"floor": True, "overlap": []} for _ in xy]
        q.robot_path = check
        result = q.bypass([0, 0], .6069825421953869, 0)
        self.assertTrue(result["pass"]); self.assertAlmostEqual(radii[0], .6569825421953869, places=15)
        q.robot_path = lambda xy, radius: [{"floor": False, "overlap": ["wall"]}]
        self.assertFalse(q.bypass([0, 0], .6069825421953869, 0)["pass"])

    def test_exclusive_outputs_and_raw_immutability(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"run"; r = RunRecords(path, {})
            c = np.zeros((30, 2), np.float32); before = c.copy()
            r.request(1, c, {}); saved = (path/"raw/requests/request_000001.npy").read_bytes()
            with self.assertRaises(FileExistsError): RunRecords(path, {})
            with self.assertRaises(ValueError): r.request(1, c+1, {})
            self.assertEqual(saved, (path/"raw/requests/request_000001.npy").read_bytes())
            np.testing.assert_array_equal(c, before)

    def test_saved_evaluation_reuse_is_identical_and_immutable(self):
        from research.paperlike_analyze import record_evaluation
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/"evaluation.json"
            result = {"stage_a_classification": "PAPERLIKE_NO_HUMAN_CONTROL_FAILED"}
            record_evaluation(p, result)
            before = p.read_bytes()
            record_evaluation(p, result)
            self.assertEqual(before, p.read_bytes())
            with self.assertRaises(ValueError):
                record_evaluation(p, {"stage_a_classification": "CHANGED"})
            self.assertEqual(before, p.read_bytes())


if __name__ == "__main__":
    unittest.main()
