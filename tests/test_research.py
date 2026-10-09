import ast
import math
from pathlib import Path
import socket
import struct
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from research.control import wheel_speeds, waypoint_command, validate_chunk, rotation_wxyz
from research.records import RunRecords
from research.ipc import send, receive, MAX_JSON
from research.pedestrian import crossing_position


class KinematicsTests(unittest.TestCase):
    def test_straight(self):
        np.testing.assert_allclose(wheel_speeds(.4, 0, .098, .37559), [.4/.098]*4)

    def test_positive_turn(self):
        value = .37559*.6/(2*.098)
        np.testing.assert_allclose(wheel_speeds(0, .6, .098, .37559), [-value, -value, value, value])

    def test_arbitrary_and_joint_signs(self):
        np.testing.assert_allclose(wheel_speeds(.3, -.4, .1, .5), [4, 4, 2, 2])
        np.testing.assert_allclose(wheel_speeds(.3, -.4, .1, .5, [-1, -1, 1, 1]), [-4, -4, 2, 2])

    def test_camera_axes(self):
        rotation = rotation_wxyz([.5, .5, -.5, -.5])
        np.testing.assert_allclose(rotation@[0, 0, -1], [1, 0, 0])
        np.testing.assert_allclose(rotation@[0, 1, 0], [0, 0, 1])


class ControllerTests(unittest.TestCase):
    def test_original_source_parity(self):
        # Execute the unchanged upstream algorithm slice as the reference. No Isaac imports.
        path = Path(__file__).resolve().parents[1]/"DynaNav/behavior/nova_carter_test_ticvla.py"
        tree = ast.parse(path.read_text())
        original = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_inference_task")
        def assigns(node, name):
            return isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets)
        begin = next(i for i, n in enumerate(original.body) if assigns(n, "wps")) + 1
        end = next(i for i, n in enumerate(original.body) if assigns(n, "should_skip_commands"))
        function = ast.parse("def reference(self, wps):\n    pass").body[0]
        function.body = original.body[begin:end] + ast.parse("return v_cmd, w_cmd, j").body
        namespace = {"np": np, "math": math}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), str(path), "exec"), namespace)
        obj = SimpleNamespace(_max_linear_velocity=1.5, _max_angular_velocity=1.2)
        rng = np.random.default_rng(7)
        state = None
        cases = [np.zeros((30, 2)), np.stack([np.linspace(.1, 3, 30), np.zeros(30)], axis=1)]
        cases += [rng.normal(size=(30, 2)).astype(np.float32) for _ in range(20)]
        for chunk in cases:
            expected = namespace["reference"](obj, chunk)
            command, state, index = waypoint_command(chunk, state)
            np.testing.assert_allclose(command, expected[:2], atol=1e-12)
            self.assertEqual(index, expected[2])

    def test_common_origin_not_integrated(self):
        chunk = np.tile([.2, 0.0], (30, 1))
        _, _, index = waypoint_command(chunk)
        self.assertEqual(index, 27) # Zero arc length, no invented origin/point increments.


class RecordsTests(unittest.TestCase):
    def test_shape_finite(self):
        for bad in [np.zeros((1, 30, 2)), np.zeros((30, 3)), np.zeros((29, 2)), np.ones((30, 2), dtype=int), np.full((30, 2), np.nan)]:
            with self.assertRaises(ValueError):
                validate_chunk(bad)

    def test_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"run"
            store = RunRecords(path, {})
            a = np.arange(60, dtype=np.float32).reshape(30, 2)
            store.request(1, a, {})
            before = (path/"raw/requests/request_000001.npy").read_bytes()
            with self.assertRaises(ValueError):
                store.request(1, a+1, {})
            with self.assertRaises(FileExistsError):
                RunRecords(path, {})
            self.assertEqual(before, (path/"raw/requests/request_000001.npy").read_bytes())
            np.testing.assert_array_equal(np.load(path/"raw/requests/request_000001.npy"), a)


class ProtocolTests(unittest.TestCase):
    def test_roundtrip_and_bounds(self):
        a, b = socket.socketpair()
        try:
            send(a, {"request_id": 3}, b"RGB")
            self.assertEqual(receive(b), ({"request_id": 3}, b"RGB"))
            a.sendall(struct.pack("!II", MAX_JSON+1, 0))
            with self.assertRaises(ValueError):
                receive(b)
        finally:
            a.close(); b.close()


class CrossingTests(unittest.TestCase):
    def test_sim_time_only(self):
        config = {"start_position": [2.5, 2, 0], "velocity": [0, -1, 0], "start_time": .5, "stop_time": 4.5}
        np.testing.assert_allclose(crossing_position(config, 0), [2.5, 2, 0])
        np.testing.assert_allclose(crossing_position(config, 2.5), [2.5, 0, 0])
        np.testing.assert_allclose(crossing_position(config, 100), [2.5, -2, 0])


if __name__ == "__main__":
    unittest.main()
