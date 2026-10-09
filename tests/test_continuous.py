"""Synthetic unit fixtures are not runtime transport evidence; no Isaac imports."""
import copy
import csv
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from research.continuous import OneRequest, ActiveControl, PhysicsPacer, next_observation_time
from research.analyze_handoffs import metrics, validate_pending, analyze
from research.records import RunRecords
from research.loop import run_model_loop

REPO = Path(__file__).resolve().parents[1]


def event_fixture(rid=2):
    base = rid * 1_000_000_000
    def state(offset, sim_time, tick, position, yaw=0):
        return {"wall_time_ns": base+offset, "monotonic_ns": base+offset,
                "sim_time": sim_time, "tick": tick, "position": position,
                "pose": [*position[:2], yaw], "event_source": "synthetic_unit_fixture"}
    obs = state(0, .5*(rid-1), (rid-1)*5, [rid-1, 0, 0], math.radians(179))
    det = state(250_000_000, obs["sim_time"]+.2, obs["tick"]+2, [rid-1+.3, .4, 0], math.radians(-179))
    app = {**det, "wall_time_ns": base+260_000_000, "monotonic_ns": base+260_000_000}
    command, wheels = [1.5, 0.0], [15.0]*4
    return {"request_id": rid, "bootstrap": rid == 1,
            "old_control_source_request_id": rid-1 if rid > 1 else None,
            "old_controller_command": command if rid > 1 else None,
            "old_wheel_commands": wheels if rid > 1 else None,
            "observation": obs, "request_submit": state(10_000_000, 0, 0, [0]*3),
            "inference_start": state(50_000_000, 0, 0, [0]*3),
            "ready": state(200_000_000, 0, 0, [0]*3), "response_detected": det,
            "application": app, "agent_pose_at_observation": obs["pose"],
            "agent_pose_at_application": app["pose"], "agent_pose_at_ready": None,
            "controller_command": command, "wheel_commands": wheels,
            "native_tensor_shape": [1, 30, 2],
            "human_at_observation": None, "human_at_response_detected": None, "human_at_application": None}


def tick_fixture(event):
    rows = []
    for i in (1, 2):
        mono = event["observation"]["monotonic_ns"] + i*80_000_000
        row = {"tick": str(event["observation"]["tick"]+i), "monotonic_ns": str(mono),
               "physics_step_start_monotonic_ns": str(mono-10_000_000),
               "pending_request_id": str(event["request_id"]),
               "active_control_source_request_id": str(event["old_control_source_request_id"] or "")}
        targets = [0.0]*6 if event["bootstrap"] else [*event["old_controller_command"], *event["old_wheel_commands"]]
        row.update(zip(("command_v", "command_w", "command_fl", "command_rl", "command_fr", "command_rr"), map(str, targets)))
        rows.append(row)
    return rows


class ContinuousTests(unittest.TestCase):
    def test_one_outstanding_including_ready_unconsumed(self):
        entered, release = threading.Event(), threading.Event()
        main_thread = threading.get_ident()
        class Client:
            def predict(self, request, png):
                assert threading.get_ident() != main_thread
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test release timeout")
                return request
            def close(self):
                release.set()
        pending = OneRequest(Client())
        try:
            pending.submit({"request_id": 1}, b"unit fixture")
            self.assertTrue(entered.wait(2))
            with self.assertRaises(RuntimeError):
                pending.submit({"request_id": 2}, b"")
            with self.assertRaises(RuntimeError):
                pending.take()
            release.set()
            pending.future.result(timeout=2)
            with self.assertRaises(RuntimeError):
                pending.submit({"request_id": 2}, b"")
            self.assertEqual(pending.take()["request_id"], 1)
            pending.submit({"request_id": 2}, b"")
            pending.future.result(timeout=2)
            self.assertEqual(pending.take()["request_id"], 2)
        finally:
            pending.close()

    def test_old_retention_bootstrap_and_pending_audit(self):
        active = ActiveControl()
        self.assertIsNone(active.old_record()["old_control_source_request_id"])
        self.assertIsNone(active.old_record()["old_controller_command"])
        active.accept(1, [1.5, 0.0], [15.0]*4)
        before = copy.deepcopy(active.old_record())
        for _ in range(3):
            self.assertEqual(active.old_record(), before)  # Polling cannot accept a FRESH.
        event = event_fixture()
        self.assertEqual(active.old_record()["old_controller_command"], event["old_controller_command"])
        rows = tick_fixture(event)
        self.assertEqual(validate_pending(event, rows)["physics_ticks_wholly_inside_action_call"], 2)
        rows[0]["command_w"] = "0.1"
        with self.assertRaises(ValueError):
            validate_pending(event, rows)
        active.accept(2, [1.4, .1], [14.0]*4)
        self.assertEqual(active.old_record()["old_control_source_request_id"], 2)
        self.assertEqual(before["old_control_source_request_id"], 1)

    def test_timing_and_se2_wrap(self):
        m = metrics(event_fixture())
        self.assertAlmostEqual(m["action_wall_s"], .15)
        self.assertAlmostEqual(m["observation_to_switch_wall_s"], .26)
        self.assertAlmostEqual(m["pending_sim_s"], .2)
        self.assertAlmostEqual(m["achieved_rtf"], .8)
        self.assertAlmostEqual(m["robot_translation_m"], .5)
        self.assertAlmostEqual(m["robot_yaw_change_deg"], 2)
        self.assertEqual(m["pending_physics_ticks"], 2)

    def test_physics_pacing_only_no_catchup(self):
        now, waits = [0.0], []
        def sleep(duration):
            waits.append(duration)
            now[0] += duration
        pacer = PhysicsPacer(.1, 1, now=lambda: now[0], sleep=sleep)
        def step(duration):
            now[0] += duration
        pacer.step(lambda: step(.02))
        self.assertAlmostEqual(waits[0], .08)
        pacer.step(lambda: step(.2))
        self.assertEqual(len(waits), 1)
        pacer.step(lambda: step(.01))
        self.assertAlmostEqual(waits[1], .09)  # No catch-up compression.
        self.assertEqual(next_observation_time(0, .2, .5), .5)
        self.assertEqual(next_observation_time(0, .8, .5), .8)

    def test_config_preserves_scenario_and_frozen_path(self):
        frozen = yaml.safe_load((REPO/"configs/research/crossing_pedestrian.yaml").read_text())
        continuous = yaml.safe_load((REPO/"configs/research/crossing_pedestrian_continuous.yaml").read_text())
        continuous["simulation"].pop("real_time_pacing")
        continuous["simulation"].pop("target_real_time_factor")
        continuous["simulation"]["pause_physics_during_inference"] = True
        self.assertEqual(continuous, frozen)

    def test_frozen_execution_regression(self):
        cfg = {"simulation": {"pause_physics_during_inference": True, "control_hz": 10,
                              "inference_hz": 2, "predictions": 4}, "controller": {}, "instruction": "test"}
        with tempfile.TemporaryDirectory() as directory:
            records = RunRecords(Path(directory)/"run", {})
            class Sim:
                dt, pedestrian = .1, None
                def __init__(self):
                    self.tick, self.x, self.command = 0, 0.0, (0, 0)
                def state(self):
                    return {"tick": self.tick, "sim_time": self.tick*self.dt,
                            "position": [self.x, 0, 0], "pose": [self.x, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]}
                def capture(self, name):
                    path = records.path/"diagnostics"/name
                    path.write_bytes(b"synthetic unit fixture")
                    return None, self.state(), path
                def apply(self, *command):
                    self.command = command
                    return [0.0]*4
                def step(self):
                    self.x += self.command[0]*self.dt
                    self.tick += 1
                    return self.state()
            sim = Sim()
            class Client:
                def __init__(self, *args): pass
                def predict(self, request, png):
                    assert sim.tick == (request["request_id"]-1)*5
                    assert sim.state() == request["observation"]
                    return {"raw_action": np.column_stack([np.arange(1, 31)*.1, np.zeros(30)]).tolist()}
                def close(self): pass
            with patch("research.loop.InferenceClient", Client):
                result = run_model_loop(sim, cfg, records, "static")
            self.assertEqual(result["successful_predictions"], 4)
            self.assertEqual(sim.tick, 20)
            for path in (records.path/"raw/requests").glob("*.json"):
                event = json.loads(path.read_text())
                self.assertEqual(event["observation"]["sim_time"], event["application"]["sim_time"])

    def test_saved_analysis_anchor_immutability_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)/"synthetic"
            records = RunRecords(run, {"mode": "static", "config": {"simulation": {
                "pause_physics_during_inference": False, "predictions": 4}}})
            (run/"inference-runtime.json").write_text(json.dumps({"strict_checkpoint": True, "simulation_app_started": False}))
            rows = []
            chunk = np.tile([1.0, 0.0], (30, 1))
            for rid in range(1, 5):
                event = event_fixture(rid)
                event["rgb_observation_reference"] = f"diagnostics/rgb_{rid}.png"
                (run/event["rgb_observation_reference"]).write_bytes(b"synthetic unit fixture")
                records.request(rid, chunk, event)
                rows += tick_fixture(event)
            with (run/"robot_state.csv").open("w") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            before = {p: p.read_bytes() for p in run.rglob("*") if p.is_file()}
            with patch("research.analyze_handoffs.plot"):
                summary = analyze(run)
            self.assertTrue(summary["continuous_handoff_validated"])
            self.assertEqual(summary["number_of_nonbootstrap_handoffs"], 3)
            projected = np.load(run/"derived/handoff/request_000001_world.npy")
            np.testing.assert_allclose(projected[0], [math.cos(math.radians(179)), math.sin(math.radians(179))])
            for path, data in before.items():
                self.assertEqual(path.read_bytes(), data)
            with self.assertRaises(FileExistsError):
                analyze(run)
            with self.assertRaises(FileExistsError):
                RunRecords(run, {})


if __name__ == "__main__":
    unittest.main()
