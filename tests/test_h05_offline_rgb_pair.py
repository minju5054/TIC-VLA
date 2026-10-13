"""Synthetic replay-boundary tests; no model, simulator, render or GPU calls."""
import ast
import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from research.records import write_json
from research.two_hz_preflight import read, digest
from research.h05_offline_rgb_pair import (shared_request, effective_deadline, difference_stats,
    replay_condition, verify, RULES, ORDER, BASE)
from research.analyze_h05_offline_rgb_pair import project, direction, opposed
from research.control import validate_chunk


class OfflinePairTests(unittest.TestCase):
    def test_exact_reference_and_copy_without_action_feedback(self):
        e = read(BASE/'raw/requests/request_000012.json')
        a, b = shared_request(e), shared_request(e)
        self.assertEqual(a, b)
        self.assertEqual(a['observation'], e['observation'])
        self.assertEqual(a['previous_waypoints_text'], e['previous_waypoints_text'])
        a['observation']['position'][0] += 100
        self.assertEqual(b['observation'], e['observation'])
        self.assertNotIn('controller_command', a)

    def test_no_catchup_and_unchanged_nominal_clock(self):
        nominal, guarded = effective_deadline(100., 1., 101., .5)
        self.assertEqual(nominal, 101.)
        self.assertEqual(guarded, 101.5)
        self.assertEqual(effective_deadline(100., 0., None, 0.), (100., 100.))

    def test_mask_conditioned_difference_and_previsible_background(self):
        a = np.zeros((2, 2, 3), np.uint8); b = a.copy(); b[0, 0] = 30
        mask = np.zeros((2, 2), bool)
        result = difference_stats(a, b, mask)
        self.assertIsNone(result['inside_human'])
        self.assertEqual(result['whole']['rgb_mae_0_255'], 7.5)
        self.assertEqual(result['outside_human']['fraction_any_channel_difference_gt10'], .25)
        mask[0, 0] = True; result = difference_stats(a, b, mask)
        self.assertEqual(result['inside_human']['rgb_mae_0_255'], 30.)
        self.assertEqual(result['outside_human']['rgb_mae_0_255'], 0.)

    def test_full_body_to_world_transform(self):
        obs = {'position': [10., 20., 3.], 'quaternion_wxyz': [2**-.5, 0, 0, 2**-.5]}
        np.testing.assert_allclose(project(np.array([[1., 0], [0, 1.]]), obs), [[10, 21], [9, 20]], atol=1e-12)
        self.assertTrue(opposed(direction(.04, .02), direction(-.04, .02)))
        self.assertEqual(direction(.01, .02), 'NEAR_ZERO')

    def test_frozen_hash_rejects_changed_input(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td); p = out/'input.png'; p.write_bytes(b'fixture')
            for name in ['reference-requests.json', 'rgb-audit.json', 'config.json']:
                write_json(out/name, {})
            write_json(out/'protocol.json', {'rules': RULES, 'order': ORDER,
                'input_sha256': {str(p): digest(p)}, 'source_sha256': {}, 'protected_sha256': {},
                'reference_requests_sha256': digest(out/'reference-requests.json'),
                'rgb_audit_sha256': digest(out/'rgb-audit.json'), 'config_sha256': digest(out/'config.json')})
            verify(out); p.write_bytes(b'changed')
            with self.assertRaises(ValueError):
                verify(out)

    def test_two_sessions_fourteen_requests_no_state_reuse_or_motion(self):
        clients = []
        class FakeClient:
            def __init__(self, cfg, records):
                if clients:
                    assert clients[-1].process.returncode == 0
                self.process = SimpleNamespace(pid=100+len(clients), returncode=None)
                self.received = []; clients.append(self)
            def predict(self, request, png):
                self.received.append(copy.deepcopy(request))
                chunk = np.column_stack([np.linspace(.1, 3, 30), np.full(30, .2 if self.process.pid == 100 else -.2)])
                return {'type': 'prediction', 'request_id': request['request_id'],
                        'raw_action': chunk.tolist(), 'robot_state_input': [0.]*6}
            def close(self):
                self.process.returncode = 0
        refs = [{'type': 'predict', 'request_id': i+1, 'observation': {'sim_time': i*.001, 'position': [i, 0, 0]},
                 'instruction': 'fixture', 'previous_waypoints_text': 'saved history'} for i in range(14)]
        original = copy.deepcopy(refs)
        with tempfile.TemporaryDirectory() as td:
            out = Path(td); write_json(out/'protocol.json', {'fixture': True})
            rgb = out/'input.png'; rgb.write_bytes(b'fixture, not an experimental image')
            pairs = [{'paths': {c: str(rgb) for c in ORDER}, 'sha256': {c: digest(rgb) for c in ORDER}}]*14
            with patch('research.h05_offline_rgb_pair.InferenceClient', FakeClient), patch('builtins.print'):
                for c in ORDER:
                    replay_condition(c, out, {'controller': {}}, refs, pairs)
            self.assertEqual(len(clients), 2)
            self.assertEqual(clients[0].received, clients[1].received)
            self.assertEqual(refs, original)
            for c in ORDER:
                files = sorted((out/'raw'/c/'raw/requests').glob('*.json'))
                self.assertEqual(len(files), 14)
                self.assertIsNone(read(files[0])['yaw_filter_before'])
                for f in files:
                    e = read(f); self.assertFalse(e['command_applied']); self.assertIsNone(e['application'])
                    self.assertEqual(e['new_physics_steps'], 0)
                    validate_chunk(np.load(f.with_suffix('.npy'), allow_pickle=False))

    def test_runner_has_no_simulation_execution_entrypoint(self):
        source = Path('research/h05_offline_rgb_pair.py').read_text()
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                self.assertFalse((node.module or '').startswith(('isaacsim', 'omni', 'research.sim', 'research.nova_simulation')))
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, ['step', 'apply', 'render', 'capture', 'update_pose'])


if __name__ == '__main__':
    unittest.main()
