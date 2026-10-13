"""Configuration isolation/immutability and runtime symbol guards; no Isaac."""
import builtins
import copy
import importlib
import json
from pathlib import Path
import symtable
import tempfile
import unittest

from tools.h05_stationary_obstacles.obstacles import scenario_config


class SeparateScenarios(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.reference = Path('outputs/prior')
        base = self.root/self.reference
        (base/'derived/api-smoke').mkdir(parents=True)
        (base/'raw/requests').mkdir(parents=True)
        self.cfg = {'simulation': {'headless': True}, 'stationary_human': {
            'position': [-2.4, 10.35, 0.], 'start_position': [-2.4, 10.35, 0.],
            'motion': 'stationary', 'speed_m_s': 0., 'radius_m': .25}}
        (base/'runtime-config.json').write_text(json.dumps(self.cfg))
        (base/'derived/api-smoke/summary.json').write_text(json.dumps({
            'runtime_pass': True, 'goal_entered_strict_2m': True, 'nonfloor_contact_spans': 0}))
        (base/'raw/requests/request_000024.json').write_text(json.dumps({
            'agent_pose_at_observation': [-1.5, 11.35, -2.9], 'observation': {'tick': 100}}))
        frozen = json.loads((Path(__file__).resolve().parents[1]/'outputs/e16-cart-preflight-20261010-01/freeze.json').read_text())
        target = self.root/'outputs/e16-cart-preflight-20261010-01'
        target.mkdir(); (target/'freeze.json').write_text(json.dumps(frozen))

    def tearDown(self):
        self.tmp.cleanup()

    def test_separate_single_actors_at_identical_center(self):
        original = copy.deepcopy(self.cfg)
        human = scenario_config(self.root, self.cfg, self.reference, 'human')
        cart = scenario_config(self.root, self.cfg, self.reference, 'cart')
        self.assertNotIn('stationary_cart', human)
        self.assertNotIn('stationary_human', cart)
        self.assertEqual(human['stationary_human']['position'], cart['stationary_cart']['cart']['center_world_xyz'])
        self.assertEqual(human['stationary_human']['motion'], 'stationary')
        self.assertEqual(human['stationary_human']['speed_m_s'], 0.)
        self.assertEqual(self.cfg, original)

    def test_changed_baseline_settings_rejected(self):
        changed = copy.deepcopy(self.cfg); changed['instruction'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'configuration changed'):
            scenario_config(self.root, changed, self.reference, 'cart')

    def test_combined_scenario_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Exactly one'):
            scenario_config(self.root, self.cfg, self.reference, 'both')

    def test_no_unresolved_runtime_global_names(self):
        for name in ('runner', 'obstacles', 'analysis'):
            mod = importlib.import_module('tools.h05_stationary_obstacles.'+name)
            path = Path(mod.__file__)
            available = set(vars(mod)) | set(vars(builtins))
            missing = []
            def check(scope):
                if scope.get_type() == 'function':
                    missing.extend((scope.get_name(), s.get_name()) for s in scope.get_symbols()
                        if s.is_global() and s.is_referenced() and s.get_name() not in available)
                for child in scope.get_children(): check(child)
            check(symtable.symtable(path.read_text(), str(path), 'exec'))
            self.assertEqual(missing, [], name)


if __name__ == '__main__':
    unittest.main()
