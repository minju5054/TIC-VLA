"""One C10 wheelchair at saved XY with the exact original instruction."""
import builtins
import copy
import importlib
from pathlib import Path
import symtable
import unittest
from unittest.mock import patch
from tools.c10_wheelchair_only.scene import INSTRUCTION, scenario_config


class C10WheelchairOnlyConfig(unittest.TestCase):
    def config(self):
        return {'instruction': INSTRUCTION, 'stationary_human': {'prim': '/old', 'radius_m': .25},
                'controller': {'max_v': 1.5}}

    def reader(self, path):
        if path.name == 'metadata.json':
            return {'config': {'scene': {}}}
        rid = int(path.stem.split('_')[-1]); pose = [rid * .7, rid * .2, .1]
        return {'request_id': rid, 'agent_pose_at_observation': pose, 'observation': {'pose': pose}}

    def test_only_c10_exact_original_xy_and_input_unchanged(self):
        cfg = self.config(); before = copy.deepcopy(cfg)
        with patch('tools.c10_wheelchair_only.scene.read', side_effect=self.reader), patch('tools.exact_two_humans.scene.read', side_effect=self.reader):
            out = scenario_config(Path('/r'), cfg, Path('outputs/baseline'))
        self.assertNotIn('fixed_humans', out)
        self.assertEqual(out['fixed_wheelchair']['id'], 'C10')
        self.assertEqual(out['fixed_wheelchair']['position'], [10*.7, 10*.2, 0.])
        self.assertEqual(out['fixed_wheelchair']['xy_offset_m'], [0., 0.])
        self.assertEqual(out['instruction'], INSTRUCTION)
        self.assertNotIn('stationary_human', out)
        self.assertEqual(out['controller'], before['controller'])
        self.assertEqual(cfg, before)

    def test_avoidance_suffix_or_extra_actor_rejected(self):
        for key, value in [('instruction', INSTRUCTION+' If an obstacle blocks your path, navigate around it.'),
                           ('fixed_wheelchair', {}), ('fixed_humans', []), ('stationary_cart', {})]:
            cfg = self.config(); cfg[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                scenario_config(Path('/r'), cfg, Path('outputs/baseline'))

    def test_obstructed_reference_rejected(self):
        with patch('tools.c10_wheelchair_only.scene.read', return_value={'config': {'fixed_wheelchair': {}}}):
            with self.assertRaisesRegex(ValueError, 'no added obstacle'):
                scenario_config(Path('/r'), self.config(), Path('outputs/baseline'))

    def test_inconsistent_saved_observation_rejected(self):
        def reader(path):
            value = self.reader(path)
            if path.name == 'request_000010.json':
                value['observation']['pose'] = [0., 0., 0.]
            return value
        with patch('tools.c10_wheelchair_only.scene.read', side_effect=reader), patch('tools.exact_two_humans.scene.read', side_effect=reader):
            with self.assertRaisesRegex(ValueError, 'Invalid saved coordinate reference'):
                scenario_config(Path('/r'), self.config(), Path('outputs/baseline'))

    def test_no_unbound_runtime_globals(self):
        for name in ('runner', 'scene', 'analysis'):
            mod = importlib.import_module('tools.c10_wheelchair_only.'+name); path = Path(mod.__file__)
            available = set(vars(mod)) | set(vars(builtins)); missing = []
            def check(scope):
                if scope.get_type() == 'function':
                    missing.extend((scope.get_name(), s.get_name()) for s in scope.get_symbols()
                                   if s.is_global() and s.is_referenced() and s.get_name() not in available)
                for child in scope.get_children():
                    check(child)
            check(symtable.symtable(path.read_text(), str(path), 'exec'))
            self.assertEqual(missing, [], name)


if __name__ == '__main__':
    unittest.main()
