"""Two C10/C24 wheelchairs at saved XY with the exact original instruction."""
import builtins
import copy
import importlib
from pathlib import Path
import symtable
import unittest
from unittest.mock import patch
from tools.two_wheelchairs_slow.scene import INSTRUCTION, scenario_config


class TwoWheelchairsSlowConfig(unittest.TestCase):
    def config(self):
        return {'instruction': INSTRUCTION, 'stationary_human': {'prim': '/old', 'radius_m': .25},
                'controller': {'max_v': 1.5, 'max_w': 1.2, 'lookahead': 1.0}, 'simulation': {'predictions': 48, 'inference_hz': 2}}

    def reader(self, path):
        if path.name == 'metadata.json':
            return {'config': {'scene': {}}}
        rid = int(path.stem.split('_')[-1]); pose = [rid * .7, rid * .2, .1]
        return {'request_id': rid, 'agent_pose_at_observation': pose, 'observation': {'pose': pose}}

    def test_two_exact_original_xy_and_input_unchanged(self):
        cfg = self.config(); before = copy.deepcopy(cfg)
        with patch('tools.two_wheelchairs_slow.scene.read', side_effect=self.reader), patch('tools.exact_two_humans.scene.read', side_effect=self.reader):
            out = scenario_config(Path('/r'), cfg, Path('outputs/baseline'))
        self.assertNotIn('fixed_humans', out)
        self.assertNotIn('fixed_wheelchair', out)
        self.assertEqual([w['id'] for w in out['fixed_wheelchairs']], ['C10', 'C24'])
        for rid,w in zip((10,24),out['fixed_wheelchairs']):
            self.assertEqual(w['position'], [rid*.7, rid*.2, 0.])
            self.assertEqual(w['xy_offset_m'], [0.,0.])
            self.assertEqual(w['label'], 'stationary_wheelchair_c'+str(rid))
        self.assertEqual(out['instruction'], INSTRUCTION)
        self.assertNotIn('stationary_human', out)
        self.assertEqual(out['controller'], {**before['controller'], 'max_v':1.2})
        self.assertEqual(out['simulation'], {**before['simulation'], 'predictions':60})
        self.assertEqual(cfg, before)

    def test_avoidance_suffix_or_extra_actor_rejected(self):
        for key, value in [('instruction', INSTRUCTION+' If an obstacle blocks your path, navigate around it.'),
                           ('fixed_wheelchair', {}), ('fixed_wheelchairs', []), ('fixed_humans', []), ('stationary_cart', {})]:
            cfg = self.config(); cfg[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                scenario_config(Path('/r'), cfg, Path('outputs/baseline'))

    def test_obstructed_reference_rejected(self):
        with patch('tools.two_wheelchairs_slow.scene.read', return_value={'config': {'fixed_wheelchair': {}}}):
            with self.assertRaisesRegex(ValueError, 'no added obstacle'):
                scenario_config(Path('/r'), self.config(), Path('outputs/baseline'))

    def test_inconsistent_saved_observation_rejected(self):
        def reader(path):
            value = self.reader(path)
            if path.name == 'request_000010.json':
                value['observation']['pose'] = [0., 0., 0.]
            return value
        with patch('tools.two_wheelchairs_slow.scene.read', side_effect=reader), patch('tools.exact_two_humans.scene.read', side_effect=reader):
            with self.assertRaisesRegex(ValueError, 'Invalid saved coordinate reference'):
                scenario_config(Path('/r'), self.config(), Path('outputs/baseline'))

    def test_lowercase_render_labels_keep_each_object_separate(self):
        import numpy as np
        from research.nova_occlusion import segmentation_stats
        with patch('tools.two_wheelchairs_slow.scene.read',side_effect=self.reader), patch('tools.exact_two_humans.scene.read',side_effect=self.reader):
            cfg=scenario_config(Path('/r'),self.config(),Path('outputs/baseline'))
        labels={'0':{'class':'BACKGROUND'},'3':{'class':'stationary_wheelchair_c10'},'4':{'class':'stationary_wheelchair_c24'}}
        raster=np.array([[0,3,3],[4,0,4]])
        for w,expected in zip(cfg['fixed_wheelchairs'],(3,4)):
            stats,mask=segmentation_stats(raster,labels,w['label'])
            self.assertEqual(stats['semantic_ids'],[expected])
            self.assertEqual(stats['human_visible_pixel_count'],2)
            np.testing.assert_array_equal(mask,raster==expected)

    def test_controller_limit_preserves_lookahead_and_yaw_filter(self):
        import numpy as np
        from research.control import waypoint_command
        for angle in (0., .15, -.35, 1.2):
            chunk=np.column_stack((np.linspace(.1,3.,30), np.linspace(.1,3.,30)*angle))
            original,f0,j0=waypoint_command(chunk,max_v=1.5)
            slow,f1,j1=waypoint_command(chunk,max_v=1.2)
            self.assertLessEqual(slow[0],1.2)
            self.assertLessEqual(slow[0],original[0])
            self.assertEqual((f0,j0),(f1,j1))
        self.assertEqual(waypoint_command(np.column_stack((np.linspace(.1,3.,30),np.zeros(30))),max_v=1.2)[0],(1.2,0.))

    def test_changed_original_speed_rejected(self):
        cfg=self.config();cfg['controller']['max_v']=2.
        with patch('tools.two_wheelchairs_slow.scene.read',side_effect=self.reader), patch('tools.exact_two_humans.scene.read',side_effect=self.reader):
            with self.assertRaisesRegex(ValueError,'Expected original'):
                scenario_config(Path('/r'),cfg,Path('outputs/baseline'))

    def test_no_unbound_runtime_globals(self):
        for name in ('runner', 'scene', 'analysis'):
            mod = importlib.import_module('tools.two_wheelchairs_slow.'+name); path = Path(mod.__file__)
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
