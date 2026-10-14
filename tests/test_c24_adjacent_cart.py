import builtins
import importlib
import itertools
import symtable
import unittest
from pathlib import Path
import numpy as np
from tools.c24_adjacent_cart.scene import cart_pose
from tools.c24_adjacent_cart.analysis import passage


class AdjacentCartTest(unittest.TestCase):
    def test_rotated_asymmetric_asset_is_grounded_and_separated(self):
        bounds = np.array([[-.46,-.26,0.03],[.64,.27,1.14]])
        chair = {'center_world_xyz': [-1.7,11.,0.], 'world_bounds': [[-2.3,10.5,0.],[-1.1,11.45,1.16]]}
        corners = np.array(list(itertools.product(*zip(*bounds))))
        for yaw in (-2.986, 0., 1.2):
            pose = cart_pose(bounds, chair, yaw)
            points = (np.column_stack([corners, np.ones(8)])@pose['matrix_row_vector'])[:,:3]
            lo, hi = points.min(0), points.max(0)
            self.assertAlmostEqual(lo[2], 0.)
            self.assertAlmostEqual((lo[0]+hi[0])/2, -1.7)
            self.assertAlmostEqual(lo[1]-11.45, .05)
            np.testing.assert_allclose((lo[:2]+hi[:2])/2, pose['center_world_xyz'][:2])
            self.assertGreater(pose['center_world_xyz'][1], 11.4)

    def test_left_for_westward_motion_is_world_negative_y(self):
        t = np.array([0.,1.,2.])
        for y, side in [(-1., 'left'), (1., 'right')]:
            xy = np.array([[2.,y],[0.,y],[-2.,y]])
            result = passage(t, xy, [0.,0.], np.pi, .8)
            self.assertEqual(result['first_centre_plane_crossing']['side'], side)
            self.assertAlmostEqual(result['first_centre_plane_crossing']['sim_s'], 1.)
            self.assertEqual(result['fully_passed_sim_s'], 2.)

    def test_turning_back_without_crossing_is_not_passage(self):
        t = np.arange(4.)
        xy = np.array([[3.,0.],[2.,-1.],[1.,-1.],[2.,0.]])
        result = passage(t, xy, [0.,0.], np.pi, .8)
        self.assertIsNone(result['first_centre_plane_crossing'])
        self.assertIsNone(result['fully_passed_sim_s'])

    def test_all_grounding_images_share_one_exclusive_manifest(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from tools.c24_adjacent_cart.scene import grounding_views
        cfg = {'fixed_wheelchairs': [{'id': 'C10'}, {'id': 'C24', 'reference_tangent_rad': -2.986}]}
        sim = SimpleNamespace(pedestrian=SimpleNamespace(cart={'center_world_xyz': [-1.7,11.8,0.]}))
        with patch('tools.c24_adjacent_cart.scene.base.grounding_views') as render:
            grounding_views(sim, cfg)
        render.assert_called_once()
        self.assertEqual([w['id'] for w in render.call_args.args[1]['fixed_wheelchairs']], ['C10','C24','C24_cart'])
        self.assertEqual(len(cfg['fixed_wheelchairs']), 2)

    def test_no_unbound_runtime_globals(self):
        for name in ('scene', 'runner', 'analysis'):
            mod = importlib.import_module('tools.c24_adjacent_cart.'+name)
            path = Path(mod.__file__); known = set(vars(mod))|set(vars(builtins)); missing = []
            def check(scope):
                if scope.get_type() == 'function':
                    missing.extend(s.get_name() for s in scope.get_symbols() if s.is_global() and s.is_referenced() and s.get_name() not in known)
                for child in scope.get_children(): check(child)
            check(symtable.symtable(path.read_text(), str(path), 'exec'))
            self.assertEqual(missing, [], name)


if __name__ == '__main__':
    unittest.main()
