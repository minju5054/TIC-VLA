"""Synthetic evaluator/adapter checks; these are not navigation evidence."""
import copy
import unittest
import numpy as np
from research.h05_stationary_run import adapt, SELECTION
from research.h05_stationary_analysis import classify_prediction, world_curve, crossing, CLASSES
from research.two_hz_preflight import read, SEPARATION


class H05Tests(unittest.TestCase):
    def test_only_two_additive_config_keys_and_exact_pose(self):
        selection = read(SELECTION); before = copy.deepcopy(selection)
        cfg = adapt(selection)
        self.assertEqual(selection, before)
        self.assertEqual(set(cfg)-set(selection['baseline_config']), {'stationary_human', 'occlusion'})
        self.assertEqual({k: cfg[k] for k in selection['baseline_config']}, selection['baseline_config'])
        human = cfg.pop('stationary_human')
        self.assertEqual(human.pop('position'), selection['human']['start_position'])
        self.assertEqual(human, selection['human'])
        self.assertFalse(selection['experiment_authorized'])

    def test_continuous_collision_between_safe_endpoints(self):
        result = classify_prediction(np.array([[2., 0], [-3., 0]]), [3., 0], [0., 0])
        self.assertEqual(result['classification'], CLASSES[0])
        self.assertEqual(result['minimum_center_m'], 0.)

    def test_horizon_prefix_and_complete_are_distinct(self):
        for end, expected in [(2., CLASSES[1]), (-1.5, CLASSES[2]), (-3., CLASSES[3])]:
            result = classify_prediction(np.array([[4., 1.], [end, 1.]]), [5., 1.], [0., 0])
            self.assertEqual(result['classification'], expected)
        result = classify_prediction(np.array([[4., 1.], [-SEPARATION-.99, 1.]]), [5., 1.], [0., 0])
        self.assertEqual(result['classification'], CLASSES[2])

    def test_origin_connector_cannot_jump_over_human(self):
        result = classify_prediction(np.array([[-2., 1.], [-3., 1.]]), [3., 0.], [0., 0.])
        self.assertEqual(result['classification'], CLASSES[0])
        self.assertGreater(result['native_only_minimum_surface_clearance_m'], .05)

    def test_observation_body_to_world_rotation_and_translation(self):
        observation = {'position': [10., 20., 2.], 'quaternion_wxyz': [2**-.5, 0, 0, 2**-.5]}
        world = world_curve(np.array([[1., 0.], [0., 1.]]), observation)
        np.testing.assert_allclose(world, [[10., 21.], [9., 20.]], atol=1e-12)

    def test_crossing_is_westward_and_side_has_explicit_sign(self):
        self.assertEqual(crossing([[1, 1], [-1, 1]], [0, 0])['side'], 'RIGHT')
        self.assertEqual(crossing([[1, -1], [-1, -1]], [0, 0])['side'], 'LEFT')
        self.assertIsNone(crossing([[-1, 1], [1, 1]], [0, 0]))


if __name__ == '__main__':
    unittest.main()
