"""Synthetic rule tests only; rendered and model evidence remain separate."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from research.obstacle_reveal import cart_at, temporal_proxy, reveal_decision, predicted_passage, densify
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_chunk_geometry import project_world


class ObstacleRevealTests(unittest.TestCase):
    def cart(self):
        return {'center_world_xyz': [3.,0.,0.], 'yaw_rad': 0., 'tangent_rad': 0., 'footprint_half_m': [.5,.25]}

    def run_data(self):
        return {'events': [{'request_id':i+1,'agent_pose_at_observation':[t,0.,0.],
                'observation': {'sim_time':t},'application': {'sim_time':t+.1}}
                for i,t in enumerate([0.,.5,1.,1.5])],
                'worlds': [np.c_[t+.1*np.arange(1,31),np.zeros(30)] for t in [0.,.5,1.,1.5]]}

    def rows(self, states):
        return [{'request_id':i+1,'state':s,'cart_visible_pixel_count':0 if s=='HIDDEN' else 5000 if s=='CLEAR' else 1}
                for i,s in enumerate(states)]

    def test_frozen_cap_and_unchanged_thresholds(self):
        cfg=json.loads(Path('configs/research/obstacle_reveal_acquisition.json').read_text())
        self.assertEqual(len(cfg['station_request_ids'])*len(cfg['left_offsets_m']),12)
        self.assertFalse({19,26}&set(cfg['station_request_ids']))
        self.assertEqual(json.loads(Path(cfg['visibility_config']).read_text())['visibility']['hidden_max_pixels'],0)

    def test_pivot_offset_and_observation_chord(self):
        events=self.run_data()['events']; original=copy.deepcopy(events); bounds=[[-.4,-.2,0],[.6,.2,1.1]]
        cart=cart_at(events,bounds,2,.4)
        np.testing.assert_allclose(cart['center_world_xyz'],[.5,.4,0])
        np.testing.assert_allclose(np.array([.1,0,0,1])@cart['matrix_row_vector'],[.5,.4,0,1])
        self.assertEqual(events,original)
        with self.assertRaises(ValueError):cart_at(events,[[0,0,0],[0,1,1]],2,0.)

    def test_old_remaining_exact_conflict_and_no_extrapolation(self):
        run=self.run_data(); row=temporal_proxy(run,self.cart(),4,.2)
        self.assertAlmostEqual(row['conflict']['sim_time'],2.3)
        self.assertAlmostEqual(row['switch_to_conflict_margin_s'],.7)
        self.assertTrue(row['pass']); self.assertEqual(row['old_request_id'],3)
        cart=self.cart();cart['center_world_xyz'][0]=20.
        self.assertIsNone(temporal_proxy(run,cart,4,.2)['conflict'])

    def test_hidden_first_clear_and_no_later_substitution(self):
        temporal=[temporal_proxy(self.run_data(),self.cart(),i,.2) for i in range(2,5)]
        decision=reveal_decision(self.rows(['HIDDEN']*3+['CLEAR']),temporal,True,4)
        self.assertTrue(decision['qualified']);self.assertEqual(decision['old_request_id'],3)
        rows=self.rows(['HIDDEN','MARGINAL','CLEAR','HIDDEN','HIDDEN','HIDDEN','CLEAR'])
        decision=reveal_decision(rows,temporal,True,7)
        self.assertEqual(decision['first_clear_request_id'],3)
        self.assertFalse(decision['qualified'])
        self.assertIn('IMMEDIATE_OLD_NOT_HIDDEN',decision['failures'])

    def test_incomplete_or_false_hidden_rejected(self):
        with self.assertRaises(ValueError):reveal_decision(self.rows(['HIDDEN']*3),[],True)
        rows=self.rows(['HIDDEN']*3+['CLEAR']);rows[0]['cart_visible_pixel_count']=1
        with self.assertRaises(ValueError):reveal_decision(rows,[],True)
        rows=self.rows(['HIDDEN']*3+['CLEAR']);rows[0]['request_id']=2
        with self.assertRaises(ValueError):reveal_decision(rows,[],True)

    def test_fresh_clear_but_short_is_not_avoidance(self):
        cart=self.cart()
        short=predicted_passage([[0,1],[2,1]],[0,1],cart,.2)
        self.assertGreater(short['minimum_clearance_m'],0);self.assertFalse(short['obstacle_bypass_supported'])
        partial=predicted_passage([[0,1],[3.9,1]],[0,1],cart,.2)
        self.assertFalse(partial['obstacle_bypass_supported'])
        full=predicted_passage([[0,1],[5,1]],[0,1],cart,.2)
        self.assertTrue(full['obstacle_bypass_supported']);self.assertTrue(full['not_executed_navigation_success'])
        collision=predicted_passage([[0,0],[5,0]],[0,1],cart,.2)
        self.assertFalse(collision['obstacle_bypass_supported'])

    def test_no_backward_or_already_beyond_passage(self):
        self.assertFalse(predicted_passage([[5,1],[0,1]],[0,1],self.cart(),.2)['obstacle_bypass_supported'])
        self.assertFalse(predicted_passage([[4,1],[5,1]],[0,1],self.cart(),.2)['obstacle_bypass_supported'])

    def test_connected_sampling_bound(self):
        p=densify([[0,0],[0,1],[1,1]],.05)
        self.assertLessEqual(np.linalg.norm(np.diff(p,axis=0),axis=1).max(),.0500000001)
        np.testing.assert_array_equal(p[[0,-1]],[[0,0],[1,1]])

    def test_source_immutability_and_exclusive_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';source.mkdir();(source/'raw').write_bytes(b'original')
            hashes=evidence_hashes([source]);verify_hashes(hashes)
            with self.assertRaises(ValueError):validate_destination(source/'derived',[source])
            with self.assertRaises(FileExistsError):validate_destination(root,[])

    def test_viewer_unmeasured_tail_is_not_held_visibility(self):
        from research.view_reveal_window_search import archived_visibility
        c={'visibility':self.rows(['HIDDEN','MARGINAL','CLEAR'])}
        self.assertEqual(archived_visibility(c,3)['state'],'CLEAR')
        self.assertIsNone(archived_visibility(c,4));self.assertIsNone(archived_visibility(c,48))

    def test_real_baseline_exact_observation_and_switch_poses(self):
        path=Path('outputs/nova-e16-hospital-lights-baseline-20261010-01');before=evidence_hashes([path])
        run=load_saved(path);self.assertEqual(len(run['events']),48)
        for native,world,event in zip(run['chunks'],run['worlds'],run['events']):
            np.testing.assert_array_equal(world,project_world(native,event['agent_pose_at_observation']))
            for name,key in [('observation','agent_pose_at_observation'),('application','agent_pose_at_application')]:
                row=run['robot'][event[name]['tick']]
                self.assertEqual(event[key],[float(row[k]) for k in ['x','y','yaw']])
        verify_hashes(before)


if __name__=='__main__':unittest.main()
