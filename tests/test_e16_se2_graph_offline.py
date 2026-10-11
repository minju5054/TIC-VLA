"""Saved-data contracts; no model calls, render, physics or writes to source runs."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from research.compare_e16_se2_graph import CONFIG,load_sources,serial
from research.e16_se2_metrics import build_pair,evaluate
from research.se2_graph import rigid_align,compose
from research.route_geometry import boundary_seam,interpolate
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination

ROOT=Path(__file__).resolve().parents[1]
RUNS=[ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01',ROOT/'outputs/nova-e16-no-cart-15-repro-20261011-01']


@unittest.skipUnless(all((p/'summary.json').exists() for p in RUNS),'Saved local research evidence unavailable')
class SavedGraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads(CONFIG.read_text());cls.hashes=evidence_hashes(RUNS);cls.runs,cls.audit=load_sources(cls.cfg)

    @classmethod
    def tearDownClass(cls):verify_hashes(cls.hashes)

    def test_96_real_chunks_and_94_adjacent_sources(self):
        self.assertEqual(sum(len(r['events']) for r in self.runs.values()),96)
        self.assertEqual(sum(len(d['rows']) for d in self.audit.values()),94)
        self.assertEqual(self.runs['A']['cfg'],self.runs['B']['cfg'])

    def test_exact_boundary_interpolation_own_origin_and_correspondence(self):
        for name,i in self.cfg['selected_pairs']:
            r=self.runs[name];p=build_pair(r,i,self.cfg);e=r['events'][i];world=r['worlds'][i]
            np.testing.assert_array_equal(p['boundary'],e['agent_pose_at_application'])
            np.testing.assert_array_equal(p['fresh_observation_pose'],e['agent_pose_at_observation'])
            seam=boundary_seam(world,e['observation'],e['application'],r['times'],r['xy'])
            np.testing.assert_allclose(p['reference'][0,:2],seam['fresh_expected_xy'],atol=1e-12)
            if p['eligibility']=='ELIGIBLE':
                c=p['correspondence'];self.assertTrue(np.all(c['absolute_times']>p['switch_sim_time']))
                self.assertTrue(np.all(c['absolute_times']<p['switch_sim_time']+.5))
                ot=r['events'][i-1]['observation']['sim_time']+.1*np.arange(1,31)
                np.testing.assert_allclose(c['old_xy'],interpolate(ot,r['worlds'][i-1],c['absolute_times']),atol=1e-12)

    def test_future_measured_poses_cannot_change_problem(self):
        r=self.runs['A'];p=build_pair(r,13,self.cfg);altered=copy.deepcopy(r)
        after=altered['times']>p['switch_sim_time'];altered['xy'][after]+=10000
        for row in altered['robot']:
            if float(row['sim_time'])>p['switch_sim_time']:row['x']='10000';row['y']='-10000'
        q=build_pair(altered,13,self.cfg)
        self.assertEqual(serial(p),serial(q))

    def test_native_tensor_and_previous_raw_immutability(self):
        r=self.runs['B'];old=r['chunks'][15].tobytes();p=build_pair(r,15,self.cfg)
        self.assertIsNone(p['native_yaw']);self.assertIsNone(p['native_timestamps'])
        self.assertEqual(r['chunks'][15].shape,(30,2));self.assertEqual(r['chunks'][15].tobytes(),old)
        self.assertNotIn('native_yaw',dict(native_array=r['chunks'][15]))
        verify_hashes(self.hashes)
        with self.assertRaises((ValueError,FileExistsError)):validate_destination(RUNS[0],RUNS)

    def test_ineligible_headings_explicit_no_fake_yaw(self):
        r=copy.deepcopy(self.runs['A']);r['worlds'][13][:]=r['events'][13]['agent_pose_at_observation'][:2]
        p=build_pair(r,13,self.cfg);self.assertEqual(p['eligibility'],'INELIGIBLE');self.assertEqual(p['valid_node_count'],0)
        dumped=json.dumps(serial(p),allow_nan=False);self.assertNotIn('NaN',dumped);self.assertIn('Unreliable reference tangent',dumped)

    def test_raw_rigid_metrics_independent_xy_oracle(self):
        p=build_pair(self.runs['B'],15,self.cfg);f=p['reference'];b=p['boundary'];x=rigid_align(f,b)
        raw=evaluate(f,f,p['times'],b,p['executed_direction'],self.cfg);m=evaluate(f,x,p['times'],b,p['executed_direction'],self.cfg)
        self.assertAlmostEqual(raw['boundary_position_m'],float(np.linalg.norm(f[0,:2]-b[:2])))
        self.assertLess(m['boundary_position_m'],1e-12);self.assertLess(m['relative_translation_rmse_m'],1e-12)
        self.assertAlmostEqual(m['endpoint_displacement_m'],float(np.linalg.norm(x[-1,:2]-f[-1,:2])))
        self.assertEqual(raw['fresh_xy_rmse_m'],0.)
        near=np.linalg.norm(np.diff(x[:,:2],axis=0),axis=1)-np.linalg.norm(np.diff(f[:,:2],axis=0),axis=1)
        self.assertLess(np.max(np.abs(near)),1e-12)


if __name__=='__main__':unittest.main()
