"""Synthetic fixtures only test arithmetic; real evidence is saved A/B data."""
import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from research.e16_curve_fitting import Curve, methods, project_residual, tangent, chunk_metrics, chord_basis
from research.analyze_e16_chunk_smoothing import CONFIG, ROOT, boundary_metrics, pair_analysis
from research.route_geometry import rotation


class FittingTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads(CONFIG.read_text());self.t=.1*np.arange(1,31)

    def test_exact_nominal_times_and_spline_budget(self):
        xy=np.column_stack([self.t+.02*np.sin(self.t*22),.02*np.cos(self.t*19)])
        for m,s in methods(self.cfg)[1:4]:
            c=Curve(xy,m,s,self.cfg);np.testing.assert_array_equal(c.times,self.t)
            self.assertAlmostEqual(c.info['s_per_coordinate_m2'],30*s*s,places=15)
            self.assertTrue(np.all(np.array(c.info['achieved_ssr_m2'])<=30*s*s*1.00101))

    def test_straight_line_zero_residual(self):
        xy=np.column_stack([1.5*self.t,.2*self.t]);raw=Curve(xy,'RAW',None,self.cfg)
        for m,s in methods(self.cfg)[1:]:
            c=Curve(xy,m,s,self.cfg);r,d=chunk_metrics(c,raw,self.cfg)
            self.assertLess(r['xy_rmse_m'],1e-12);self.assertLess(r['lateral_rmse_m'],1e-12)
            self.assertEqual(r['reliable_tangent_samples'],30)

    def test_known_smooth_turn_derivatives_and_endpoints(self):
        xy=np.column_stack([self.t,.3*self.t**2]);c=Curve(xy,'SPLINE_03cm',.03,self.cfg)
        np.testing.assert_allclose(c.value(self.t),xy,atol=1e-12)
        np.testing.assert_allclose(c.value(self.t,1),np.column_stack([np.ones(30),.6*self.t]),atol=1e-11)
        np.testing.assert_allclose(c.value([.1,3],2),[[0,.6],[0,.6]],atol=1e-10)

    def test_tangent_normal_projection_and_mad_convention(self):
        r=np.array([[3.,4.],[-2.,1.]]);d=np.array([[0.,2.],[0.,2.]])
        f,l,valid,speed=project_residual(r,d,.1)
        np.testing.assert_array_equal(f,[4,1]);np.testing.assert_array_equal(l,[-3,2]);self.assertTrue(valid.all())
        np.testing.assert_allclose(f*f+l*l,np.sum(r*r,axis=1))

    def test_world_rotation_frame_consistency(self):
        xy=np.column_stack([self.t,.2*self.t**2]);c=Curve(xy,'SPLINE_03cm',.03,self.cfg)
        pose=[3,-2,1.1];world=c.world(self.t,pose)
        np.testing.assert_allclose(world,xy@rotation(1.1).T+pose[:2],atol=1e-12)
        residual=np.column_stack([np.sin(self.t),np.cos(self.t)])*.02;der=c.value(self.t,1)
        local=project_residual(residual,der,.1);global_=project_residual(residual@rotation(1.1).T,der@rotation(1.1).T,.1)
        np.testing.assert_allclose(local[:2],global_[:2],atol=1e-15)

    def test_independent_old_fresh_no_shared_fit_or_native_mutation(self):
        old=np.column_stack([self.t,.1*self.t**2]);fresh=old+[.3,.1];before=old.tobytes()
        a=Curve(old,'SPLINE_03cm',.03,self.cfg);original=a.value(self.t)
        b=Curve(fresh,'SPLINE_03cm',.03,self.cfg);fresh[:]=100
        np.testing.assert_array_equal(a.value(self.t),original);self.assertEqual(old.tobytes(),before)
        self.assertFalse(np.shares_memory(a.native,old));self.assertFalse(np.shares_memory(a.native,b.native))

    def test_origin_bridge_is_separate_and_no_reanchoring(self):
        xy=np.column_stack([self.t+.2,.2*self.t]);c=Curve(xy,'SPLINE_03cm',.03,self.cfg)
        self.assertEqual(len(c.times),30);np.testing.assert_allclose(c.value(.05,allow_bridge=True),c.value(.1)/2)
        np.testing.assert_array_equal(c.value(0,allow_bridge=True),[0,0])
        self.assertGreater(c.value(.1)[0],.25)
        e=dict(observation=dict(sim_time=2),application=dict(sim_time=2.2),agent_pose_at_observation=[4,5,.4],agent_pose_at_application=[20,30,1])
        a=boundary_metrics(c,e,0,self.cfg);e['agent_pose_at_application']=[-40,25,-1]
        b=boundary_metrics(c,e,0,self.cfg);np.testing.assert_array_equal(a['boundary_world_xy'],b['boundary_world_xy'])

    def test_no_temporal_extrapolation_including_derivatives(self):
        xy=np.column_stack([self.t,self.t*0])
        for m,s in methods(self.cfg):
            c=Curve(xy,m,s,self.cfg)
            for q,bridge in [(-.1,True),(.05,False),(3.01,True)]:
                for der in [0,1,2]:
                    with self.assertRaises(ValueError):c.value(q,der,bridge)

    def test_invalid_derivative_not_fabricated(self):
        c=Curve(np.zeros((30,2)),'SPLINE_03cm',.03,self.cfg)
        f,l,valid,speed=project_residual(np.ones((30,2)),c.value(self.t,1),.1)
        self.assertFalse(valid.any());self.assertTrue(np.isnan(f).all());self.assertTrue(np.isnan(l).all())
        r,_=chunk_metrics(c,Curve(np.zeros((30,2)),'RAW',None,self.cfg),self.cfg)
        self.assertIsNone(r['forward_rmse_m']);self.assertEqual(r['reliable_tangent_samples'],0)

    def test_raw_right_derivative_and_chord_basis(self):
        xy=np.column_stack([self.t,self.t**2]);c=Curve(xy,'RAW',None,self.cfg)
        np.testing.assert_allclose(c.value(.5,1),(xy[5]-xy[4])/.1)
        graph=json.loads((ROOT/'configs/research/e16_se2_graph_offline.json').read_text())
        basis,diagnostics=chord_basis(c,np.array([.1,.5,3]),graph['tangent'])
        self.assertTrue(all(d['valid'] for d in diagnostics));np.testing.assert_allclose(np.linalg.norm(basis,axis=1),1)

    def test_malformed_native_rejected(self):
        for x in [np.zeros((30,3)),np.full((30,2),np.nan)]:
            with self.assertRaises(ValueError):Curve(x,'SPLINE_03cm',.03,self.cfg)


@unittest.skipUnless((ROOT/'outputs/nova-e16-no-cart-15-repro-20261011-01/summary.json').exists(),'Saved E16 evidence unavailable')
class SavedFittingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from research.compare_e16_se2_graph import load_sources
        from research.reveal_window import evidence_hashes
        cls.cfg=json.loads(CONFIG.read_text());cls.runs,cls.audit=load_sources(cls.cfg)
        cls.hashes=evidence_hashes([r['path'] for r in cls.runs.values()])

    @classmethod
    def tearDownClass(cls):
        from research.reveal_window import evidence_hashes
        assert evidence_hashes([r['path'] for r in cls.runs.values()])==cls.hashes

    def test_exact_96_native_source_hashes_and_observation_projection(self):
        from research.analyze_e16_handoff_audit import exact_event_pose
        count=0
        for r in self.runs.values():
            self.assertEqual(len(r['events']),48)
            for e,c,w in zip(r['events'],r['chunks'],r['worlds']):
                p=r['path']/f"raw/requests/request_{e['request_id']:06d}.npy"
                self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),self.hashes[str(p)])
                np.testing.assert_array_equal(c,np.load(p,allow_pickle=False));self.assertEqual(c.shape,(30,2));self.assertTrue(np.isfinite(c).all())
                exact_event_pose(e,r['robot'],'observation');exact_event_pose(e,r['robot'],'application')
                curve=Curve(c,'RAW',None,self.cfg);np.testing.assert_allclose(curve.world(curve.times,e['agent_pose_at_observation']),w,atol=1e-12)
                count+=1
        self.assertEqual(count,96)

    def test_real_pair_timing_raw_audit_and_no_graph_rerun(self):
        run=self.runs['B'];curves={};rows={};i=15
        for rid in [i,i+1]:
            native=run['chunks'][rid-1];raw=Curve(native,'RAW',None,self.cfg);curves[rid]={'RAW':raw}
            for m,s in methods(self.cfg)[1:]:
                c=Curve(native,m,s,self.cfg);curves[rid][m]=c;rows[(rid,m)]=chunk_metrics(c,raw,self.cfg)[0]
        graph=json.loads((ROOT/self.cfg['previous_graph_dir']/'pairs/B_C15_C16.json').read_text())
        gc=json.loads((ROOT/'configs/research/e16_se2_graph_offline.json').read_text())
        with patch('research.compare_e16_se2_graph.solve_graph',side_effect=AssertionError('Forbidden graph solve')),patch('research.se2_graph.solve_graph',side_effect=AssertionError('Forbidden graph solve')):
            result,d=pair_analysis('B',i,run,curves,rows,self.audit['B'],graph,self.cfg,gc)
        self.assertEqual(len(result),5)
        for m,v in d['methods'].items():
            q=np.array(v['overlaps']['editable']['absolute_times']);self.assertTrue(np.all(q>d['switch_sim_time']+1e-8))
            np.testing.assert_array_equal(v['overlaps']['full']['absolute_times'],d['methods']['RAW']['overlaps']['full']['absolute_times'])
        self.assertEqual(result[0]['full_point_count'],25)


if __name__=='__main__':unittest.main()
