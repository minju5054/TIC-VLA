"""Mathematical fixture tests only; fixtures are not E16 experimental evidence."""
import copy
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from research.audit_e16_wheelchair_handoffs import (
    to_world, to_local, rotation, interpolate, remaining, tangent, time_comparison,
    crossing, proxy_clearance, committed_history, is_primary, snapshot, verify_snapshot,
    classify, write, curvature_diagnostics, prepare_only)

P = dict(min_segment_m=.02, max_adjacent_heading_change_deg=45, safe_proxy_margin_m=.02)


class CoordinateTests(unittest.TestCase):
    def test_recorded_float32_quaternion_norm_precision(self):
        from research.control import planar_pose
        q=np.array([.5782780647277832,.0006370693445205688,-.0004519884241744876,.8158395290374756],dtype=np.float32)
        original=planar_pose([0,0,0],q)
        decoded=planar_pose([0,0,0],np.array(q.tolist(),dtype=np.float32))
        self.assertEqual(original,decoded)
        self.assertLess(abs(original[2]-planar_pose([0,0,0],q.tolist())[2]),8*np.finfo(np.float32).eps)

    def test_observation_to_world_axes(self):
        np.testing.assert_allclose(to_world([[1, 0], [0, 1]], [3, 4, math.pi/2]), [[3, 5], [2, 4]])

    def test_roundtrip_rotated_frame(self):
        local=np.array([[.2,-.4],[4,2],[-1,.5]]);pose=[1.4,-3.2,-2.1]
        np.testing.assert_allclose(to_local(to_world(local,pose),pose),local,atol=1e-14)

    def test_rigid_transform_equivariance(self):
        local=np.array([[1,2],[2,-1]]);body=[2,3,.3];offset=[-4,7,.7]
        compound=[*to_world([body[:2]],offset)[0],body[2]+offset[2]]
        np.testing.assert_allclose(to_world(to_world(local,body),offset),to_world(local,compound),atol=1e-14)

    def test_switch_not_used_to_anchor_fresh(self):
        world=to_world([[.1,0],[.2,0]],[2,3,0]);t=np.array([1.1,1.2])
        _,_,q=remaining(t,world,1.15);B=np.array([2.25,3])
        self.assertAlmostEqual(np.linalg.norm(B-q),.1)
        self.assertFalse(np.allclose(q,to_world([[.15,0]],[*B,0])[0]))


class TimingTests(unittest.TestCase):
    def test_interpolation_no_extrapolation(self):
        t=[.1,.2];p=[[0,0],[1,2]]
        np.testing.assert_allclose(interpolate(t,p,.15),[.5,1])
        for q in (0,.21):
            with self.assertRaises(ValueError):interpolate(t,p,q)

    def test_no_synthetic_time_zero_knot(self):
        t=np.array([.1,.2]);p=np.array([[.3,0],[.6,0]])
        ts,ps,q=remaining(t,p,.05)
        self.assertIsNone(q);np.testing.assert_array_equal(ts,t);np.testing.assert_array_equal(ps,p)

    def test_only_common_future_support(self):
        r=time_comparison(np.array([.1,.2,.3]),np.array([[.1,0],[.2,0],[.3,0]]),
                          np.array([.2,.3,.4]),np.array([[.2,0],[.3,0],[.4,0]]),.25,0)
        self.assertAlmostEqual(r['position_rmse_m'],0);self.assertEqual(r['start_sim_s'],.25);self.assertEqual(r['end_sim_s'],.3)

    def test_time_integral_not_knot_density(self):
        r=time_comparison(np.array([0.,1.]),np.array([[0,0],[0,0]]),
                          np.array([0.,.1,1.]),np.array([[0,0],[.1,0],[1,0]]),0,0)
        self.assertAlmostEqual(r['position_rmse_m'],math.sqrt(1/3))

    def test_no_common_horizon(self):
        r=time_comparison(np.array([0.,1.]),np.zeros((2,2)),np.array([2.,3.]),np.zeros((2,2)),2,0)
        self.assertFalse(r['valid'])

    def test_goal_boundary_excluded(self):
        self.assertTrue(is_primary(32.57,32.58));self.assertFalse(is_primary(32.58,32.58));self.assertFalse(is_primary(40,32.58))


class GeometryTests(unittest.TestCase):
    def test_short_segment_not_heading_evidence(self):
        r=tangent([[0,0],[.001,0],[1,0]],0,P)
        self.assertFalse(r['valid']);self.assertEqual(r['reason'],'SHORT_FIRST_SEGMENT')

    def test_backwards_and_reversal(self):
        self.assertFalse(tangent([[0,0],[-1,0],[-2,0]],0,P)['valid'])
        self.assertFalse(tangent([[0,0],[1,0],[0,0]],0,P)['valid'])

    def test_valid_tangent(self):
        r=tangent([[0,0],[1,.1],[2,.3]],0,P)
        self.assertTrue(r['valid']);self.assertAlmostEqual(r['yaw_rad'],math.atan2(.1,1))

    def test_predicted_sides_and_no_crossing(self):
        for y,side in ((2,'LEFT'),(-2,'RIGHT'),(.1,'LEFT'),(0,'CENTRE')):
            self.assertEqual(crossing([[-1,y],[1,y]],[0,0],0,.5,.5)['side'],side)
        self.assertEqual(crossing([[-2,2],[-1,2]],[0,0],0,.5,.5)['status'],'NO_PREDICTED_CROSSING')

    def test_side_rotation_invariance(self):
        p=to_world([[-1,2],[1,2]],[3,4,1.1])
        self.assertEqual(crossing(p,[3,4],1.1,.5,.5)['side'],'LEFT')

    def test_passage_side_separate_from_proxy_overlap(self):
        r=crossing([[-1,-.8],[1,-.8]],[0,0],0,.41,.61)
        self.assertEqual(r['side'],'RIGHT');self.assertTrue(r['proxy_corridor_overlap'])

    def test_conservative_proxy_is_not_contact(self):
        box=dict(name='wheelchair',center=np.zeros(2),half=np.ones(2),yaw=0)
        r=proxy_clearance([[1.2,-2],[1.2,2]],[box],.5,.01)
        self.assertTrue(r['proxy_overlap_observed']);self.assertFalse(r['actual_physx_collision_claim'])
        self.assertLessEqual(r['certified_lower_m'],r['sampled_min_m'])

    def test_segment_crosses_box_between_knots(self):
        box=dict(name='wall',center=np.zeros(2),half=np.array([.1,1]),yaw=0)
        r=proxy_clearance([[-2,0],[2,0]],[box],.2,.01)
        self.assertTrue(r['proxy_overlap_observed'])

    def test_executed_old_comes_from_measured_rows(self):
        rows=[dict(x=x,y=2*x) for x in range(4)]
        a=committed_history(rows,1,3);np.testing.assert_array_equal(a,[[1,2],[2,4],[3,6]])
        self.assertEqual(len(a),3)  # No OLD prediction/timed waypoint input.

    def test_curvature_cap_is_nominal_only(self):
        r=curvature_diagnostics(np.array([0,.1,.2]),np.array([[0,0],[1,0],[2,0]]),.02,.8,1.2)
        self.assertEqual(r['nominal_speed_cap_excess_count'],2);self.assertEqual(r['max_valid_curvature_per_m'],0)


class DecisionAndPreservationTests(unittest.TestCase):
    def base(self):
        clear=dict(valid=True,certified_lower_m=.1,proxy_overlap_observed=False)
        return dict(primary=True,fresh_proxy=copy.deepcopy(clear),boundary_proxy=copy.deepcopy(clear),connector_proxy=copy.deepcopy(clear),
                    boundary_position_m=.1,tangent=dict(valid=True),aligned=dict(valid=True),pending_path_m=.1,
                    obstacles=dict(C10=dict(in_window=True,meaningful_revision=False,side_change=False)))

    def test_large_mismatch_alone_never_candidate(self):
        m=self.base();m['boundary_position_m']=5
        self.assertEqual(classify(m,P)[0],'MISMATCH_WITHOUT_DEMONSTRATED_RISK')

    def test_upstream_fresh_risk_has_priority(self):
        m=self.base();m['fresh_proxy']['certified_lower_m']=-.1;m['obstacles']['C10']['meaningful_revision']=True
        self.assertEqual(classify(m,P)[0],'UNSAFE_OR_UNQUALIFIED_FRESH')

    def test_candidate_requires_connector_only_risk(self):
        m=self.base();m['obstacles']['C10']['meaningful_revision']=True;m['connector_proxy']['proxy_overlap_observed']=True
        self.assertEqual(classify(m,P)[0],'POTENTIAL_RECONCILIATION_HARD_CASE')
        m['boundary_proxy']['certified_lower_m']=-.1
        self.assertNotEqual(classify(m,P)[0],'POTENTIAL_RECONCILIATION_HARD_CASE')

    def test_safe_side_change_is_not_hard_case(self):
        m=self.base();m['obstacles']['C10'].update(meaningful_revision=True,side_change=True)
        self.assertEqual(classify(m,P)[0],'INTENT_CHANGE_WITHOUT_HANDOFF_PROBLEM')

    def test_missing_tangent_is_unqualified(self):
        m=self.base();m['tangent']['valid']=False
        self.assertEqual(classify(m,P)[0],'INSUFFICIENT_EVIDENCE')

    def test_source_modification_detected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').write_bytes(b'original');before=snapshot(p)
            self.assertEqual(verify_snapshot(p,before),1)
            (p/'raw').write_bytes(b'changed')
            with self.assertRaises(ValueError):verify_snapshot(p,before)

    def test_exclusive_output_no_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'result.json';write(p,{'a':1})
            with self.assertRaises(FileExistsError):write(p,{'a':2})
            self.assertIn('1',p.read_text())

    def test_prepare_rejects_existing_directory(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source=p/'source';out=p/'out';source.mkdir();out.mkdir()
            with self.assertRaises(FileExistsError):prepare_only(source,out,p)

    def test_prepare_rejects_output_inside_raw_source(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            with self.assertRaises(ValueError):prepare_only(p,p/'derived',p)


if __name__=='__main__':
    unittest.main()
