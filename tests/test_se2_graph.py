"""Synthetic fixtures are numerical tests only, never real experiment evidence."""
import copy
import json
from pathlib import Path
import unittest
import numpy as np
from research.se2_graph import compose,inverse,between,exp,log,rigid_align,relative_error,Graph,solve_graph,reference_at_switch,tangent_headings,correspondence
from research.e16_se2_metrics import evaluate
from research.route_geometry import wrap

ROOT=Path(__file__).resolve().parents[1]

def fixture():
    cfg=json.loads((ROOT/'configs/research/e16_se2_graph_offline.json').read_text())
    t=np.arange(0,1.1,.1);f=np.column_stack([t,np.zeros_like(t),np.zeros_like(t)])
    b=np.array([.04,.08,.15]);corr=correspondence(t,f,f[1:,:2]+[0,.2],0,cfg)
    return cfg,t,f,b,corr


class SE2Tests(unittest.TestCase):
    def test_composition_inverse_matrix_oracle(self):
        a=np.array([2.,-3.,1.2]);b=np.array([.5,4.,-.7])
        def matrix(p):
            c,s=np.cos(p[2]),np.sin(p[2]);return np.array([[c,-s,p[0]],[s,c,p[1]],[0,0,1]])
        np.testing.assert_allclose(matrix(compose(a,b)),matrix(a)@matrix(b),atol=1e-14)
        np.testing.assert_allclose(compose(a,inverse(a)),0,atol=1e-14)
        self.assertFalse(np.allclose(compose(a,b),compose(b,a)))

    def test_exp_log_wrap(self):
        a=np.array([[1.,2.,0],[.3,-.8,1e-10],[.1,.8,np.pi-1e-8],[.1,.8,-np.pi+1e-8]])
        np.testing.assert_allclose(log(exp(a)),a,atol=1e-9)
        np.testing.assert_allclose(exp(log(a)),a,atol=1e-9)
        self.assertAlmostEqual(wrap((-np.pi+.01)-(np.pi-.01)),.02)
        self.assertAlmostEqual(log(between([0,0,np.pi-.01],[0,0,-np.pi+.01]))[2],.02)

    def test_relative_error_and_rigid_preservation(self):
        cfg,t,f,b,corr=fixture();x=rigid_align(f,b)
        np.testing.assert_allclose(x[0],b,atol=1e-14)
        np.testing.assert_allclose(relative_error(f,x),0,atol=1e-14)
        changed=x.copy();changed[4,1]+=.1;self.assertGreater(np.linalg.norm(relative_error(f,changed)),.1)

    def test_fixed_boundary_identity(self):
        cfg,t,f,_,corr=fixture();b=f[0].copy();corr=correspondence(t,f,f[1:,:2],0,cfg)
        answer=solve_graph(f,b,t,corr,0.,cfg)
        self.assertTrue(answer['success']);np.testing.assert_array_equal(answer['poses'][0],b)
        np.testing.assert_allclose(answer['poses'],f,atol=1e-10);self.assertLess(answer['final_objective'],1e-15)
        g=Graph(f,b,t,corr,0,cfg);self.assertEqual(g.sparsity().shape[1],3*(len(f)-1))

    def test_relative_only_graph_recovers_rigid_from_nonrigid_initial(self):
        cfg,t,f,b,corr=fixture();cfg['weights']={k:float(k=='relative') for k in cfg['weights']}
        initial=f.copy();initial[:,1]+=.02*np.sin(t*5)
        answer=solve_graph(f,b,t,corr,None,cfg,initial=initial)
        self.assertTrue(answer['success'],answer['message'])
        np.testing.assert_allclose(answer['poses'],rigid_align(f,b),atol=2e-6)
        self.assertLess(answer['final_objective'],1e-9)

    def test_competing_factors_make_nonrigid_and_ablation_differs(self):
        cfg,t,f,b,corr=fixture();yes=solve_graph(f,b,t,corr,.15,cfg);no=solve_graph(f,b,t,corr,.15,cfg,False)
        self.assertTrue(yes['success']);self.assertTrue(no['success'])
        self.assertLess(yes['final_objective'],yes['initial_objective'])
        self.assertGreater(np.linalg.norm(relative_error(f,yes['poses'])),1e-4)
        self.assertGreater(np.linalg.norm(yes['poses']-no['poses']),1e-5)
        self.assertFalse(np.array_equal(yes['poses'][1:],rigid_align(f,b)[1:]))

    def test_global_frame_invariance(self):
        cfg,t,f,b,corr=fixture();a=solve_graph(f,b,t,corr,.15,cfg);g=np.array([5,-3,1.8])
        c2=copy.deepcopy(corr);c2['old_xy']=compose(g,np.column_stack([corr['old_xy'],np.zeros(len(corr['indices']))]))[:,:2]
        z=solve_graph(compose(g,f),compose(g,b),t,c2,wrap(.15+g[2]),cfg)
        self.assertTrue(z['success']);np.testing.assert_allclose(z['poses'],compose(g,a['poses']),atol=3e-5)
        ma=evaluate(f,a['poses'],t,b,.15,cfg);mb=evaluate(compose(g,f),z['poses'],t,compose(g,b),wrap(.15+g[2]),cfg)
        for key in ['far_fresh_xy_rmse_m','relative_translation_rmse_m','first_segment_robot_heading_deg']:
            if ma[key] is not None:self.assertAlmostEqual(ma[key],mb[key],places=3)

    def test_nominal_correspondence_and_no_extrapolation(self):
        cfg,t,f,b,corr=fixture();old=np.column_stack([np.arange(1,31)*.1,np.zeros(30)])
        times=.6+np.arange(6)*.1;ref=np.column_stack([times,np.zeros((6,2))])
        c=correspondence(times,ref,old,0,cfg)
        self.assertEqual(c['indices'].tolist(),[1,2,3,4]);np.testing.assert_allclose(c['old_xy'][:,0],[.7,.8,.9,1.])
        empty=correspondence(times+3,ref,old,0,cfg);self.assertEqual(len(empty['indices']),0)
        with self.assertRaises(ValueError):reference_at_switch(old,[0,0],0,-.1,cfg)
        with self.assertRaises(ValueError):reference_at_switch(old,[0,0],0,3.1,cfg)

    def test_tangent_short_backward_endpoint_and_immutability(self):
        cfg,t,f,b,corr=fixture();native=np.column_stack([np.arange(1,31)*.1,np.zeros(30)]).astype(np.float32);before=native.tobytes()
        times,ref,info=reference_at_switch(native,[0,0],0,.15,cfg)
        self.assertEqual(native.tobytes(),before);self.assertEqual(native.shape,(30,2));self.assertEqual(times[0],.15)
        np.testing.assert_allclose(ref[:,2],0);self.assertTrue(info[-1]['valid'])
        zig=f[:,:2].copy();zig[2,0]=.09
        yaw,diag=tangent_headings(t,zig,cfg['tangent']);self.assertTrue(np.isfinite(yaw).all());np.testing.assert_allclose(yaw,0)
        yaw,diag=tangent_headings(t,np.zeros((len(t),2)),cfg['tangent']);self.assertTrue(np.isnan(yaw).all());self.assertFalse(diag[0]['valid'])

    def test_failure_status_not_silently_success(self):
        cfg,t,f,b,corr=fixture();cfg['solver']['max_nfev']=1
        a=solve_graph(f,b,t,corr,.15,cfg);self.assertFalse(a['success']);self.assertEqual(a['status'],0)
        bad=f.copy();bad[2,0]=np.nan;b=solve_graph(bad,b,t,corr,.15,cfg)
        self.assertFalse(b['success']);self.assertTrue(b['numerical_failure']);self.assertIsNone(b['poses'])


if __name__=='__main__':unittest.main()
