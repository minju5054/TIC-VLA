"""Synthetic contracts, never substitutes for actual Hospital rendered evidence."""
import ast
import copy
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from research.reveal_window import (coarse_positions, state, first_clear, conflict_time,
    temporal_metrics, classify, ranked, freeze_selected, evidence_hashes, verify_hashes, CandidateNavigation, validate_destination)
from research.nova_dynamic_geometry import remaining_curve

CONFIG=json.loads(Path('configs/research/reveal_window_search.json').read_text())


class RevealWindowTests(unittest.TestCase):
    def run_data(self):
        times=np.arange(4)*.5
        return {'events':[{'request_id':i+1,'observation':{'sim_time':float(t),'position':[float(t),0.,0.]},
                           'application':{'sim_time':float(t+.1)}} for i,t in enumerate(times)],
                'worlds':[np.column_stack([t+.1*np.arange(1,31),np.zeros(30)]) for t in times]}

    def rows(self, states=('HIDDEN','HIDDEN','HIDDEN','CLEAR')):
        return [{'request_id':i+1,'state':s,'human_visible_fraction':.003 if s=='CLEAR' else 0.}
                for i,s in enumerate(states)]

    def candidate(self):
        return {'id':1,'position':[3.,0.,0.],'geometry':{'floor_ok':True,'overlap_paths':[]},
                'bypass':{'pass':True,'clearance_m':.5}}

    def test_preregistered_constraints(self):
        self.assertEqual(CONFIG['model_calls'],0)
        self.assertFalse(CONFIG['navigation_physics_reexecuted'])
        self.assertEqual(CONFIG['visibility']['clearly_visible_min_fraction'],.002)
        self.assertEqual(CONFIG['human_yaw_rad'],0.)
        self.assertEqual(CONFIG['ranking'][0],'switch_to_conflict_margin_s ascending, strictly positive')

    def test_generation_and_order_deterministic(self):
        path=np.array([[0,0],[0,1],[1,1]])
        a=coarse_positions(path,.5,1.)
        self.assertEqual(a,coarse_positions(path[::-1],.5,1.))
        self.assertEqual(a,sorted(a));self.assertEqual(len(a),len(set(map(tuple,a))))

    def test_visibility_boundaries(self):
        r=CONFIG['visibility'];s={'human_visible_pixel_count':0,'human_visible_fraction':0,'human_bbox_if_visible':None}
        self.assertEqual(state(s,r),'HIDDEN')
        s.update(human_visible_pixel_count=1,human_visible_fraction=.002,human_bbox_if_visible=[0,0,20,80])
        self.assertEqual(state(s,r),'CLEAR')
        s['human_bbox_if_visible']=[0,0,19,80];self.assertEqual(state(s,r),'MARGINAL')
        s['human_bbox_if_visible']=[0,0,20,79];self.assertEqual(state(s,r),'MARGINAL')

    def test_first_clear_no_later_substitution(self):
        rows=self.rows(['MARGINAL','CLEAR','HIDDEN','HIDDEN','HIDDEN','CLEAR'])
        self.assertEqual(first_clear(rows)['fresh_request_id'],2)
        self.assertFalse(first_clear(rows)['three_prior_hidden'])

    def test_three_hidden_history(self):
        self.assertTrue(first_clear(self.rows())['three_prior_hidden'])
        self.assertFalse(first_clear(self.rows(['HIDDEN','HIDDEN','CLEAR']))['three_prior_hidden'])
        self.assertFalse(first_clear(self.rows(['MARGINAL','HIDDEN','HIDDEN','CLEAR']))['three_prior_hidden'])
        self.assertIsNone(first_clear(self.rows(['HIDDEN']*4)))

    def test_history_gaps_fail(self):
        r=self.rows();r[0]['request_id']=2
        with self.assertRaises(ValueError):first_clear(r)

    def test_remaining_interpolation_no_mutation(self):
        p=np.column_stack([np.arange(1,31)*.1,np.zeros(30)]);before=p.copy()
        t,q=remaining_curve(p,1.,1.55)
        self.assertAlmostEqual(t[0],1.55);self.assertAlmostEqual(q[0,0],.55)
        self.assertAlmostEqual(t[-1],4.);np.testing.assert_array_equal(before,p)
        self.assertEqual(remaining_curve(p,1,5),(None,None))

    def test_exact_entry(self):
        hit=conflict_time([1,3],[[0,0],[4,0]],[3,0],1)
        self.assertAlmostEqual(hit['time'],2);np.testing.assert_allclose(hit['point'],[2,0])

    def test_tangent_and_degenerate_segment(self):
        self.assertAlmostEqual(conflict_time([0,1],[[0,1],[2,1]],[1,0],1)['time'],.5)
        self.assertIsNone(conflict_time([0,1],[[0,2],[0,2]],[1,0],1))
        self.assertEqual(conflict_time([0,1],[[1,0],[1,0]],[0,0],1)['time'],0)

    def test_no_extrapolation(self):
        self.assertIsNone(conflict_time([0,1],[[0,0],[1,0]],[3,0],1))
        self.assertEqual(conflict_time([0,1],[[0,0],[1,0]],[2,0],1)['time'],1)

    def test_temporal_lead_margin_current_clearance(self):
        r=temporal_metrics(self.run_data(),4,[3,0,0],CONFIG)
        radius=CONFIG['human_radius_m']+CONFIG['robot_radius_m']
        self.assertAlmostEqual(r['conflict']['time'],3-radius)
        self.assertAlmostEqual(r['reveal_lead_s'],1.5-radius)
        self.assertAlmostEqual(r['switch_to_conflict_margin_s'],1.4-radius)
        self.assertAlmostEqual(r['robot_current_clearance_at_reveal_m'],1.5-radius)

    def test_strict_pass(self):
        c=classify(self.candidate(),self.rows(),self.run_data(),CONFIG)
        self.assertTrue(c['strict_qualified']);self.assertEqual(c['failures'],[])

    def test_geometry_and_bypass_filters(self):
        c=self.candidate();c['geometry']['floor_ok']=False
        self.assertIn('E',classify(c,self.rows(),self.run_data(),CONFIG)['failures'])
        c=self.candidate();c['geometry']['overlap_paths']=['wall']
        self.assertIn('E',classify(c,self.rows(),self.run_data(),CONFIG)['failures'])
        c=self.candidate();c['bypass']['pass']=False
        self.assertIn('D',classify(c,self.rows(),self.run_data(),CONFIG)['failures'])

    def test_near_miss_categories(self):
        c=self.candidate();run=self.run_data()
        self.assertIn('B',classify(c,self.rows(['HIDDEN','HIDDEN','MARGINAL','CLEAR']),run,CONFIG)['failures'])
        self.assertIn('F',classify(c,self.rows(['HIDDEN']*4),run,CONFIG)['failures'])
        c['position']=[8,0,0];self.assertIn('A',classify(c,self.rows(),run,CONFIG)['failures'])
        c['position']=[1.5,0,0];self.assertIn('C',classify(c,self.rows(),run,CONFIG)['failures'])
        c['position']=[2.38,0,0];self.assertIn('G',classify(c,self.rows(),run,CONFIG)['failures'])

    def test_unmeasured_not_never_clear(self):
        c=classify(self.candidate(),None,self.run_data(),CONFIG)
        self.assertEqual(c['failures'],['A_prefilter']);self.assertFalse(c['rendered'])

    def test_ranking_and_ties(self):
        c=classify(self.candidate(),self.rows(),self.run_data(),CONFIG)
        b=copy.deepcopy(c);b['id']=2;b['position']=[2.9,0,0]
        self.assertEqual(ranked([c,b])[0]['id'],2)
        b['bypass']['clearance_m']=.4;self.assertEqual(ranked([c,b])[0]['id'],1)
        b['first_clear']['human_visible_fraction']=.004;self.assertEqual(ranked([c,b])[0]['id'],2)
        b['switch_to_conflict_margin_s']+=.001;self.assertEqual(ranked([c,b])[0]['id'],1)

    def test_selected_exclusive_freeze(self):
        c=classify(self.candidate(),self.rows(),self.run_data(),CONFIG)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'selected.json';self.assertEqual(freeze_selected(p,[c])['selection_rank'],1)
            with self.assertRaises(FileExistsError):freeze_selected(p,[])
            q=Path(d)/'none.json';self.assertIsNone(freeze_selected(q,[]));self.assertEqual(json.loads(q.read_text()),None)

    def test_navigation_and_event_jumps(self):
        c=classify(self.candidate(),self.rows(),self.run_data(),CONFIG)
        b=copy.deepcopy(c);b.update(id=2,strict_qualified=False)
        nav=CandidateNavigation([b,c]);self.assertEqual(nav.top()['id'],1)
        self.assertEqual(nav.jump('old'),3);self.assertEqual(nav.jump('fresh'),4)
        self.assertEqual(nav.move(1)['id'],2);self.assertEqual(nav.move(-1)['id'],1)
        self.assertEqual(nav.filter('near')['id'],2);self.assertEqual(nav.filter('strict')['id'],1)

    def test_source_immutability(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'source';p.write_text('native');hashes=evidence_hashes([p])
            classify(self.candidate(),self.rows(),self.run_data(),CONFIG);verify_hashes(hashes)
            p.write_text('modified')
            with self.assertRaises(ValueError):verify_hashes(hashes)

    def test_destination_cannot_modify_source_or_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'source';source.mkdir()
            with self.assertRaises(ValueError):validate_destination(source/'derived',[source])
            with self.assertRaises(ValueError):validate_destination(source,[source])
            with self.assertRaises(FileExistsError):validate_destination(source,[])
            self.assertEqual(validate_destination(Path(d)/'fresh',[source]),Path(d)/'fresh')

    def test_no_model_or_navigation_calls(self):
        for name in ['reveal_window.py','search_reveal_window.py','view_reveal_window_search.py']:
            p=Path('research')/name
            if not p.exists():continue
            tree=ast.parse(p.read_text())
            for n in ast.walk(tree):
                if isinstance(n,ast.ImportFrom):
                    self.assertNotIn(n.module,['research.ipc','research.continuous','research.sim'])
                if isinstance(n,ast.Call):
                    name=ast.unparse(n.func)
                    self.assertNotIn(name,['timeline.play','sim.world.step','world.step','model.predict','Client'])


if __name__=='__main__':unittest.main()
