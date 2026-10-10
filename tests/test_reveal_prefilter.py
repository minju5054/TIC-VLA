"""Cheap gates are scheduling heuristics; semantic qualification stays separate."""
import ast
import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
from research.reveal_prefilter import human_samples, possible_projection, ray_transition, temporal_pairs, prefilter_result
from research.reveal_window import classify
from tests import test_reveal_window as fixture_module
CONFIG=fixture_module.CONFIG

RULES=json.loads(Path('configs/research/reveal_window_prefilter.json').read_text())


class RevealPrefilterTests(unittest.TestCase):
    def data(self):
        fixture=fixture_module.RevealWindowTests();return fixture.run_data(),fixture.candidate()

    def rays(self, fresh=True):
        return [{'request_id':i+1,'exposed':[False]*3 if i<3 or not fresh else [True,True,True]} for i in range(4)]

    def test_samples_deterministic_and_fixed_yaw(self):
        a=human_samples([1,2,0],0,RULES);self.assertEqual(a,human_samples([1,2,0],0,RULES))
        self.assertEqual(len(a),17);self.assertEqual({s['group'] for s in a},{'head','torso','leg','arm'})
        b=human_samples([1,2,0],np.pi/2,RULES)
        np.testing.assert_allclose(np.array(b[1]['world'])[:2]-[1,2],[0,-.12],atol=1e-12)

    def test_projection_is_only_broad_envelope(self):
        m=np.eye(4)
        self.assertTrue(possible_projection([1.5,0,-1],m,1,1,2))
        self.assertFalse(possible_projection([3,0,-1],m,1,1,2))
        self.assertFalse(possible_projection([0,0,1],m,1,1,2))

    def test_ray_transition_and_minimum_history(self):
        r=self.rays();groups=['head','torso','leg']
        self.assertTrue(ray_transition(r,4,groups)['pass'])
        self.assertFalse(ray_transition(r,3,groups)['pass'])
        self.assertFalse(ray_transition(self.rays(False),4,groups)['pass'])

    def test_partial_old_exposure_retained_not_hidden_label(self):
        r=self.rays();r[0]['exposed']=[True,False,False];r[1]['exposed']=[True,False,False];r[2]['exposed']=[True,False,False]
        outcome=ray_transition(r,4,['head','torso','leg'])
        self.assertTrue(outcome['pass']);self.assertNotIn('HIDDEN',json.dumps(outcome))

    def test_all_prior_groups_exposed_rejected(self):
        r=self.rays();r[2]['exposed']=[True,True,True]
        self.assertFalse(ray_transition(r,4,['head','torso','leg'])['pass'])

    def test_no_new_exposure_rejected(self):
        r=self.rays()
        for row in r:row['exposed']=[True,False,False]
        self.assertFalse(ray_transition(r,4,['head','torso','leg'])['pass'])

    def test_temporal_same_pair_conjunction(self):
        run,c=self.data();pairs=temporal_pairs(run,c,CONFIG)
        self.assertTrue(pairs[-1]['temporal_pass'])
        # Passing different gates at different pairs cannot qualify.
        for i,e in enumerate(run['events']):
            if i<3:e['application']['sim_time']=100.
            else:e['observation']['position']=[3.,0.,0.]
        self.assertFalse(any(p['temporal_pass'] for p in temporal_pairs(run,c,CONFIG)))

    def test_filters_all_required(self):
        run,c=self.data();pairs=temporal_pairs(run,c,CONFIG);groups=['head','torso','leg']
        result=prefilter_result(c,copy.deepcopy(pairs),self.rays(),groups,RULES)
        self.assertTrue(result['pass']);self.assertEqual(result['surviving_fresh_request_ids'],[4])
        c['bypass']['pass']=False
        self.assertFalse(prefilter_result(c,copy.deepcopy(pairs),self.rays(),groups,RULES)['pass'])
        c['bypass']['pass']=True
        self.assertFalse(prefilter_result(c,copy.deepcopy(pairs),self.rays(False),groups,RULES)['pass'])

    def test_no_conflict_reason(self):
        run,c=self.data();c['position']=[30.,0,0]
        result=prefilter_result(c,temporal_pairs(run,c,CONFIG),None,[],RULES)
        self.assertIn('no_remaining_conflict_at_any_pair',result['rejection_reasons']);self.assertFalse(result['pass'])

    def test_prefilter_pair_never_substitutes_first_clear(self):
        run,c=self.data();fixture=fixture_module.RevealWindowTests()
        c['prefilter']={'temporal_pass_pair_count':1,'surviving_fresh_request_ids':[4]}
        result=classify(c,fixture.rows(['HIDDEN','CLEAR','HIDDEN','CLEAR']),run,CONFIG)
        self.assertEqual(result['fresh_request_id'],2);self.assertFalse(result['strict_qualified'])

    def test_unrendered_ray_failure_not_semantic_state(self):
        run,c=self.data();c['prefilter']={'temporal_pass_pair_count':1}
        result=classify(c,None,run,CONFIG)
        self.assertEqual(result['failures'],['P_RAY']);self.assertIsNone(result['visibility'])

    def test_no_render_model_or_physics_calls_in_prefilter(self):
        source=Path('research/prefilter_reveal_window.py').read_text();tree=ast.parse(source)
        forbidden={'capture','step','play','reset','create_render_product','get_annotator','author_human'}
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
                self.assertNotIn(node.func.attr,forbidden)
        self.assertIn("'disable_viewport_updates':True",source)
        self.assertNotIn('SavedRenderer',source)

    def test_rejected_candidate_cannot_enter_new_render_loop(self):
        s=Path('research/search_reveal_window.py').read_text()
        self.assertIn("if c.get('prefilter_pass') is False:",s)
        self.assertLess(s.index("if c['id'] in completed:"),s.index("if c.get('prefilter_pass') is False:"))
        self.assertLess(s.index("if c.get('prefilter_pass') is False:"),s.index("author_human(render.stage"))

    def test_base_config_visibility_ranking_unchanged(self):
        self.assertEqual(RULES['model_calls'],0)
        self.assertEqual(CONFIG['visibility']['clearly_visible_min_fraction'],.002)

    def test_legacy_exhaustive_render_refused_before_isaac_start(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'preregistered.json'
            path.write_text(json.dumps({'config':{}}))
            before=path.read_bytes()
            result=subprocess.run([sys.executable,'research/search_reveal_window.py',
                                   '--phase','render','--output-dir',folder],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Cheap prefilter required before rendering',result.stderr)
            self.assertEqual(path.read_bytes(),before)
            self.assertEqual(list(Path(folder).iterdir()),[path])


if __name__=='__main__':unittest.main()
