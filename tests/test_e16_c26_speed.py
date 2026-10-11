import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from research.e16_c26_speed_ablation import (
    config_diff, validate_ablation, validate_authorization, EXPECTED_DIFF, SCOPE, freeze)
from research.analyze_e16_c26_speed import classify
from research.hospital_episode import digest
from research.reveal_window import evidence_hashes, verify_hashes


class SpeedAblationTests(unittest.TestCase):
    def test_only_speed_changes_and_source_is_immutable(self):
        old = dict(controller=dict(max_v=1.5,max_w=1.2,lookahead=1.,filter_gain=.35),
                   simulation=dict(predictions=48),stationary_cart=dict(intervention='EXACT_C26',pose=[1,2,3]),seed=36)
        before=copy.deepcopy(old);new=copy.deepcopy(old);new['controller']['max_v']=1.2
        with patch('research.e16_c26_speed_ablation.validate_config'):
            validate_ablation(old,new,old['stationary_cart'])
            self.assertEqual(config_diff(old,new),EXPECTED_DIFF);self.assertEqual(old,before)
            for key,value in [('max_w',1.),('lookahead',.8),('filter_gain',.3)]:
                wrong=copy.deepcopy(new);wrong['controller'][key]=value
                with self.assertRaises(ValueError):validate_ablation(old,wrong,old['stationary_cart'])
            for key,value in [('simulation',dict(predictions=60)),('stationary_cart',dict(intervention='EXACT_C26',pose=[2,2,3])),('seed',37)]:
                wrong=copy.deepcopy(new);wrong[key]=value
                with self.assertRaises(ValueError):validate_ablation(old,wrong,old['stationary_cart'])
            wrong=copy.deepcopy(new);wrong['unexpected_null']=None
            with self.assertRaises(ValueError):validate_ablation(old,wrong,old['stationary_cart'])

    def test_new_scope_source_run_and_config_binding(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);pre=p/'pre';ref=p/'ref';pre.mkdir();ref.mkdir()
            (pre/'freeze.json').write_text('{}');(ref/'metadata.json').write_text('{}')
            physical={'pass':False,'checks':{k:True for k in ['floor_support','no_static_overlap','no_initial_collision','baseline_intersects_inflated_cart']}}
            physical['checks']['local_bypass']=False;(pre/'physical.json').write_text(json.dumps(physical))
            (pre/'summary.json').write_text(json.dumps(dict(cart_visible_at_C1=False,stop_reasons=['PHYSICAL_PLACEMENT_INVALID','NO_FEASIBLE_BYPASS'])))
            auth=dict(scope=SCOPE,run_id='one',reference_run=str(ref),reference_metadata_sha256=digest(ref/'metadata.json'),
                preflight_freeze_sha256=digest(pre/'freeze.json'),config_sha256='cfg',max_v=1.2,predictions=48,user_instruction='one slower diagnostic')
            protected=evidence_hashes([pre,ref]);validate_authorization(auth,'one',pre,ref,'cfg');verify_hashes(protected)
            for k,v in [('scope','ONE_UNCHANGED_C26_RUN_DESPITE_BYPASS_FAILURE'),('run_id','two'),('max_v',1.5),('predictions',60),('config_sha256','other')]:
                with self.assertRaises(ValueError):validate_authorization(dict(auth,**{k:v}),'one',pre,ref,'cfg')

    def test_fresh_destination_rejected_before_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);out=p/'existing';out.mkdir();(out/'sentinel').write_text('preserve')
            with self.assertRaises(FileExistsError):freeze(p/'ref',p/'pre',out,'unused',p/'instruction',p/'probe')
            self.assertEqual((out/'sentinel').read_text(),'preserve')

    def test_passage_and_contact_are_distinct(self):
        no=dict(clear_region=None,clears_before_any_contact=False,geometric_passage_side='UNKNOWN / DOES NOT REACH')
        self.assertEqual(classify(True,no,None,None),'INCOMPLETE APPROACH / DOES NOT REACH CART')
        self.assertEqual(classify(False,no,None,None),'EXECUTION FAILED')
        self.assertEqual(classify(True,no,13.,19.),'CONTACT BEFORE BYPASS')
        self.assertEqual(classify(True,no,None,9.),'OTHER STRUCTURAL CONTACT BEFORE CART')
        clear=dict(clear_region={'sim_time':14.},clears_before_any_contact=True,geometric_passage_side='SOUTH/LEFT BYPASS')
        self.assertEqual(classify(True,clear,None,None),'COLLISION-FREE COMPLETE BYPASS')
        # Contact after centre crossing but before downstream clearance still fails.
        self.assertEqual(classify(True,dict(clear,clears_before_any_contact=False),13.,None),'CONTACT BEFORE BYPASS')


if __name__=='__main__':unittest.main()
