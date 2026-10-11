"""Model-free protocol and evidence checks; synthetic runs are not navigation trials."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from research.e16_zero_repeatability import ORDER,plan,prior_gate,choose_hook,audit_saved
from research.hospital_no_cart_reproduction import validate_reference
from research.records import RunRecords
from research.reveal_window import evidence_hashes,verify_hashes
from tests.test_late_handoff import loop_fixture,fixture_cfg

ROOT=Path(__file__).resolve().parents[1]
REFS=[ROOT/'outputs'/p for p in ['nova-e16-hospital-lights-baseline-20261010-01','nova-e16-no-cart-15-repro-20261011-01',
    'nova-e16-late-20261011-01-d000-r1','nova-e16-late-20261011-01-d000-r2']]


class RepeatabilityTests(unittest.TestCase):
    def test_fixed_plan_exclusive_ids(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);trials=plan('test',root)
            self.assertEqual([t['condition'] for t in trials],['NO_HOOK','ZERO_HOOK','ZERO_HOOK','NO_HOOK','NO_HOOK','ZERO_HOOK'])
            self.assertEqual(len({t['run_id'] for t in trials}),6)
            (root/'outputs'/trials[0]['run_id']).mkdir(parents=True)
            with self.assertRaises(FileExistsError):plan('test',root)

    def test_continue_navigation_failure_stop_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);trials=plan('test',root);prior_gate(trials,0,root)
            p=root/'outputs'/trials[0]['run_id'];p.mkdir(parents=True)
            s=dict(runtime_status='PASS',successful_predictions=48,repeatability_contract_validated=True,navigation='FAIL')
            (p/'summary.json').write_text(json.dumps(s));prior_gate(trials,1,root)
            with self.assertRaises(ValueError):prior_gate(trials,2,root)
            (p/'failure.json').write_text('{}')
            with self.assertRaises(ValueError):prior_gate(trials,1,root)

    def test_no_hook_does_not_construct_hook(self):
        with patch('research.e16_zero_repeatability.ApplicationHold') as constructor:
            self.assertIsNone(choose_hook('NO_HOOK',{}, {},None));constructor.assert_not_called()
            choose_hook('ZERO_HOOK',{}, {},None);constructor.assert_called_once_with({}, {},0.,None)

    def test_same_config_no_extra_physics_original_native(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=loop_fixture(Path(tmp)/'a',None);b=loop_fixture(Path(tmp)/'b',0.)
            self.assertIsNone(a[-1]);self.assertIsNone(b[-1]);self.assertEqual(a[0].trace,b[0].trace);self.assertEqual(a[0].captures,b[0].captures)
            self.assertFalse((a[2].path/'raw/late_handoff').exists())
            h=json.loads((b[2].path/'raw/late_handoff/hold.json').read_text());self.assertEqual(h['realized_ticks'],0);self.assertTrue(h['fresh_unchanged'])
            hashes=evidence_hashes([a[2].path,b[2].path]);verify_hashes(hashes)
            with self.assertRaises(FileExistsError):RunRecords(a[2].path,{})

    @unittest.skipUnless(all((p/'summary.json').exists() for p in REFS),'Local saved evidence absent')
    def test_saved_contracts_config_controller_immutability(self):
        hashes=evidence_hashes(REFS);cfg=json.loads((REFS[0]/'metadata.json').read_text())['config']
        rules=json.loads((ROOT/'configs/research/e16_late_handoff_stress.json').read_text())
        for i,p in enumerate(REFS):
            validate_reference(cfg,json.loads((p/'metadata.json').read_text())['config'])
            result=audit_saved(p,'NO_HOOK' if i<2 else 'ZERO_HOOK',rules)
            self.assertEqual(result['controller_reconstructed_count'],48);self.assertEqual(result['selected_request_id'],13)
            self.assertEqual(result['additional_physics_ticks'],0)
        altered=copy.deepcopy(cfg);altered['controller']['filter_gain']=.4
        with self.assertRaises(ValueError):validate_reference(cfg,altered)
        with self.assertRaises(ValueError):audit_saved(REFS[2],'NO_HOOK',rules)
        verify_hashes(hashes)


if __name__=='__main__':unittest.main()
