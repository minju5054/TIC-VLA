"""Synthetic lifecycle tests are not physical navigation evidence."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from research.late_handoff import ApplicationHold, HoldContactStop, delay_ticks, eligible, nonfloor_contact
from research.hospital_late_handoff import preflight, RULES, next_allowed
from research.hospital_no_cart_reproduction import validate_reference
from research.records import RunRecords, clocks
from research.continuous import run_model_loop
from research.reveal_window import evidence_hashes, verify_hashes

ROOT=Path(__file__).resolve().parents[1]
REFS=[ROOT/'outputs'/s for s in ['nova-e16-hospital-lights-baseline-20261010-01','nova-e16-no-cart-15-repro-20261011-01']]


def fixture_cfg():
    return dict(simulation=dict(dt=1/60,pause_physics_during_inference=False,real_time_pacing=True,target_real_time_factor=1,
        control_hz=60,inference_hz=2,predictions=4),controller={},instruction='synthetic',
        robot=dict(start_position=[5,0,0],prim='/Robot'),
        spatial_turn_gate=dict(exit_center_x_max_m=2,center_y_min_m=9,center_y_max_m=11,robot_conservative_radius_m=.6))


def loop_fixture(folder,delay,contact=False,missing=False,corrupt=False):
    cfg=fixture_cfg();records=RunRecords(folder,{})
    hook=ApplicationHold(cfg,dict(trigger_band_south_m=.75),delay,records) if delay is not None else None
    class Sim:
        dt=1/60;pedestrian=None;phase='baseline';pending_request_id=None;active_control_source_request_id=None
        def __init__(self):
            self.tick=0;self.command=[0.,0.];self.wheel_command=[0.,0.];self.trace=[];self.captures=[]
            self.state_file=SimpleNamespace(flush=lambda:None)
            self.hospital_diagnostics=SimpleNamespace(contacts=[],errors=[])
        def state(self):
            y=0 if missing else 8.3
            return dict(tick=self.tick,sim_time=self.tick*self.dt,pose=[5,y,np.pi/2],position=[5,y,0],
                quaternion_wxyz=[1,0,0,0],linear_velocity_world=[0,1,0],**clocks())
        def capture(self,name):
            assert not hook or not hook.holding
            self.captures.append(self.tick);p=records.path/'diagnostics'/name;p.write_bytes(b'synthetic')
            return None,self.state(),p
        def apply(self,*cmd):
            self.command=list(cmd);self.wheel_command=[cmd[0]-cmd[1],cmd[0]+cmd[1]]
            return self.wheel_command
        def step(self):
            self.tick+=1
            self.trace.append((self.tick,self.active_control_source_request_id,self.pending_request_id,list(self.command),list(self.wheel_command)))
            if hook and hook.holding and contact:
                self.hospital_diagnostics.contacts.append(dict(phase='run',collider0='/Robot/body',collider1='/Root/wall',contacts=[dict(normal=[1,0,0])]))
            if hook and hook.holding and corrupt:self.wheel_command[0]+=1
            return self.state()
    sim=Sim()
    class Pending:
        def __init__(self,*args):self.request=None
        def submit(self,request,png):
            assert self.request is None and (hook is None or not hook.holding)
            self.request=request;self.tick=sim.tick
        def done(self):return sim.tick>self.tick
        def take(self):
            rid=self.request['request_id'];self.request=None
            c=np.column_stack([np.arange(1,31)*.1,np.full(30,.05*rid)]).astype(np.float32)
            return dict(raw_action=c.tolist(),inference_start=clocks(),ready=clocks(),native_tensor_shape=[1,30,2])
        def close(self):pass
    class Pacer:
        def __init__(self,*args):pass
        def step(self,callback):return callback()
    with patch('research.continuous.OneRequest',Pending),patch('research.continuous.InferenceClient'),patch('research.continuous.PhysicsPacer',Pacer):
        try:run_model_loop(sim,cfg,records,'static',lambda p:dict(continuous_handoff_validated=True,transport_decision='synthetic'),hook)
        except (HoldContactStop,RuntimeError) as exc:return sim,hook,records,exc
    return sim,hook,records,None


class LateTests(unittest.TestCase):
    def test_quantization(self):
        self.assertEqual([delay_ticks(d,1/60) for d in [0,.25,.5,.75,1]],[0,15,30,45,60])
        for d in [-1,.251,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):delay_ticks(d,1/60)

    def test_zero_compatibility_and_immutability(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=loop_fixture(Path(tmp)/'default',None);b=loop_fixture(Path(tmp)/'zero',0)
            self.assertEqual(a[0].trace,b[0].trace);self.assertEqual(a[0].captures,b[0].captures)
            self.assertIsNone(b[-1]);self.assertEqual(b[1].selected,2)
            hashes=evidence_hashes([b[2].path])
            for rid in range(1,5):
                pa=a[2].path/'raw/requests'/f'request_{rid:06d}';pb=b[2].path/'raw/requests'/pa.name
                np.testing.assert_array_equal(np.load(pa.with_suffix('.npy')),np.load(pb.with_suffix('.npy')))
                ea=json.loads(pa.with_suffix('.json').read_text());eb=json.loads(pb.with_suffix('.json').read_text())
                self.assertEqual(ea['agent_pose_at_observation'],eb['agent_pose_at_observation'])
                self.assertEqual(ea['controller_command'],eb['controller_command'])
            verify_hashes(hashes)
            with self.assertRaises(FileExistsError):RunRecords(b[2].path,{})

    def test_one_hold_old_wheels_switch_and_no_new_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim,hook,records,exc=loop_fixture(Path(tmp)/'held',.75)
            self.assertIsNone(exc);self.assertEqual(hook.selected,2)
            h=json.loads((records.path/'raw/late_handoff/hold.json').read_text())
            self.assertEqual(h['realized_ticks'],45);self.assertAlmostEqual(h['realized_delay_sim_s'],.75)
            rows=[r for r in sim.trace if h['start']['tick']<r[0]<=h['end']['tick']]
            self.assertEqual(len(rows),45)
            self.assertTrue(all(r[1:3]==(1,2) and r[3]==h['old_command'] and r[4]==h['old_wheel_targets'] for r in rows))
            self.assertFalse(any(h['start']['tick']<t<h['end']['tick'] for t in sim.captures))
            events=[json.loads(p.read_text()) for p in sorted((records.path/'raw/requests').glob('*.json'))]
            self.assertEqual(sum('late_handoff' in e for e in events),1)
            self.assertEqual(events[1]['application']['tick'],h['end']['tick'])
            self.assertEqual(sim.trace[h['end']['tick']][1],2)
            np.testing.assert_array_equal(np.load(records.path/'raw/late_handoff/received.npy'),np.load(records.path/'raw/requests/request_000002.npy'))

    def test_contact_stops_with_unapplied_native(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim,hook,records,exc=loop_fixture(Path(tmp)/'contact',1,contact=True)
            self.assertIsInstance(exc,HoldContactStop);self.assertEqual(records.ids,{1})
            h=json.loads((records.path/'raw/late_handoff/hold.json').read_text())
            self.assertFalse(h['applied']);self.assertEqual(h['realized_ticks'],1)
            self.assertTrue((records.path/'raw/late_handoff/received.npy').exists())
            self.assertEqual(len(sim.captures),2)

    def test_missing_trigger_and_retention_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim,hook,records,exc=loop_fixture(Path(tmp)/'missing',1,missing=True)
            self.assertIsNone(exc);self.assertIsNone(hook.selected);self.assertEqual(len(records.ids),4)
            sim,hook,records,exc=loop_fixture(Path(tmp)/'corrupt',1,corrupt=True)
            self.assertIsInstance(exc,RuntimeError);self.assertEqual(len(records.ids),1)

    def test_trigger_entry_and_direction(self):
        cfg=fixture_cfg();st=dict(pose=[5,8.5,np.pi/2],linear_velocity_world=[0,1,0])
        self.assertTrue(eligible(st,cfg,.75));self.assertFalse(eligible(st,cfg,.75,True))
        st['linear_velocity_world'][1]=-1;self.assertFalse(eligible(st,cfg,.75))

    def test_stopping_gates(self):
        good=dict(outcome='A',baseline_gate_pass=True);bad=dict(outcome='D',baseline_gate_pass=False)
        trials=[{}]*10
        self.assertTrue(next_allowed(trials,[bad])[0]) # Always complete both zero controls unless runtime failure.
        self.assertFalse(next_allowed(trials,[good,bad])[0])
        self.assertFalse(next_allowed(trials,[good,good,bad,bad])[0])
        self.assertTrue(next_allowed(trials,[good,good,bad,good])[0])
        self.assertFalse(next_allowed(trials,[dict(outcome='F')])[0])

    def test_result_classification(self):
        from research.analyze_late_handoff import outcome
        self.assertEqual(outcome(True,True,None,10.,True),'A')
        self.assertEqual(outcome(True,True,11.,10.,True),'B')
        self.assertEqual(outcome(True,True,None,None,False),'C')
        self.assertEqual(outcome(False,True,9.,None,False),'D')
        self.assertEqual(outcome(True,False,None,10.,True),'E')
        self.assertEqual(outcome(False,False,None,None,False,False),'F')

    @unittest.skipUnless(all((p/'summary.json').exists() for p in REFS),'Saved local references absent')
    def test_analysis_serializes_before_exclusive_write(self):
        from research.analyze_late_handoff import analyze
        from research.analyze_e16_handoff_audit import serial
        for path in REFS:
            _,result=analyze(path,True)
            text=json.dumps(serial(result),allow_nan=False)
            self.assertTrue(json.loads(text)['metrics']['baseline_gate_pass'])

    @unittest.skipUnless(all((p/'summary.json').exists() for p in REFS),'Saved local references absent')
    def test_saved_preflight_config_and_source_immutability(self):
        hashes=evidence_hashes(REFS);rules=json.loads(RULES.read_text())
        a=preflight(REFS,rules);b=preflight(REFS,rules);self.assertEqual(a,b)
        self.assertEqual([r['selected']['request_id'] for r in a['references']],[13,13])
        cfg=json.loads((REFS[0]/'metadata.json').read_text())['config'];validate_reference(cfg,copy.deepcopy(cfg))
        changed=copy.deepcopy(cfg);changed['controller']['max_v']=1.2
        with self.assertRaises(ValueError):validate_reference(cfg,changed)
        verify_hashes(hashes)


if __name__=='__main__':unittest.main()
