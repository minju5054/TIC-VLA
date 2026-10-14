"""Synthetic fixtures test contracts only; never used as E16 experiment evidence."""
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from research.control import waypoint_command
from research.records import write_json
from tools.e16_wheelchair_delay.common import (
    ticks_for,unicycle,trigger_check,fully_passed,movement,motion_direction,boundary_metrics,
    to_world,rotation,wrap,remaining,contact_is_nonfloor,contact_kind,PLAN)
from tools.e16_wheelchair_delay.hook import C10ApplicationHold,DiagnosticStop
from tools.e16_wheelchair_delay.evidence import load_run

POLICY=dict(min_segment_m=.02,max_adjacent_heading_change_deg=45,safe_proxy_margin_m=.02)
RULES=dict(center_world_xy_m=[0,0],progress_yaw_rad=0,reference_yaw_rad=0,
           half_along_m=.5,half_lateral_m=.6,yaw_half_width_rad=np.pi/6,min_forward_speed_m_s=.05,
           c10_center_xy_m=[2,0],c10_forward_world=[1,0],c10_complete_passage_offset_m=1.)


def obs(tick=0,time=0,x=0):
    return dict(tick=tick,sim_time=time,pose=[x,0,0],position=[x,0,0],quaternion_wxyz=[1,0,0,0],
                linear_velocity_world=[.8,0,0],linear_velocity_body_flu=[.8,0,0],angular_velocity_body_flu=[0,0,0],
                monotonic_ns=tick*10_000_000+5_000_000,wall_time_ns=tick*10_000_000+5_000_000)


class FakeSim:
    dt=1/60
    def __init__(self,contact_tick=None):
        self.tick=0;self.phase='run';self.active_control_source_request_id=1;self.pending_request_id=2
        self.command=[.8,0.];self.wheel_command=[1.,1.];self.hospital_diagnostics=SimpleNamespace(contacts=[],errors=[])
        self.contact_tick=contact_tick
    def state(self):return obs(self.tick,self.tick*self.dt,self.tick*.8*self.dt)
    def advance(self):
        self.tick+=1
        if self.tick==self.contact_tick:
            self.hospital_diagnostics.contacts.append(dict(phase='run',sim_time=self.tick*self.dt,
                collider0='/Robot/base',collider1='/Wall',contacts=[dict(normal=[1,0,0],impulse=[0,0,0],separation=.002)]))
        return self.state()


class MathTests(unittest.TestCase):
    def test_exact_grid(self):
        self.assertEqual(ticks_for(.25),15);self.assertEqual(ticks_for(0),0)
        for x in (-1,.251,float('nan')):
            with self.assertRaises(ValueError):ticks_for(x)
    def test_unicycle_straight(self):np.testing.assert_allclose(unicycle([1,2,0],[.8,0],.25),[1.2,2,0])
    def test_unicycle_arc(self):np.testing.assert_allclose(unicycle([0,0,0],[1,1],np.pi/2),[1,1,np.pi/2])
    def test_se2_world_and_wrap(self):
        np.testing.assert_allclose(to_world([[1,0]],[3,4,np.pi/2]),[[3,5]])
        self.assertAlmostEqual(wrap(np.pi+.2),-np.pi+.2)
    def test_no_extrapolation(self):
        t=np.arange(1,31)*.1;w=np.column_stack([t,np.zeros(30)])
        r=boundary_metrics(w,t,[0,0,0],.05,0,POLICY)
        self.assertFalse(r['nominal_valid']);self.assertIsNone(r['position_error_m'])
        r=boundary_metrics(w,t,[4,0,0],3.2,0,POLICY);self.assertFalse(r['nominal_valid'])
    def test_same_fresh_fixed_vs_time_aligned(self):
        t=np.arange(1,31)*.1;w=np.column_stack([t,np.zeros(30)])
        b=boundary_metrics(w,t,[.5,0,0],.5,0,POLICY,.5)
        a=boundary_metrics(w,t,[.75,0,0],.75,0,POLICY,.5)
        self.assertAlmostEqual(a['position_error_m'],0)
        self.assertAlmostEqual(a['fixed_reference_position_error_m']-b['fixed_reference_position_error_m'],.25)
    def test_short_tangent_undefined(self):
        r=boundary_metrics(np.array([[0,0],[.001,0],[1,0]]),np.array([.1,.2,.3]),[0,0,0],.1,0,POLICY)
        self.assertFalse(r['tangent']['valid']);self.assertIsNone(r['direction_error_rad'])
    def test_measured_B0_Bdelta(self):
        r=movement([[0,0],[.1,0],[.1,.1]],[0,.1,.2],0,2)
        self.assertAlmostEqual(r['path_m'],.2);self.assertAlmostEqual(r['net_displacement_m'],math.sqrt(.02))
        self.assertAlmostEqual(r['yaw_change_rad'],.2)
    def test_recent_direction(self):
        r=motion_direction(np.array([[0,0],[.03,.03]]),1)
        self.assertTrue(r['valid']);self.assertAlmostEqual(r['yaw_rad'],np.pi/4)
    def test_primary_before_goal(self):
        from research.audit_e16_wheelchair_handoffs import is_primary
        self.assertTrue(is_primary(1,2));self.assertFalse(is_primary(2,2))


class TriggerTests(unittest.TestCase):
    def test_observation_only(self):
        self.assertTrue(trigger_check(obs(),RULES,1)['eligible'])
        self.assertFalse(trigger_check(obs(x=.51),RULES,1)['eligible'])
        s=obs();s['pose'][2]=np.pi/2;self.assertFalse(trigger_check(s,RULES,1)['eligible'])
    def test_no_bootstrap_prior_contact_or_passage(self):
        self.assertFalse(trigger_check(obs(),RULES,None)['eligible'])
        self.assertFalse(trigger_check(obs(),RULES,1,contact_before=True)['eligible'])
        self.assertFalse(trigger_check(obs(),RULES,1,passed_before=True)['eligible'])
    def test_stationary_not_normal_forward(self):
        s=obs();s['linear_velocity_body_flu'][0]=0
        self.assertFalse(trigger_check(s,RULES,1)['eligible'])
    def test_maximum_plan_and_actual_delay(self):
        self.assertEqual([m for m,d in PLAN[:4]],['NO_HOOK','ZERO_HOOK','ZERO_HOOK','NO_HOOK'])
        self.assertEqual([d for m,d in PLAN[4:]],[.25,.25])


class HookTests(unittest.TestCase):
    def invoke(self,folder,seconds=0,sim=None,request=None):
        sim=sim or FakeSim();r=request or dict(request_id=2,observation=obs())
        h=C10ApplicationHold({'simulation':{'dt':1/60},'robot':{'prim':'/Robot'}},
                             {'trigger':RULES,'nominal_horizon_s':3.},seconds,SimpleNamespace(path=folder))
        old=dict(old_control_source_request_id=1,old_controller_command=[.8,0.],old_wheel_commands=[1.,1.])
        chunk=np.column_stack([np.arange(1,31)*.1,np.zeros(30)]).astype(np.float32)
        def call():return h.before_application(sim,r,{},old,chunk,[.8,.1],3,.1,sim.state(),sim.advance)
        return sim,h,chunk,call
    def test_zero_no_extra_physics_and_identical_fresh(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').mkdir();s,h,c,f=self.invoke(p);before=c.tobytes();anchor=obs()['pose']
            r=f();self.assertEqual(s.tick,0);self.assertEqual(r['realized_ticks'],0)
            self.assertEqual(c.tobytes(),before);self.assertEqual(r['anchor'],anchor)
            self.assertTrue(r['fresh_application_authorized'])
    def test_fifteen_ticks_old_retained(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').mkdir();s,h,c,f=self.invoke(p,.25);r=f()
            self.assertEqual(s.tick,15);self.assertEqual(r['realized_ticks'],15)
            self.assertEqual(s.command,[.8,0.]);self.assertEqual(s.wheel_command,[1,1]);self.assertEqual(s.pending_request_id,2)
            self.assertEqual(len(r['hold_states']),15)
    def test_at_most_once(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').mkdir();s,h,c,f=self.invoke(p,.25);f();self.assertIsNone(f());self.assertEqual(s.tick,15)
    def test_trigger_absent_no_replacement(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').mkdir();s,h,c,f=self.invoke(p,.25,request=dict(request_id=2,observation=obs(x=9)))
            self.assertIsNone(f());self.assertIsNone(h.selected);self.assertEqual(s.tick,0)
    def test_contact_stop_before_fresh_even_zero_impulse(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').mkdir();s,h,c,f=self.invoke(p,.25,sim=FakeSim(contact_tick=3))
            with self.assertRaisesRegex(DiagnosticStop,'CONTACT_DURING_HOLD'):f()
            r=json.loads((p/'raw/delay/hold.json').read_text());self.assertEqual(s.tick,3)
            self.assertFalse(r['fresh_application_authorized']);self.assertEqual(s.active_control_source_request_id,1)
            self.assertTrue(r['contact_kind']['zero_impulse_nonpenetrating_callback'])
    def test_horizon_expires_no_hold_no_application(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'raw').mkdir();sim=FakeSim();sim.tick=170
            s,h,c,f=self.invoke(p,.25,sim=sim)
            with self.assertRaisesRegex(DiagnosticStop,'FRESH_HORIZON_EXPIRED'):f()
            self.assertEqual(s.tick,170)
    def test_reject_unauthorized_actual_sweep(self):
        with self.assertRaises(ValueError):C10ApplicationHold({}, {}, .5, None)


class EvidenceTests(unittest.TestCase):
    def make(self,path,count=90):
        controller=dict(lookahead=1.,angular_gain=.8,filter_gain=.35,epsilon=.001,max_v=.8,max_w=1.2)
        cfg=dict(simulation=dict(predictions=90,dt=1/60,pause_physics_during_inference=False),controller=controller,instruction='fixture')
        write_json(path/'metadata.json',dict(config=cfg,command_variant='direct'))
        write_json(path/'inference-runtime.json',dict(strict_checkpoint=True,simulation_app_started=False))
        (path/'raw/requests').mkdir(parents=True);(path/'diagnostics').mkdir()
        chunk=np.column_stack([np.arange(1,31)*.1,np.zeros(30)]).astype(np.float32)
        yf=None;commands=[]
        for i in range(count):
            cmd,yf,idx=waypoint_command(chunk,yf,**controller);commands.append(list(cmd))
        allrows=[]
        for tick in range(count*3+1):
            active=None if tick<=1 else min(count,(tick-2)//3+1)
            pending=(tick-1)//3+1 if tick%3==1 else None
            cmd=commands[active-1] if active else [0.,0.];wheels=[1.,1.] if active else [0.,0.]
            r=dict(tick=tick,sim_time=tick/60,x=max(0,tick-1)*.8/60,y=0,yaw=0,qw=1,qx=0,qy=0,qz=0,
                   monotonic_ns=tick*10_000_000+5_000_000,physics_step_start_monotonic_ns=(tick-1)*10_000_000+8_000_000,
                   active_control_source_request_id=active,pending_request_id=pending,
                   target_v=cmd[0],target_w=cmd[1],applied_v=cmd[0],applied_w=cmd[1],wheel_target_left=wheels[0],wheel_target_right=wheels[1])
            allrows.append(r)
        with (path/'robot_state.csv').open('x') as f:
            w=csv.DictWriter(f,fieldnames=allrows[0].keys());w.writeheader();w.writerows(allrows)
        yf=None
        for rid in range(1,count+1):
            a=(rid-1)*3;b=a+1
            def state(tick,offset):
                r=allrows[tick];return dict(tick=tick,sim_time=tick/60,pose=[r['x'],0,0],quaternion_wxyz=[1,0,0,0],monotonic_ns=r['monotonic_ns']+offset)
            ob=state(a,100);det=state(b,20);app=state(b,30)
            cmd,yf,idx=waypoint_command(chunk,yf,**controller)
            e=dict(request_id=rid,bootstrap=rid==1,old_control_source_request_id=rid-1 if rid>1 else None,
                old_controller_command=commands[rid-2] if rid>1 else None,old_wheel_commands=[1,1] if rid>1 else None,
                native_frame='observation_body',native_axes=['forward','left'],native_units='meters',native_tensor_shape=[1,30,2],raw_action_shape=[30,2],native_yaw=None,
                agent_pose_at_ready=None,agent_pose_at_observation=ob['pose'],observation=ob,response_detected=det,application=app,
                request_submit={'monotonic_ns':ob['monotonic_ns']+1},inference_start={'monotonic_ns':ob['monotonic_ns']+2},ready={'monotonic_ns':det['monotonic_ns']-1},
                instruction='fixture',controller_command=list(cmd),yaw_filter_state=yf,lookahead_index=idx,wheel_commands=[1,1],rgb_observation_reference=f'diagnostics/{rid}.png')
            write_json(path/'raw/requests'/f'request_{rid:06d}.json',e);np.save(path/'raw/requests'/f'request_{rid:06d}.npy',chunk)
            (path/'diagnostics'/f'{rid}.png').write_bytes(b'fixture only')
    def test_full_90_contract_and_raw_read_only(self):
        from tools.e16_wheelchair_delay.common import snapshot,verify_snapshot
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.make(p);before=snapshot(p);r=load_run(p)
            self.assertEqual(len(r['events']),90);verify_snapshot(p,before)
    def test_48_not_complete_90(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.make(p,48)
            with self.assertRaises(ValueError):load_run(p)
    def test_row_pose_alignment_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.make(p);e=p/'raw/requests/request_000090.json';v=json.loads(e.read_text());v['application']['pose'][0]+=1;e.write_text(json.dumps(v))
            with self.assertRaisesRegex(ValueError,'state and robot row'):load_run(p)
    def test_raw_no_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'record.json';write_json(p,{'x':1})
            with self.assertRaises(FileExistsError):write_json(p,{'x':2})


if __name__=='__main__':unittest.main()
