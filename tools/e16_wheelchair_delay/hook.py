"""Opt-in one-shot post-response hold for 90-prediction E16 trials."""
import hashlib
import time
import numpy as np
from research.records import write_json
from tools.e16_wheelchair_delay.common import (
    ticks_for, trigger_check, fully_passed, contact_is_nonfloor, contact_kind,
)


class DiagnosticStop(RuntimeError):
    def __init__(self, status):
        self.status = status
        super().__init__(status)


class C10ApplicationHold:
    def __init__(self, cfg, protocol, seconds, records):
        if seconds not in (0., .25):
            raise ValueError('Only zero or 0.25 s actual application hold is authorized')
        self.cfg, self.protocol, self.seconds, self.records = cfg, protocol, seconds, records
        self.ticks = ticks_for(seconds, cfg['simulation']['dt'])
        self.selected = None; self.first_passage = None; self.calls = []

    def observe_state(self, state):
        if self.first_passage is None and fully_passed(state['pose'], self.protocol['trigger']):
            self.first_passage = state['sim_time']

    def before_application(self, sim, request, response, old, chunk, command, index, yaw_filter, detected, advance):
        wall_start = time.monotonic_ns(); obs = request['observation']
        past_contact = any(contact_is_nonfloor(e, self.cfg['robot']['prim']) and e['sim_time'] <= obs['sim_time']+1e-9
                           for e in sim.hospital_diagnostics.contacts)
        check = trigger_check(obs, self.protocol['trigger'], old['old_control_source_request_id'],
                              self.first_passage is not None and self.first_passage <= obs['sim_time'], past_contact)
        if self.selected is not None or not check['eligible']:
            self.calls.append(dict(request_id=request['request_id'], eligible=check['eligible'],
                                   selected=False, overhead_wall_ns=time.monotonic_ns()-wall_start))
            return None
        self.selected = request['request_id']
        folder = self.records.path/'raw/delay'; folder.mkdir(exist_ok=False)
        original=chunk.copy(); anchor=list(obs['pose']); proposal=list(command)
        start=sim.state(); start['event_source']='validated_response_before_additional_hold'
        with (folder/'received.npy').open('xb') as f: np.save(f,chunk,allow_pickle=False)
        write_json(folder/'received.json',dict(request=request,response=response,old=old,
            response_detected=detected,controller_command=proposal,lookahead_index=index,
            yaw_filter_state=yaw_filter,agent_pose_at_observation=anchor,
            native_frame='observation_body',native_axes=['forward','left'],native_units='meters',
            native_yaw=None,raw_action_shape=[30,2],trigger=check))
        result=dict(request_id=self.selected,old_request_id=old['old_control_source_request_id'],
                    requested_delay_s=self.seconds,requested_ticks=self.ticks,start=start,
                    fresh_application_authorized=False,stop_status=None,contact=None,
                    native_sha256=hashlib.sha256(chunk.tobytes()).hexdigest(),anchor=anchor,
                    old_command=old['old_controller_command'],old_wheel_targets=old['old_wheel_commands'],
                    fresh_command=proposal,trigger=check,
                    intervention='Post-response application hold; not additional inference latency')
        phase=sim.phase; cursor=len(sim.hospital_diagnostics.contacts); states=[]
        try:
            horizon=obs['sim_time']+self.protocol['nominal_horizon_s']
            if start['sim_time']+self.ticks*sim.dt >= horizon-1e-7:
                result['stop_status']='FRESH_HORIZON_EXPIRED'
                raise DiagnosticStop(result['stop_status'])
            for _ in range(self.ticks):
                sim.phase='e16_application_hold'
                state=advance(); states.append(state)
                if (sim.active_control_source_request_id!=old['old_control_source_request_id']
                    or sim.pending_request_id!=self.selected
                    or list(sim.command)!=old['old_controller_command']
                    or not np.allclose(sim.wheel_command,old['old_wheel_commands'],rtol=0,atol=1e-12)):
                    raise RuntimeError('OLD command/source/wheel target changed during hold')
                if sim.hospital_diagnostics.errors:
                    raise RuntimeError('Contact logger failed during hold')
                hits=[e for e in sim.hospital_diagnostics.contacts[cursor:] if contact_is_nonfloor(e,self.cfg['robot']['prim'])]
                cursor=len(sim.hospital_diagnostics.contacts)
                if hits:
                    result.update(contact=hits[0],contact_kind=contact_kind(hits[0]),stop_status='CONTACT_DURING_HOLD')
                    raise DiagnosticStop(result['stop_status'])
                if state['sim_time'] >= horizon-1e-7:
                    result['stop_status']='FRESH_HORIZON_EXPIRED'
                    raise DiagnosticStop(result['stop_status'])
            result['fresh_application_authorized']=True
        finally:
            end=sim.state();end['event_source']='end_of_hold_before_application_or_stop'
            result.update(end=end,realized_ticks=end['tick']-start['tick'],
                          realized_delay_sim_s=end['sim_time']-start['sim_time'],hold_states=states,
                          hook_total_wall_ns=time.monotonic_ns()-wall_start)
            np.testing.assert_array_equal(chunk,original)
            if obs['pose']!=anchor or list(command)!=proposal:
                raise RuntimeError('FRESH anchor/command changed')
            result['fresh_unchanged']=True;sim.phase=phase
            self.calls.append(dict(request_id=self.selected,eligible=True,selected=True,
                                   overhead_wall_ns=result['hook_total_wall_ns']))
            write_json(folder/'hold.json',result)
        return result

    def finalize(self):
        write_json(self.records.path/'raw/hook-calls.json',dict(selected_request_id=self.selected,
                   calls=self.calls,first_complete_c10_passage_s=self.first_passage,
                   note='Selected-call wall cost includes disk diagnostics and paced hold, not pure code overhead'))
