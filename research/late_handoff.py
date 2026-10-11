"""Opt-in post-response physics hold. No model/controller/trajectory modification."""
import hashlib
import json
from pathlib import Path
import numpy as np
from research.analyze_no_cart_speed import in_corner
from research.analyze_chunk_geometry import project_world, wrap
from research.records import write_json
from research.control import validate_chunk, waypoint_command
from research.nova_response import read_csv
from research.analyze_nova_response import contacts


def delay_ticks(seconds, dt):
    if not np.isfinite(seconds) or seconds < 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError("Invalid hold duration/physics dt")
    ticks = round(seconds/dt)
    if abs(ticks*dt-seconds) > 1e-9:
        raise ValueError('Hold must be nonnegative and exactly on the physics grid')
    return ticks


def eligible(state, cfg, band, entered=False):
    x, y, yaw = state['pose']; g = cfg['spatial_turn_gate']
    return bool(not entered and g['exit_center_x_max_m'] < x <= cfg['robot']['start_position'][0]+g['robot_conservative_radius_m']
        and g['center_y_min_m']-band <= y < g['center_y_min_m']
        and np.sin(yaw) > 0 and state['linear_velocity_world'][1] > 0)


def nonfloor_contact(events, robot_prim):
    return next((e for e in events if e['phase'] == 'run' and e['contacts']
        and not all(e[k].startswith(robot_prim+'/') for k in ['collider0', 'collider1'])
        and any(abs(c['normal'][2]) < .5 for c in e['contacts'])), None)


class HoldContactStop(RuntimeError):
    """Completed diagnostic stop, not an inference error or fabricated application."""


class ApplicationHold:
    def __init__(self, cfg, rules, seconds, records):
        self.cfg, self.rules, self.seconds, self.records = cfg, rules, seconds, records
        self.ticks = delay_ticks(seconds, cfg['simulation']['dt'])
        self.entered = False; self.selected = None; self.holding = False

    def before_application(self, sim, request, response, old, chunk, command, index, yaw_filter, detected, advance):
        # Use observation-time history, not later response-time entry status.
        obs = request['observation']
        observed_entry = getattr(self, 'entry_time', None)
        if self.selected is not None or old['old_control_source_request_id'] is None or not eligible(
                obs, self.cfg, self.rules['trigger_band_south_m'], observed_entry is not None and observed_entry <= obs['sim_time']):
            return None
        self.selected = request['request_id']; self.holding = True
        folder = self.records.path/'raw/late_handoff'; folder.mkdir()
        original = chunk.copy(); anchor = list(obs['pose']); proposal = list(command)
        start = sim.state(); start['event_source'] = 'post_validation_pre_additional_hold'
        with (folder/'received.npy').open('xb') as f:
            np.save(f, chunk, allow_pickle=False)
        received = dict(request=request, response=response, old=old, response_detected=detected,
            controller_command=proposal, lookahead_index=index, yaw_filter_state=yaw_filter,
            agent_pose_at_observation=anchor, native_frame='observation_body', native_axes=['forward','left'], native_units='meters')
        write_json(folder/'received.json', received)
        result = dict(request_id=self.selected, old_request_id=old['old_control_source_request_id'],
            requested_delay_s=self.seconds, requested_ticks=self.ticks, start=start, applied=False, contact=None,
            native_sha256=hashlib.sha256(chunk.tobytes()).hexdigest(), old_command=old['old_controller_command'],
            old_wheel_targets=old['old_wheel_commands'], fresh_command=proposal,
            intervention='Artificial post-response hold, not model inference latency')
        old_phase = sim.phase; cursor = len(sim.hospital_diagnostics.contacts)
        hold_rows = []
        try:
            for _ in range(self.ticks):
                sim.phase = 'late_handoff_hold'
                state = advance()
                if (sim.active_control_source_request_id != old['old_control_source_request_id']
                        or sim.pending_request_id != self.selected
                        or list(sim.command) != old['old_controller_command']
                        or not np.allclose(sim.wheel_command, old['old_wheel_commands'], rtol=0, atol=1e-12)):
                    raise RuntimeError('OLD command/source/wheels changed during hold')
                hold_rows.append(state['tick'])
                if sim.hospital_diagnostics.errors:
                    raise RuntimeError('Contact logger failed during hold')
                hit = nonfloor_contact(sim.hospital_diagnostics.contacts[cursor:], self.cfg['robot']['prim'])
                cursor = len(sim.hospital_diagnostics.contacts)
                if hit:
                    result.update(contact=hit, applied=False)
                    raise HoldContactStop('Actual nonfloor contact during held-response execution')
            result.update(applied=True, contact=None)
        finally:
            end = sim.state(); end['event_source'] = 'end_of_additional_hold_before_application_or_stop'
            result.update(end=end, realized_ticks=end['tick']-start['tick'],
                realized_delay_sim_s=end['sim_time']-start['sim_time'], hold_ticks=hold_rows,
                hold_translation_xy_m=float(np.linalg.norm(np.asarray(end['position'][:2])-start['position'][:2])),
                hold_yaw_change_deg=float(np.degrees(wrap(end['pose'][2]-start['pose'][2]))))
            np.testing.assert_array_equal(chunk, original)
            if obs['pose'] != anchor or list(command) != proposal:
                raise RuntimeError('FRESH anchor/command changed during hold')
            result['fresh_unchanged'] = True
            sim.phase = old_phase; self.holding = False
            write_json(folder/'hold.json', result)
        return result

    def observe_state(self, state):
        if bool(in_corner(state['position'][:2], self.cfg)) and not self.entered:
            self.entry_time = state['sim_time']; self.entered = True


def load_late(path):
    """Validate actual pending+held ticks without weakening historical validators."""
    path = Path(path).resolve(); meta = json.loads((path/'metadata.json').read_text()); cfg = meta['config']
    runtime = json.loads((path/'inference-runtime.json').read_text())
    if not runtime['strict_checkpoint'] or runtime['simulation_app_started'] or cfg['simulation']['pause_physics_during_inference'] or meta['command_variant'] != 'direct' or cfg['robot']['type'] != 'nova_carter':
        raise ValueError('Invalid model/continuous contract')
    rows = read_csv(path/'robot_state.csv'); files = sorted((path/'raw/requests').glob('*.json'))
    if not files or len(files) > 48 or {p.stem for p in files} != {p.stem for p in (path/'raw/requests').glob('*.npy')}:
        raise ValueError('Missing/extra applied request arrays')
    events, chunks, yf, inside = [], [], None, []
    for rid, file in enumerate(files, 1):
        e = json.loads(file.read_text()); c = validate_chunk(np.load(file.with_suffix('.npy'), allow_pickle=False))
        if (e['request_id'] != rid or e['native_frame'] != 'observation_body' or e['native_axes'] != ['forward','left']
                or e['native_units'] != 'meters' or e['native_tensor_shape'] != [1,30,2]
                or e['bootstrap'] != (rid == 1) or e['agent_pose_at_ready'] is not None
                or e['old_control_source_request_id'] != (rid-1 if rid > 1 else None)):
            raise ValueError('Invalid native/source contract')
        if rid > 1 and (e['old_controller_command'] != events[-1]['controller_command'] or e['old_wheel_commands'] != events[-1]['wheel_commands']):
            raise ValueError('Incorrect OLD source')
        cmd, yf, idx = waypoint_command(c, yf, **cfg['controller'])
        np.testing.assert_allclose(cmd, e['controller_command'], rtol=0, atol=1e-12)
        np.testing.assert_allclose(yf, e['yaw_filter_state'], rtol=0, atol=1e-12)
        if idx != e['lookahead_index']: raise ValueError('Lookahead changed')
        for key in ['observation','response_detected','application']:
            st = e[key]; row = rows[st['tick']]
            if st['sim_time'] != float(row['sim_time']) or st['pose'] != [float(row[k]) for k in ['x','y','yaw']]:
                raise ValueError('Snapshot not exact physics tick')
        if e['agent_pose_at_observation'] != e['observation']['pose'] or e['agent_pose_at_application'] != e['application']['pose']:
            raise ValueError('Anchor changed')
        if not (path/e['rgb_observation_reference']).is_file(): raise ValueError('Missing RGB')
        clocks = [e[k]['monotonic_ns'] for k in ['observation','request_submit','inference_start','ready','response_detected','application']]
        if clocks != sorted(clocks): raise ValueError('Clock order invalid')
        selected = [r for r in rows if r['pending_request_id'] == str(rid)]
        if [int(r['tick']) for r in selected] != list(range(e['observation']['tick']+1,e['application']['tick']+1)):
            raise ValueError('Missing pending/held tick')
        cmd = e['old_controller_command'] if rid > 1 else [0.,0.]
        wheels = e['old_wheel_commands'] if rid > 1 else [0.,0.]
        for r in selected:
            if r['active_control_source_request_id'] != (str(rid-1) if rid > 1 else ''):
                raise ValueError('OLD source changed before FRESH switch')
            np.testing.assert_allclose([float(r[k]) for k in ['target_v','target_w','applied_v','applied_w','wheel_target_left','wheel_target_right']], [*cmd,*cmd,*wheels], rtol=0, atol=1e-12)
        extra = e['application']['tick']-e['response_detected']['tick']
        if extra != e.get('late_handoff', {}).get('requested_ticks',0): raise ValueError('Unexpected hold duration')
        inside.append(sum(e['inference_start']['monotonic_ns'] <= int(r['physics_step_start_monotonic_ns']) and int(r['monotonic_ns']) <= e['ready']['monotonic_ns'] for r in selected))
        events.append(e); chunks.append(c)
    hold_path = path/'raw/late_handoff/hold.json'
    hold = json.loads(hold_path.read_text()) if hold_path.exists() else None
    if sum('late_handoff' in e for e in events) > 1: raise ValueError('Multiple interventions')
    if hold:
        received = json.loads((hold_path.parent/'received.json').read_text()); c = validate_chunk(np.load(hold_path.parent/'received.npy',allow_pickle=False))
        if hashlib.sha256(c.tobytes()).hexdigest() != hold['native_sha256']: raise ValueError('Held native changed')
        selected = [r for r in rows if hold['start']['tick'] < int(r['tick']) <= hold['end']['tick']]
        if len(selected) != hold['realized_ticks'] or len(selected) != len(hold['hold_ticks']): raise ValueError('Hold coverage incomplete')
        for r in selected:
            if r['pending_request_id'] != str(hold['request_id']) or r['active_control_source_request_id'] != str(hold['old_request_id']): raise ValueError('Held source mismatch')
            np.testing.assert_allclose([float(r[k]) for k in ['target_v','target_w','wheel_target_left','wheel_target_right']], [*hold['old_command'],*hold['old_wheel_targets']],rtol=0,atol=1e-12)
        obs = received['request']['observation']; det = received['response_detected']
        if (received['request']['request_id'] != hold['request_id'] or hold['old_request_id'] != hold['request_id']-1
                or received['agent_pose_at_observation'] != obs['pose'] or hold['start']['tick'] != det['tick']
                or abs(hold['realized_delay_sim_s']-hold['realized_ticks']*cfg['simulation']['dt']) > 1e-6):
            raise ValueError('Held timing/anchor mismatch')
        held_rows = [r for r in rows if r['pending_request_id'] == str(hold['request_id'])]
        if [int(r['tick']) for r in held_rows] != list(range(obs['tick']+1, hold['end']['tick']+1)):
            raise ValueError('Held inference/physics coverage incomplete')
        for r in held_rows:
            if r['active_control_source_request_id'] != str(hold['old_request_id']): raise ValueError('Held OLD source changed')
            np.testing.assert_allclose([float(r[k]) for k in ['target_v','target_w','applied_v','applied_w','wheel_target_left','wheel_target_right']],
                [*hold['old_command'],*hold['old_command'],*hold['old_wheel_targets']],rtol=0,atol=1e-12)
        if not hold['applied'] and (len(events) != hold['old_request_id'] or not hold['contact']):
            raise ValueError('Unsupported incomplete held trial')
        if hold['applied']:
            e = events[hold['request_id']-1]
            if e['application']['tick'] != hold['end']['tick']: raise ValueError('Switch not at hold end')
            np.testing.assert_array_equal(c, chunks[hold['request_id']-1])
            if e['agent_pose_at_observation'] != received['agent_pose_at_observation'] or e['controller_command'] != received['controller_command']: raise ValueError('Held response altered')
    return dict(path=path, metadata=meta, cfg=cfg, robot=rows, events=events, chunks=chunks,
        worlds=[project_world(c,e['agent_pose_at_observation']) for c,e in zip(chunks,events)],
        times=np.array([float(r['sim_time']) for r in rows]), xy=np.array([[float(r['x']),float(r['y'])] for r in rows]),
        contacts=contacts(path), inside_action_ticks=inside, complete=len(events)==48, hold=hold)


def validate_late(path):
    run = load_late(path)
    valid = sum(n > 0 for n in run['inside_action_ticks'][1:]) >= 3
    out = Path(path)/'derived/handoff'; out.mkdir(parents=True)
    result = dict(continuous_handoff_validated=valid, physics_ticks_inside_action_by_request=run['inside_action_ticks'],
        additional_hold_validated=True, transport_decision='OLD retained through actual inference and separately recorded artificial hold')
    write_json(out/'summary.json',result)
    return result
