"""Frozen E16 post-response hold sweep; each invocation consumes one trial only."""
import argparse
import copy
import inspect
import json
import os
from pathlib import Path
import sys
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json, provenance, clocks
from research.hospital_episode import digest
from research.hospital_no_cart_reproduction import validate_reference, stage_audit
from research.hospital_lights_run import verify
from research.nova_evidence import NovaRecords
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination
from research.analyze_nova_dynamic_handoff import load_saved
from research.analyze_no_cart_speed import analyze_run, RULES as NAV_RULES, INVENTORY, in_corner
from research.late_handoff import eligible, delay_ticks, ApplicationHold, HoldContactStop, validate_late

RULES = ROOT/'configs/research/e16_late_handoff_stress.json'
SCOPE = 'E16_ONE_POST_RESPONSE_HOLD_PER_TRIAL_MAX10_NO_RETRIES'


def preflight(references, rules):
    results = []; cfg = None
    for path in references:
        run = load_saved(path)
        validate_reference(cfg or run['cfg'], run['cfg']); cfg = run['cfg']
        nav = analyze_run(run, json.loads(NAV_RULES.read_text()), json.loads(INVENTORY.read_text()))['summary']
        if not nav['left_turn_completed'] or not nav['goal_region_reached'] or run['contacts']:
            raise ValueError('Reference not contact-free successful baseline')
        entry = nav['corner_entry']['sim_time']; candidates = []
        for e, c in zip(run['events'][1:], run['chunks'][1:]):
            st = e['observation']
            if eligible(st,cfg,rules['candidate_band_south_m'],entry <= st['sim_time']):
                idx = e['lookahead_index']; old = e['old_controller_command']; fresh = e['controller_command']
                candidates.append(dict(request_id=e['request_id'],observation=st,old_command=old,
                    fresh_command=fresh,lookahead_index=idx,native_lookahead=c[idx].tolist(),
                    south_distance_to_entry_y_m=cfg['spatial_turn_gate']['center_y_min_m']-st['pose'][1],
                    east_distance_to_exit_x_m=st['pose'][0]-cfg['spatial_turn_gate']['exit_center_x_max_m'],
                    delta_w=fresh[1]-old[1],both_left=old[1]>0 and fresh[1]>0,
                    original_pending_sim_s=e['application']['sim_time']-st['sim_time'],
                    remaining_horizon_by_hold_s={str(d):3-(e['application']['sim_time']-st['sim_time'])-d for d in rules['delays_s']},
                    frozen_eligible=eligible(st,cfg,rules['trigger_band_south_m'],entry <= st['sim_time'])))
        selected = next((c for c in candidates if c['frozen_eligible']),None)
        if selected is None or not selected['both_left'] or selected['delta_w'] <= 1e-6 or min(selected['remaining_horizon_by_hold_s'].values()) <= 0:
            raise ValueError('No meaningful common pre-corner stronger-left handoff')
        results.append(dict(source_run=str(path),corner_entry_sim_time=entry,candidates=candidates,selected=selected))
    return dict(pass_saved_preflight=True,references=results,rule=rules['trigger_rule'],
        refinement='Inner half of initial 1.5m band selects stronger left command in both saved references. Both OLD and FRESH already turn left; no differing route choice established. Runtime eligibility uses only measured spatial progress, not prediction content.')


def next_allowed(trials, completed):
    """Prespecified stopping gate. completed contains ordered per-trial metrics."""
    if len(completed) >= len(trials): return False, 'Prespecified cap reached'
    if any(m['outcome'] == 'F' for m in completed): return False, 'Runtime/inference failure; no automatic retry'
    if len(completed) >= 2 and not all(m['baseline_gate_pass'] for m in completed[:2]):
        return False, 'Zero-delay controls not reproducibly successful; stop sweep'
    for i in range(2,len(completed)-1,2):
        pair = completed[i:i+2]; lower = completed[i-2:i]
        if all(m['outcome'] in ['C','D'] for m in pair) and all(m['baseline_gate_pass'] for m in lower):
            return False, 'Both corner failures after preceding successful level; stop larger holds'
    return True, 'Next prespecified independent trial allowed'


def freeze(references, out, instruction, startup, tests, prefix):
    out = validate_destination(out,references)
    if Path(prefix).name != prefix: raise ValueError('Simple run prefix required')
    if json.loads(startup.read_text()).get('pass') is not True: raise ValueError('Startup gate failed')
    if json.loads(tests.read_text()).get('pass') is not True: raise ValueError('Pre-model tests failed')
    rules = json.loads(RULES.read_text()); pre = preflight(references,rules)
    cfg = copy.deepcopy(json.loads((references[0]/'metadata.json').read_text())['config'])
    validate_reference(json.loads((references[1]/'metadata.json').read_text())['config'],cfg)
    trials = [dict(index=i*2+j,delay_s=d,trial=j+1,ticks=delay_ticks(d,cfg['simulation']['dt']),
        run_id=f'{prefix}-d{round(d*100):03d}-r{j+1}') for i,d in enumerate(rules['delays_s']) for j in range(2)]
    if len(trials)!=10 or any((ROOT/'outputs'/t['run_id']).exists() for t in trials): raise ValueError('Existing trial or wrong cap')
    runtime = json.loads((references[0]/'inference-runtime.json').read_text())
    weights = [Path(cfg['inference']['base_model'])/'model.safetensors',Path(cfg['inference']['checkpoint'])]
    if [digest(p) for p in weights] != [runtime['artifact_sha256']['base'],runtime['artifact_sha256']['checkpoint']]: raise ValueError('Weights changed')
    out.mkdir(parents=True); write_json(out/'config.json',cfg); write_json(out/'preflight.json',pre)
    code = {p:h for p,h in evidence_hashes([ROOT/'research',ROOT/'DynaNav',ROOT/'scripts/isaac6_python.sh']).items() if Path(p).suffix in ['.py','.sh']}
    receipt = dict(scope=SCOPE,trials=trials,rules=rules,reference_runs=list(map(str,references)),
        mode='baseline',lighting_visual_pass=True,config_sha256=digest(out/'config.json'),
        exact_baseline_config_difference=[],user_instruction=instruction.read_text(),code_sha256=code,
        source_sha256=evidence_hashes([*references,*weights,instruction,startup,tests,RULES,NAV_RULES,INVENTORY,out/'preflight.json']),
        **{k:v for k,v in provenance(ROOT).items() if k!='source_sha256'})
    write_json(out/'receipt.json',receipt)


def execute(folder, index):
    receipt = json.loads((folder/'receipt.json').read_text()); cfg = json.loads((folder/'config.json').read_text())
    if receipt['scope'] != SCOPE or len(receipt['trials']) != 10 or index not in range(10): raise ValueError('Invalid plan')
    trials = receipt['trials']; trial = trials[index]; completed = []
    for previous in trials[:index]:
        path = ROOT/'outputs'/previous['run_id']/'derived/late_metrics_v2.json'
        if not path.is_file(): raise ValueError('Previous trial absent/unreviewed; no skipping')
        completed.append(json.loads(path.read_text())['metrics'])
    allowed, reason = next_allowed(trials,completed)
    if not allowed: raise ValueError(reason)
    if any((ROOT/'outputs'/t['run_id']).exists() for t in trials[index:]): raise ValueError('Trial ID consumed/out of order; no retry')
    reference = Path(receipt['reference_runs'][0]); validate_reference(json.loads((reference/'metadata.json').read_text())['config'],cfg)
    per_trial = {**receipt, 'authorized_run_id':trial['run_id'],'trial':trial}
    verify(cfg,folder/'config.json',per_trial,trial['run_id'],'baseline')
    records = NovaRecords(ROOT/'outputs'/trial['run_id'],dict(mode='baseline',config=cfg,command_variant='direct',
        freeze_receipt=per_trial,freeze_receipt_sha256=digest(folder/'receipt.json'),command_argv=sys.argv,**provenance(ROOT)))
    write_json(records.path/'authorization.json',dict(plan_sha256=digest(folder/'receipt.json'),trial=trial,
        previous_results=completed,decision=reason,**clocks()))
    app = None; sim = None; code = 1
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless':True,'renderer':'RealTimePathTracing','width':1920,'height':1080})
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        sim = HospitalLightsSimulation(cfg,records,actor_factory=None);sim.phase='hospital_lights_baseline'
        write_json(records.path/'no_cart_pre_inference.json',stage_audit(sim,reference))
        control = Path(inspect.getfile(DifferentialController))
        write_json(records.path/'simulation-runtime.json',dict(python=sys.version,numpy=np.__version__,
            isaac_version=(Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),
            differential_controller_source=str(control),controller_sha256=digest(control)))
        hook = ApplicationHold(cfg,receipt['rules'],trial['delay_s'],records)
        try:
            result = run_model_loop(sim,cfg,records,'static',evidence_analyzer=validate_late,application_hook=hook)
            status = 'PASS'
        except HoldContactStop as exc:
            sim.apply(0.,0.);sim.state_file.flush();sim.hospital_diagnostics.file.flush()
            result = dict(successful_predictions=len(records.ids),held_response_applied=False,
                stop_reason=str(exc),**validate_late(records.path))
            status = 'DIAGNOSTIC_CONTACT_STOP'
        write_json(records.path/'no_cart_post_run.json',stage_audit(sim,reference))
        write_json(records.path/'summary.json',dict(status=status,runtime_status='PASS',trigger_request_id=hook.selected,**result))
        print('LATE_HANDOFF_TRIAL_COMPLETE',status,hook.selected,flush=True);code=0
    except Exception as exc:
        if sim: sim.apply(0.,0.)
        traceback.print_exc();write_json(records.path/'failure.json',dict(status='FAIL',runtime_status='FAIL',error=repr(exc),**clocks()))
    finally:
        if sim: sim.close()
        if app: app.close(exit_code=code)
    if code: raise SystemExit(code)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['freeze','run'],required=True)
    p.add_argument('--receipt-dir',type=Path,required=True);p.add_argument('--index',type=int)
    p.add_argument('--reference-runs',type=Path,nargs=2)
    for k in ['instruction-file','startup-result','tests-result']:p.add_argument('--'+k,type=Path)
    p.add_argument('--run-prefix');a=p.parse_args()
    if a.phase=='freeze':freeze([s.resolve() for s in a.reference_runs],a.receipt_dir.resolve(),a.instruction_file.resolve(),a.startup_result.resolve(),a.tests_result.resolve(),a.run_prefix)
    else:execute(a.receipt_dir.resolve(),a.index)
