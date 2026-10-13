"""Six fresh E16 runs, A/B/B/A/A/B, differing only in application_hook selection."""
import argparse
import copy
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance,clocks
from research.hospital_episode import digest
from research.hospital_no_cart_reproduction import validate_reference,stage_audit
from research.hospital_lights_run import verify
from research.nova_evidence import NovaRecords,load_nova
from research.late_handoff import ApplicationHold,eligible,load_late
from research.analyze_no_cart_speed import in_corner
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination

RULES=ROOT/'configs/research/e16_zero_repeatability.json'
ORDER=['NO_HOOK','ZERO_HOOK','ZERO_HOOK','NO_HOOK','NO_HOOK','ZERO_HOOK']
SCOPE='SIX_NEW_E16_ZERO_DELAY_REPEATABILITY_ABBAAB_NO_RETRIES'


def host_state():
    return {key:subprocess.check_output(command,text=True).strip() for key,command in {
        'kernel':['uname','-r'],
        'gpu':['nvidia-smi','--query-gpu=name,uuid,driver_version,memory.total','--format=csv,noheader']}.items()}


def plan(prefix,root=ROOT):
    if not prefix or Path(prefix).name!=prefix:raise ValueError('Simple fresh prefix required')
    trials=[dict(index=i,condition=c,run_id=f'{prefix}-r{i+1:02d}-{c.lower()}') for i,c in enumerate(ORDER)]
    if any((root/'outputs'/t['run_id']).exists() for t in trials):raise FileExistsError('Run ID consumed; no retry')
    return trials


def prior_gate(trials,index,root=ROOT):
    if [t['condition'] for t in trials]!=ORDER or index not in range(6):raise ValueError('Invalid six-run order')
    for t in trials[:index]:
        folder=root/'outputs'/t['run_id'];summary=folder/'summary.json'
        if (folder/'failure.json').exists() or not summary.exists():raise ValueError('STOP: previous runtime incomplete/failed; no skip/retry')
        s=json.loads(summary.read_text())
        if s.get('runtime_status')!='PASS' or s.get('successful_predictions')!=48 or not s.get('repeatability_contract_validated'):
            raise ValueError('STOP: previous runtime/inference contract failed')
    if any((root/'outputs'/t['run_id']).exists() for t in trials[index:]):raise FileExistsError('Exclusive ID/order violated')


def choose_hook(condition,cfg,rules,records):
    if condition not in ORDER:raise ValueError('Unknown condition')
    return None if condition=='NO_HOOK' else ApplicationHold(cfg,rules,0.,records)


def selected_observation(run,rules):
    inside=np.flatnonzero(in_corner(run['xy'],run['cfg']))
    entry=run['times'][inside[0]] if len(inside) else np.inf
    return next((e for e in run['events'][1:] if eligible(e['observation'],run['cfg'],rules['trigger_band_south_m'],entry<=e['observation']['sim_time'])),None)


def audit_saved(path,condition,rules):
    original=load_nova(path);run=load_late(path)
    if len(run['events'])!=48:raise ValueError('Expected 48 finite predictions')
    selected=selected_observation(run,rules);hold=run['hold']
    if any(e['response_detected']['tick']!=e['application']['tick'] for e in run['events']):raise ValueError('Unexpected additional physics')
    if condition=='NO_HOOK':
        if hold or (Path(path)/'raw/late_handoff').exists() or any('late_handoff' in e for e in run['events']):raise ValueError('NO_HOOK has hook evidence')
    elif condition=='ZERO_HOOK':
        if bool(hold)!=bool(selected):raise ValueError('Hook/eligibility mismatch')
        if hold and (hold['request_id']!=selected['request_id'] or hold['requested_delay_s']!=0 or hold['realized_ticks']!=0 or not hold['fresh_unchanged'] or not hold['applied']):raise ValueError('Invalid zero hook')
    else:raise ValueError('Unknown condition')
    first=run['events'][0]
    if first['delay_reference'] is not None or first['sampled_image_request_ids']!=[1] or first['observation']['tick']!=0 or first['reasoning_new_start_step']!=0:
        raise ValueError('Non-fresh bootstrap history')
    if not all(e['kv_cache_available'] and e['reasoning_exact_worker_ready_wall_time_ns'] is None for e in run['events']):raise ValueError('Reasoning contract changed')
    valid=sum(n>0 for n in original['inside_action_ticks'][1:])>=3
    if not valid:raise ValueError('OLD retention/real inference evidence invalid')
    return dict(continuous_handoff_validated=True,repeatability_contract_validated=True,condition=condition,
        selected_request_id=selected['request_id'] if selected else None,triggered=bool(hold),additional_physics_ticks=0,
        fresh_bootstrap_history=True,controller_reconstructed_count=48,
        transport_decision='Original continuous OLD execution; ZERO_HOOK has exactly zero extra physics ticks',
        physics_ticks_inside_action_by_request=original['inside_action_ticks'])


def freeze(references,out,instruction,startup,tests,prefix):
    out=validate_destination(out,references);rules=json.loads(RULES.read_text())
    if rules['conditions']!=ORDER or rules['maximum_new_runs']!=6 or rules['additional_hold_s']!=0:raise ValueError('Wrong protocol')
    for p in [startup,tests]:
        if json.loads(p.read_text()).get('pass') is not True:raise ValueError('Startup/tests gate failed')
    cfg=copy.deepcopy(json.loads((references[0]/'metadata.json').read_text())['config'])
    for path in references:validate_reference(json.loads((path/'metadata.json').read_text())['config'],cfg)
    trials=plan(prefix);hook_path=ROOT/rules['hook_rules_file'];hook_rules=json.loads(hook_path.read_text())
    # Saved no-hook and zero-hook evidence establish contracts, not new trial counts.
    pre=[audit_saved(p,'NO_HOOK' if i<2 else 'ZERO_HOOK',hook_rules) for i,p in enumerate(references)]
    runtime=json.loads((references[0]/'inference-runtime.json').read_text());weights=[Path(cfg['inference']['base_model'])/'model.safetensors',Path(cfg['inference']['checkpoint'])]
    if [digest(p) for p in weights]!=[runtime['artifact_sha256']['base'],runtime['artifact_sha256']['checkpoint']]:raise ValueError('Weights changed')
    out.mkdir(parents=True);write_json(out/'config.json',cfg)
    # This receipt freezes runtime dependencies; future saved-only analysis may be additive.
    code={p:h for p,h in evidence_hashes([ROOT/'research',ROOT/'DynaNav',ROOT/'scripts/isaac6_python.sh']).items() if Path(p).suffix in ['.py','.sh']}
    receipt=dict(scope=SCOPE,trials=trials,rules=rules,hook_rules=hook_rules,reference_runs=list(map(str,references)),
        config_sha256=digest(out/'config.json'),config_difference=[],mode='baseline',lighting_visual_pass=True,
        host=host_state(),code_sha256=code,source_sha256=evidence_hashes([*references,*weights,instruction,startup,tests,RULES,hook_path,ROOT/'tests/test_e16_zero_repeatability.py']),
        source_contract_checks=pre,user_instruction=instruction.read_text(),**{k:v for k,v in provenance(ROOT).items() if k!='source_sha256'})
    write_json(out/'receipt.json',receipt)


def execute(folder,index):
    receipt=json.loads((folder/'receipt.json').read_text());cfg=json.loads((folder/'config.json').read_text())
    if receipt['scope']!=SCOPE:raise ValueError('Wrong receipt')
    prior_gate(receipt['trials'],index);trial=receipt['trials'][index];reference=Path(receipt['reference_runs'][0])
    validate_reference(json.loads((reference/'metadata.json').read_text())['config'],cfg)
    per_trial={**receipt,'authorized_run_id':trial['run_id'],'trial':trial};verify(cfg,folder/'config.json',per_trial,trial['run_id'],'baseline')
    records=NovaRecords(ROOT/'outputs'/trial['run_id'],dict(mode='baseline',config=cfg,command_variant='direct',
        repeatability_trial=trial,freeze_receipt=per_trial,freeze_receipt_sha256=digest(folder/'receipt.json'),command_argv=sys.argv,**provenance(ROOT)))
    app=None;sim=None;code=1
    try:
        host=host_state();write_json(records.path/'host.json',dict(**host,simulator_process_id=os.getpid(),**clocks()))
        if host!=receipt['host']:raise ValueError('Host GPU/driver/kernel changed')
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RealTimePathTracing','width':1920,'height':1080})
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        sim=HospitalLightsSimulation(cfg,records,actor_factory=None);sim.phase='hospital_lights_baseline'
        write_json(records.path/'no_cart_pre_inference.json',stage_audit(sim,reference))
        control=Path(inspect.getfile(DifferentialController))
        write_json(records.path/'simulation-runtime.json',dict(python=sys.version,numpy=np.__version__,
            isaac_version=(Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),controller_sha256=digest(control)))
        hook=choose_hook(trial['condition'],cfg,receipt['hook_rules'],records)
        def validate(path):
            result=audit_saved(path,trial['condition'],receipt['hook_rules'])
            result['child_process_snapshot_after_navigation']=subprocess.check_output(['ps','--ppid',str(os.getpid()),'-o','pid,ppid,args'],text=True)
            (path/'derived/handoff').mkdir(parents=True);write_json(path/'derived/handoff/summary.json',result)
            return result
        result=run_model_loop(sim,cfg,records,'static',evidence_analyzer=validate,application_hook=hook)
        write_json(records.path/'no_cart_post_run.json',stage_audit(sim,reference))
        write_json(records.path/'summary.json',dict(status='PASS',runtime_status='PASS',repeatability_contract_validated=True,
            condition=trial['condition'],selected_hook_request_id=hook.selected if hook else None,**result))
        print('ZERO_REPEATABILITY_COMPLETE',trial,flush=True);code=0
    except Exception as exc:
        if sim:sim.apply(0.,0.)
        traceback.print_exc();write_json(records.path/'failure.json',dict(status='FAIL',runtime_status='FAIL',error=repr(exc),**clocks()))
    finally:
        if sim:sim.close()
        if app:app.close(exit_code=code)
    if code:raise SystemExit(code)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['freeze','run'],required=True);p.add_argument('--receipt-dir',type=Path,required=True)
    p.add_argument('--index',type=int);p.add_argument('--reference-runs',type=Path,nargs=4);p.add_argument('--run-prefix')
    for k in ['instruction-file','startup-result','tests-result']:p.add_argument('--'+k,type=Path)
    a=p.parse_args()
    if a.phase=='freeze':freeze([v.resolve() for v in a.reference_runs],a.receipt_dir.resolve(),a.instruction_file.resolve(),a.startup_result.resolve(),a.tests_result.resolve(),a.run_prefix)
    else:execute(a.receipt_dir.resolve(),a.index)
