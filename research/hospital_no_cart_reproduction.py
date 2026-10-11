"""One exact-config E16 no-cart max_v=1.5 reproduction, 48 predictions."""
import argparse
import copy
import inspect
import json
import os
from pathlib import Path
import sys
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.e16_c26_speed_ablation import config_diff
from research.hospital_episode import digest,validate_config
from research.hospital_lights_run import verify
from research.nova_evidence import NovaRecords,validate_continuous
from research.records import write_json,provenance,clocks
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination

DIFF=[]
SCOPE='ONE_E16_NO_CART_1_5_EXACT_CONFIG_REPRODUCTION_48_PREDICTIONS'


def validate_reference(old,new):
    forbidden={'stationary_cart','stationary_human','dynamic_human','pedestrian','obstacles'}
    if forbidden.intersection(old) or forbidden.intersection(new) or 'obstacles' in new['scene']:
        raise ValueError('No custom actor or obstacle config permitted')
    if config_diff(old,new)!=DIFF or new['simulation']['predictions']!=48 or new['controller']['max_v']!=1.5:
        raise ValueError('Exact original no-cart1.5 config and 48 predictions required')
    validate_config(new)


def freeze(reference,out,run_id,instruction,startup):
    if not run_id or Path(run_id).name!=run_id:raise ValueError('Simple fresh run ID required')
    validate_destination(out,[reference])
    if (ROOT/'outputs'/run_id).exists():raise FileExistsError('Run ID consumed; no retry')
    if json.loads(startup.read_text()).get('pass') is not True:raise ValueError('Isaac 6 startup gate failed')
    meta=json.loads((reference/'metadata.json').read_text())
    if json.loads((reference/'summary.json').read_text())['status']!='PASS' or meta['freeze_receipt']['lighting_visual_pass'] is not True:
        raise ValueError('Completed validated lights-on baseline required')
    cfg=copy.deepcopy(meta['config']);validate_reference(meta['config'],cfg)
    runtime=json.loads((reference/'inference-runtime.json').read_text())
    weights=[Path(cfg['inference']['base_model'])/'model.safetensors',Path(cfg['inference']['checkpoint'])]
    if [digest(p) for p in weights]!=[runtime['artifact_sha256']['base'],runtime['artifact_sha256']['checkpoint']]:raise ValueError('Weights changed')
    out.mkdir(parents=True);write_json(out/'config.json',cfg)
    code={p:h for p,h in evidence_hashes([ROOT/'research',ROOT/'DynaNav',ROOT/'scripts/isaac6_python.sh']).items() if Path(p).suffix in ['.py','.sh']}
    receipt=dict(authorized_run_id=run_id,mode='baseline',
        config_sha256=digest(out/'config.json'),lighting_visual_pass=True,
        scope=SCOPE,reference_run=str(reference),exact_config_difference=DIFF,predictions=48,
        user_instruction=instruction.read_text(),single_run_no_retry=True,
        code_sha256=code,source_sha256=evidence_hashes([reference,*weights,instruction,startup,
            ROOT/'configs/research/e16_no_cart_15_reproduction_analysis.json',
            ROOT/'configs/research/e16_no_cart_speed10_analysis.json']))
    write_json(out/'receipt.json',{**provenance(ROOT),**receipt})


from research.hospital_no_cart_speed import stage_audit


def execute(folder):
    receipt=json.loads((folder/'receipt.json').read_text());cfg=json.loads((folder/'config.json').read_text())
    reference=Path(receipt['reference_run'])
    if receipt['scope']!=SCOPE or receipt['predictions']!=48 or not receipt['single_run_no_retry']:raise ValueError('Wrong scope')
    validate_reference(json.loads((reference/'metadata.json').read_text())['config'],cfg)
    verify(cfg,folder/'config.json',receipt,receipt['authorized_run_id'],'baseline')
    records=NovaRecords(ROOT/'outputs'/receipt['authorized_run_id'],dict(mode='baseline',config=cfg,
        command_variant='direct',freeze_receipt=receipt,command_argv=sys.argv,**provenance(ROOT)))
    from isaacsim import SimulationApp
    app=None;sim=None;code=1
    try:
        app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080})
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        sim=HospitalLightsSimulation(cfg,records,actor_factory=None);sim.phase='hospital_lights_baseline'
        write_json(records.path/'no_cart_pre_inference.json',stage_audit(sim,reference))
        control=Path(inspect.getfile(DifferentialController))
        write_json(records.path/'simulation-runtime.json',dict(python=sys.version,numpy=np.__version__,
            isaac_version=(Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),
            differential_controller_source=str(control),controller_sha256=digest(control)))
        result=run_model_loop(sim,cfg,records,'static',evidence_analyzer=validate_continuous)
        write_json(records.path/'no_cart_post_run.json',stage_audit(sim,reference))
        write_json(records.path/'summary.json',dict(status='PASS',mode='baseline',**result))
        print('NO_CART_15_REPRODUCTION_COMPLETE',json.dumps(result),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(records.path/'failure.json',dict(status='FAIL',error=repr(exc),**clocks()))
    finally:
        if sim:sim.close()
        if app:app.close(exit_code=code)
    if code:raise SystemExit(code)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['freeze','run'],required=True)
    p.add_argument('--receipt-dir',type=Path,required=True)
    for k in ['reference-run','instruction-file','startup-result']:p.add_argument('--'+k,type=Path)
    p.add_argument('--run-id');a=p.parse_args()
    if a.phase=='freeze':freeze(a.reference_run.resolve(),a.receipt_dir.resolve(),a.run_id,a.instruction_file.resolve(),a.startup_result.resolve())
    else:execute(a.receipt_dir.resolve())
