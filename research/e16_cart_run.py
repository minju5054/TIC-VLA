"""Exactly one receipt-gated E16 static-cart run, original continuous controller."""
import argparse,json,sys,traceback,os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance,clocks
from research.reveal_window import verify_hashes,evidence_hashes
from research.hospital_episode import validate_config,digest
from research.nova_evidence import NovaRecords,validate_continuous


def freeze(preflight,out,run_id):
    if out.exists() or (ROOT/'outputs'/run_id).exists():raise FileExistsError('Fresh receipt/run required')
    physical=json.loads((preflight/'physical.json').read_text());summary=json.loads((preflight/'summary.json').read_text())
    if not physical['pass'] or not summary['model_run_permitted']:raise ValueError('Physical gate not passed')
    cfg=json.loads((ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01/metadata.json').read_text())['config']
    frozen=json.loads((preflight/'freeze.json').read_text());cfg['stationary_cart']=frozen
    validate_config(cfg);out.mkdir(parents=True)
    write_json(out/'config.json',cfg)
    code=evidence_hashes([ROOT/'research',ROOT/'scripts/isaac6_python.sh',ROOT/'DynaNav/ticvla.py',ROOT/'DynaNav/ticvla_vlm.py'])
    code={p:h for p,h in code.items() if Path(p).suffix in ('.py','.sh')}
    write_json(out/'receipt.json',{'authorized_run_id':run_id,'config_sha256':digest(out/'config.json'),
        'physical_pass':True,'visibility_diagnostic_only':True,'code_sha256':code,
        'source_evidence_sha256':evidence_hashes([preflight]),'single_run_no_retry':True,**provenance(ROOT)})


def execute(folder):
    receipt=json.loads((folder/'receipt.json').read_text());cfg=json.loads((folder/'config.json').read_text())
    if digest(folder/'config.json')!=receipt['config_sha256']:raise ValueError('Config changed')
    for group in ['code_sha256','source_evidence_sha256']:verify_hashes(receipt[group])
    verify_hashes(cfg['stationary_cart']['source_evidence_sha256']);validate_config(cfg)
    # NovaRecords exclusively creates this directory before simulator or model startup.
    records=NovaRecords(ROOT/'outputs'/receipt['authorized_run_id'],{'mode':'stationary_cart','config':cfg,
        'command_variant':'direct','freeze_receipt':receipt,'command_argv':sys.argv,
        'actor_metadata_note':'Legacy human_at_* keys carry object_type=stationary_hospital_cart; cart_state.csv is the cart log',**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});sim=None;code=1
    try:
        import omni.client
        from research.hospital_cart import StationaryCart
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.continuous import run_model_loop
        import hashlib
        for url,h in cfg['stationary_cart']['asset_sha256'].items():
            result,_,data=omni.client.read_file(url)
            if result!=omni.client.Result.OK or hashlib.sha256(bytes(data)).hexdigest()!=h:raise ValueError('Frozen asset bytes changed')
        sim=HospitalLightsSimulation(cfg,records,actor_factory=lambda s:StationaryCart(s,cfg['stationary_cart']));sim.phase='hospital_lights_stationary_cart'
        write_json(records.path/'simulation-runtime.json',{'python':sys.version,'numpy':np.__version__,
            'isaac_version':(Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip()})
        result=run_model_loop(sim,cfg,records,'static',evidence_analyzer=validate_continuous)
        write_json(records.path/'summary.json',{'status':'PASS','mode':'stationary_cart',**result});print('E16_CART_RUN_COMPLETE',json.dumps(result),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(records.path/'failure.json',{'status':'FAIL','error':repr(exc),**clocks()})
    finally:
        if sim:sim.close()
        app.close(exit_code=code)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['freeze','run'],required=True)
    p.add_argument('--receipt-dir',required=True);p.add_argument('--preflight-dir');p.add_argument('--run-id');a=p.parse_args()
    if a.phase=='freeze':freeze(Path(a.preflight_dir).resolve(),Path(a.receipt_dir).resolve(),a.run_id)
    else:execute(Path(a.receipt_dir).resolve())
