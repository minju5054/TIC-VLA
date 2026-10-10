"""Receipt-gated Hospital lights-on baseline / stationary human run."""
import argparse,json,os,sys,traceback
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json,provenance,clocks
from research.hospital_episode import digest,validate_config
from research.nova_evidence import NovaRecords,validate_continuous
from research.hospital_spatial_gate import require_human_prerequisites


def verify(cfg,path,receipt,run_id,mode):
    if receipt['authorized_run_id']!=run_id or receipt['config_sha256']!=digest(path) or receipt['mode']!=mode:raise ValueError('Run authorization mismatch')
    if receipt.get('lighting_visual_pass') is not True:raise ValueError('Lighting calibration not reviewed')
    if mode=='stationary':require_human_prerequisites(receipt)
    for group in ['code_sha256','source_sha256']:
        for p,h in receipt[group].items():
            if digest(ROOT/p)!=h:raise ValueError('Frozen file changed: '+p)
    if cfg['simulation']['predictions']!=48 or 'bright_profile' in cfg:raise ValueError('Fixed v2 baseline required; v1 lights forbidden')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['config','freeze-receipt','run-id']:p.add_argument('--'+k,required=True)
    p.add_argument('--mode',choices=['baseline','stationary'],required=True);a=p.parse_args()
    if Path(a.run_id).name!=a.run_id:raise ValueError('Fresh simple run ID required')
    cfg=yaml.safe_load(Path(a.config).read_text());validate_config(cfg);receipt=json.loads(Path(a.freeze_receipt).read_text());verify(cfg,a.config,receipt,a.run_id,a.mode)
    records=NovaRecords(ROOT/'outputs'/a.run_id,{'mode':a.mode,'config':cfg,'command_variant':'direct','command_argv':sys.argv,
        'freeze_receipt':receipt,'freeze_receipt_sha256':digest(a.freeze_receipt),'config_sha256':digest(a.config),
        'all_research_code_sha256':{str(p.relative_to(ROOT)):digest(p) for p in (ROOT/'research').rglob('*.py')},**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});sim=None;code=1
    try:
        import inspect
        from research.hospital_lights_simulation import HospitalLightsSimulation
        from research.continuous import run_model_loop
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        factory=None
        if a.mode=='stationary':
            from research.stationary_human import StationaryHuman
            factory=lambda s:StationaryHuman(s,cfg['stationary_human'])
        sim=HospitalLightsSimulation(cfg,records,actor_factory=factory);sim.phase='hospital_lights_'+a.mode
        control=Path(inspect.getfile(DifferentialController))
        write_json(records.path/'simulation-runtime.json',{'python':sys.version,'numpy':np.__version__,
            'isaac_version':(Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),'differential_controller_source':str(control),'controller_sha256':digest(control)})
        result=run_model_loop(sim,cfg,records,'static',evidence_analyzer=validate_continuous)
        write_json(records.path/'summary.json',{'status':'PASS','mode':a.mode,**result});print('HOSPITAL_LIGHTS_RUN_COMPLETE',json.dumps(result),flush=True);code=0
    except Exception as exc:traceback.print_exc();write_json(records.path/'failure.json',{'status':'FAIL','error':repr(exc),**clocks()})
    finally:
        if sim:sim.close()
        app.close(exit_code=code)


if __name__=='__main__':main()
