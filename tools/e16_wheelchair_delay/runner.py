"""Bounded independent E16 processes. Existing no-hook navigation is unchanged."""
import argparse
from datetime import datetime,timezone
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.e16_wheelchair_delay.common import read,write,digest,snapshot,verify_snapshot,PLAN


def freeze_runtime(suite):
    reference=ROOT/'outputs'/read(suite/'protocol.json')['reference_run']
    verify_snapshot(reference,read(suite/'reference-before.json'))
    old=read(reference/'metadata.json')
    sources={rel:digest(ROOT/rel) for rel in old['source_sha256']}
    if sources!=old['source_sha256']:raise ValueError('Reference runtime changed')
    sources.update({str(p.relative_to(ROOT)):digest(p) for p in (ROOT/'tools/e16_wheelchair_delay').glob('*.py')})
    runtime=read(suite/'runtime-reference.json')
    for filename,h in runtime['external_sha256'].items():
        if digest(Path(filename))!=h:raise ValueError('Existing binary/model changed: '+filename)
    git=lambda *a:subprocess.check_output(['git',*a],cwd=ROOT,text=True).strip()
    write(suite/'runtime-freeze.json',dict(source_sha256=sources,protocol_sha256=digest(suite/'protocol.json'),
        config_sha256=digest(suite/'config.json'),local_sha=git('rev-parse','HEAD'),
        upstream_sha=git('rev-parse','upstream/main'),dirty_status=git('status','--short'),
        diff_sha256=__import__('hashlib').sha256(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT)).hexdigest(),
        command_argv=sys.argv,external_hashes_verified=len(runtime['external_sha256'])))
    print('RUNTIME_FREEZE_PASS',flush=True)


def verify(suite,index):
    if not 1<=index<=6:raise ValueError('At most six prespecified trials')
    protocol=read(suite/'protocol.json');frozen=read(suite/'runtime-freeze.json');cfg=read(suite/'config.json')
    if subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip()!='TIC-VLA-TEST':raise ValueError('Wrong branch')
    for p,h in frozen['source_sha256'].items():
        if digest(ROOT/p)!=h:raise ValueError('Frozen runtime source changed: '+p)
    if digest(suite/'protocol.json')!=frozen['protocol_sha256'] or digest(suite/'config.json')!=frozen['config_sha256']:
        raise ValueError('Protocol/config changed')
    reference=ROOT/'outputs'/protocol['reference_run']
    if cfg!=read(reference/'metadata.json')['config']:raise ValueError('Navigation settings differ')
    verify_snapshot(reference,read(suite/'reference-before.json'))
    if index>1:
        if not (suite/f'trial-{index-1}/process-result.json').exists() or not (suite/f'trial-{index-1}/analysis/result.json').exists():
            raise ValueError('Previous trial must finish and be analyzed before this trial')
    if index>=5:
        gate=read(suite/'stage-b-gate.json')
        if gate['status']!='BASELINE_STABLE':raise ValueError('Positive delay blocked by baseline gate')
        for rel,h in gate['result_hashes'].items():
            if digest(suite/rel)!=h:raise ValueError('Baseline evidence changed')
        if index==6 and read(suite/'trial-5/analysis/result.json').get('safety_stop'):
            raise ValueError('Stop remaining positive trials after safety stop')
    startup=read(suite/'startup-result.json')
    if not startup['pass']:raise ValueError('Simulator-only startup did not pass')
    return protocol,cfg,frozen


def run(suite,index):
    protocol,cfg,frozen=verify(suite,index);runtime=read(suite/'runtime-reference.json')
    setup=suite/f'trial-{index}';mode,seconds=PLAN[index-1]
    destination=ROOT/'outputs'/f'{suite.name}-run-{index}-{mode.lower()}'
    if destination.exists():raise FileExistsError('Refuse prior run')
    from research.nova_evidence import NovaRecords
    from research.records import clocks
    write(setup/'execution-start.json',dict(pid=os.getpid(),**clocks(),argv=sys.argv))
    records=NovaRecords(destination,dict(config=cfg,mode='static',scenario='E16 C10/C24 original stationary wheelchairs',
        command_variant='direct',scope='PRESPECIFIED_E16_POST_RESPONSE_APPLICATION_HOLD',
        trial_index=index,trial_mode=mode,extra_hold_s=seconds,protocol_relative=str((suite/'protocol.json').relative_to(ROOT)),
        local_sha=frozen['local_sha'],upstream_sha=frozen['upstream_sha'],git_status=frozen['dirty_status'],
        diff_sha256=frozen['diff_sha256'],source_sha256=frozen['source_sha256'],command_argv=sys.argv))
    app=sim=hook=None;code=1
    try:
        from isaacsim import SimulationApp
        app=SimulationApp(runtime['launch']);print('DELAY_SIMULATION_APP_READY',flush=True)
        import numpy as np
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        controller=Path(inspect.getfile(DifferentialController))
        actual=dict(python=sys.version,numpy=np.__version__,isaac_version=(Path(os.environ['ISAAC_PATH'])/'VERSION').read_text().strip(),
                    differential_controller_source=str(controller),controller_sha256=digest(controller),renderer=runtime['launch']['renderer'])
        write(destination/'simulation-runtime.json',actual)
        if actual['isaac_version']!=runtime['isaac6_version'] or actual['controller_sha256']!=runtime['controller_sha256']:
            raise ValueError('Simulator/controller identity mismatch')
        from tools.two_wheelchairs_v08.scene import WheelchairSimulation,WheelchairActor,pre_model_gate
        sim=WheelchairSimulation(cfg,records,actor_factory=lambda s:WheelchairActor(s,cfg))
        sim.phase='two_wheelchairs_v08'
        pre_model_gate(sim,cfg,runtime,ROOT/'outputs/nova-e16-hospital-lights-baseline-20261010-01')
        print('DELAY_PRE_MODEL_PASS',flush=True)
        # No-hook deliberately passes None through the unchanged original loop.
        from tools.e16_wheelchair_delay.hook import C10ApplicationHold,DiagnosticStop
        from tools.e16_wheelchair_delay.evidence import validate_continuous
        from research.continuous import run_model_loop
        if mode!='NO_HOOK':hook=C10ApplicationHold(cfg,protocol,seconds,records)
        try:
            result=run_model_loop(sim,cfg,records,'static',evidence_analyzer=validate_continuous,application_hook=hook)
            write(destination/'summary.json',dict(status='PASS',scope='Runtime only; navigation assessed separately',**result))
            code=0
        except DiagnosticStop as exc:
            write(destination/'summary.json',dict(status='SAFETY_STOP',reason=exc.status,applied_predictions=len(records.ids),
                  selected_fresh_not_applied=True,**clocks()))
            code=0
    except Exception as exc:
        traceback.print_exc();write(destination/'failure.json',dict(status='IMPLEMENTATION_FAILURE',error=repr(exc),**clocks()))
    finally:
        if hook:hook.finalize()
        if sim:sim.close()
        protected={str(p.relative_to(destination)):digest(p) for p in destination.rglob('*') if p.is_file() and 'derived' not in p.relative_to(destination).parts}
        write(destination/'raw-sha256.json',protected)
        if app:app.close(exit_code=code)
    return code


def launch(suite,index):
    protocol,cfg,frozen=verify(suite,index)
    setup=suite/f'trial-{index}';setup.mkdir(exist_ok=False)
    mode,seconds=PLAN[index-1];runtime=read(suite/'runtime-reference.json')
    destination=ROOT/'outputs'/f'{suite.name}-run-{index}-{mode.lower()}'
    write(setup/'attempt.json',dict(index=index,mode=mode,delay_s=seconds,run_directory=str(destination),
                                  utc=datetime.now(timezone.utc).isoformat()))
    cmd=['timeout','--signal=INT','--kill-after=30s','900s','bash',str(ROOT/'scripts/isaac6_python.sh'),
         str(Path(__file__)), 'run','--suite',str(suite),'--index',str(index)]
    env=os.environ.copy();env.update(TICVLA_ISAAC6_ROOT=runtime['isaac6_root'],CUDA_VISIBLE_DEVICES='0',PYTHONDONTWRITEBYTECODE='1')
    with (setup/'console.log').open('xb') as log:
        process=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
        print('TRIAL_LAUNCHED',index,mode,'pid',process.pid,flush=True)
        rc=process.wait()
    summary=read(destination/'summary.json') if (destination/'summary.json').exists() else None
    write(setup/'process-result.json',dict(argv=cmd,returncode=rc,simulator_launcher_pid=process.pid,
        summary=summary,run_directory=str(destination),process_fresh=True))
    print('TRIAL_FINISHED',index,'rc',rc,'summary',summary,flush=True)
    return rc


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['freeze','launch','run'])
    p.add_argument('--suite',type=Path,required=True);p.add_argument('--index',type=int)
    a=p.parse_args();s=a.suite.resolve()
    if a.phase=='freeze':freeze_runtime(s)
    else:raise SystemExit({'launch':launch,'run':run}[a.phase](s,a.index))
