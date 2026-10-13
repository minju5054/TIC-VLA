"""Isaac 6 semantic-label API adapter for the unchanged frozen render procedure.

The first rendering attempt failed before any candidate capture: Isaac 6 removed
add_update_semantics. Its class-label operation is provided by add_labels, as
already used by research/render_hospital_occlusion.py. This process-local alias
changes no USD geometry, camera, thresholds, candidate, controller, or vendor file.
"""
import argparse
from pathlib import Path
import sys
import traceback
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.two_hz_preflight import read,digest,PRIMARY
from research.two_hz_preflight_isaac import check_frozen,render_candidates


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir)
    check_frozen(out)
    amendment=read(out/'stage-b/render-api-adapter.json')
    if digest(__file__)!=amendment['adapter_sha256']:raise ValueError('Adapter source changed')
    cfg=read(Path(PRIMARY)/'runtime-config.json')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        from isaacsim.core.utils import semantics
        if hasattr(semantics,'add_update_semantics'):raise ValueError('Unexpected legacy API in selected runtime')
        semantics.add_update_semantics=lambda prim,label:semantics.add_labels(prim,[label],instance_name='class',overwrite=True)
        render_candidates(app,cfg,out)
        print('MODEL_FREE_PREFLIGHT_PASS render ISAAC6_CLASS_LABEL_ADAPTER',flush=True);code=0
    except Exception as exc:
        traceback.print_exc();print('MODEL_FREE_PREFLIGHT_FAIL',repr(exc),flush=True)
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
