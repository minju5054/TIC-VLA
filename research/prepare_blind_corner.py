"""Freeze one official selected route; bounded no-human pose preview, no inference."""
import argparse,json,sys,math
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.hospital_episode import official_episode,validate_config,digest
from research.records import write_json,provenance


def main():
    p=argparse.ArgumentParser();p.add_argument('--selection-dir',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
    source=Path(a.selection_dir);chosen=json.loads((source/'selected_route.json').read_text())
    if chosen is None:raise ValueError('No geometry-selected route')
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=False)
    rules=json.loads(Path('configs/research/blind_corner_protocol.json').read_text())
    cfg=yaml.safe_load(Path(rules['base_config']).read_text());src=official_episode(chosen['episode_id']);ep=src['episode']
    cfg.update(scenario='nova_hospital_blind_corner_e21',blind_corner_episode_id=chosen['episode_id'],official_episode=src,instruction=ep['instruction'])
    cfg['robot']['start_position']=[*ep['start'][:2],.11];cfg['robot']['start_yaw']=src['resolved_yaw_radians'];cfg['scene']['goal']=ep['goal'];cfg['nova_source']['episode']=src
    cfg['spatial_turn_gate']={'robot_conservative_radius_m':rules['robot_radius_m'],'note':'Blind-turn gate is blind_corner.baseline_gate, not former E16 spatial gate'}
    cfg['blind_corner']={'selection_dir':str(source.resolve()),'selected_candidate_id':chosen['id'],'baseline_gate':rules['baseline_gate'],'protocol_sha256':digest('configs/research/blind_corner_protocol.json')}
    validate_config(cfg)
    with (out/'baseline_config.yaml').open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
    write_json(out/'metadata.json',{'mode':'selected-route no-human preview','source_selection':str(source.resolve()),'model_calls':0,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1920,'height':1080});code=1
    try:
        from research.nova_render_saved import SavedRenderer
        from research.hospital_lights import apply_hospital_lights
        from research.nova_bright import image_stats,bright_pass
        from PIL import Image
        render=SavedRenderer(app,cfg);write_json(out/'lighting.json',apply_hospital_lights(render.stage,cfg['hospital_lighting']))
        poses=np.array(chosen['reference_poses']);index=chosen['transition']['fresh_reference_index'];rows=[]
        for name,i in [('start',0),('before_turn',max(0,index-1)),('reveal_reference',index)]:
            x,y,yaw=poses[i];obs={'position':[x,y,0.],'quaternion_wxyz':[math.cos(yaw/2),0,0,math.sin(yaw/2)]}
            matrix=render.pose({'observation':obs});rgb=render.capture();Image.fromarray(rgb).save(out/(name+'.png'))
            stats=image_stats(rgb);rows.append({'name':name,'reference_index':i,'camera_matrix':matrix,'brightness':stats,'bright_pass':bright_pass(stats,cfg['bright_validation'])})
        write_json(out/'preview.json',{'model_calls':0,'physics_steps':0,'views':rows,'note':'Geometric reference poses, not executed baseline'});print('BLIND_PREVIEW_COMPLETE',flush=True);code=0
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
