"""Three explicitly requested experiments; no scene/controller parameter search."""
import copy
import json
from pathlib import Path
import numpy as np
from research.hospital_episode import official_episode, validate_config
from research.nova_source import source_audit

E21 = Path('outputs/e21-current-render-20261014T044932Z-68ab22')
E16 = Path('outputs/nova-e16-hospital-lights-baseline-20261010-01')
CART = Path('outputs/c24-adjacent-cart-v05-20261014T042243Z-1da200')
SCENARIOS = ('e21_c10_v08', 'e16_no_cart_v05', 'e16_no_cart_v05_left')
SUFFIX = ' If you see the wheelchair at C24, navigate around it on the left.'


def read(p):
    return json.loads(Path(p).read_text())


def baseline(scenario):
    if scenario not in SCENARIOS: raise ValueError('Unknown requested scenario')
    return E21 if scenario == SCENARIOS[0] else E16


def config(root, scenario, previous_run=None):
    reference = baseline(scenario)
    if scenario == SCENARIOS[0]:
        cfg = copy.deepcopy(read(root/E21/'metadata.json')['config'])
        if any(k in cfg for k in ('fixed_wheelchairs','stationary_human','adjacent_cart')):
            raise ValueError('E21 reference must have no added obstacle')
        rid = 10
        event = read(root/E21/f'raw/requests/request_{rid:06d}.json')
        prev = read(root/E21/f'raw/requests/request_{rid-1:06d}.json')['agent_pose_at_observation'][:2]
        next_ = read(root/E21/f'raw/requests/request_{rid+1:06d}.json')['agent_pose_at_observation'][:2]
        delta = np.array(next_)-prev
        if np.linalg.norm(delta)<.1:raise ValueError('Undefined saved path tangent')
        cfg['fixed_wheelchairs'] = [{'id':'C10','prim':'/World/ExactWheelchairC10',
            'source_prim':'/Root/SM_WheelChair_01a_37','label':'stationary_wheelchair_c10',
            'position':[*event['agent_pose_at_observation'][:2],0.],
            'reference_run':str(E21),'reference_request_id':rid,'reference_observation':event['observation'],
            'reference_tangent_rad':float(np.arctan2(delta[1],delta[0])), 'xy_offset_m':[0.,0.],
            'motion':'static original collision mesh; no rigid body or pose updates',
            'placement':'Actual world mesh-bounds centre at exact saved E21 C10 XY; lowest mesh point at floor; long axis along adjacent-observation tangent.'}]
        cfg['occlusion']={'visibility_rule':{'clearly_visible_min_fraction':.002,'clearly_visible_min_bbox_width':20,'clearly_visible_min_bbox_height':80,'hidden_max_pixels':0}}
        cfg['controller']['max_v']=.8;cfg['simulation']['predictions']=90
    else:
        cfg=copy.deepcopy(read(root/CART/'metadata.json')['config'])
        del cfg['adjacent_cart']
        if cfg['controller']['max_v']!=.5 or cfg['simulation']['predictions']!=144:
            raise ValueError('Expected frozen 0.5 m/s configuration')
    episode='episode_21' if scenario==SCENARIOS[0] else 'episode_16'
    cfg['official_episode']=official_episode(episode);cfg['nova_source']=source_audit(episode)
    validate_config(cfg)
    if scenario==SCENARIOS[2]:
        if not previous_run:raise ValueError('Completed step 2 is required for instruction-only step 3')
        previous=Path(previous_run)
        if previous.is_absolute() or '..' in previous.parts:raise ValueError('Expected repository-relative step 2 path')
        old=read(root/previous/'metadata.json')['config']
        if read(root/previous/'summary.json')['status']!='PASS' or cfg!=old:
            raise ValueError('Step 2 must have completed with exactly matching non-instruction config')
        cfg['instruction']+=SUFFIX
    return cfg


def geometry_config(cfg):
    """Keep original episode guard for geometry, explicit new text for inference."""
    result=copy.deepcopy(cfg)
    original=cfg['official_episode']['episode']['instruction']
    if cfg['instruction'] not in (original,original+SUFFIX):raise ValueError('Unexpected instruction')
    result['instruction']=original
    validate_config(result)
    return result
