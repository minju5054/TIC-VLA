"""Freeze the spatial trigger before any counterfactual metrics or new model run."""
import argparse
import csv
from pathlib import Path
import subprocess
import numpy as np
from tools.e16_wheelchair_delay.common import *

ROOT=Path(__file__).resolve().parents[2]
REFERENCE=ROOT/'outputs/two-wheelchairs-v08-20261014T015420Z-a56068'


def prepare(suite):
    if suite.parent != ROOT/'outputs' or suite.exists():raise ValueError('Fresh direct outputs child required')
    git=lambda *a:subprocess.check_output(['git',*a],cwd=ROOT,text=True).strip()
    if git('branch','--show-current')!='TIC-VLA-TEST':raise ValueError('Wrong branch')
    meta=read(REFERENCE/'metadata.json');cfg=meta['config']
    from tools.two_wheelchairs_v08.runner import current_config
    current,launch,changes=current_config(False)
    if cfg != current:raise ValueError('Navigation config differs from actual successful reference')
    for rel,h in meta['source_sha256'].items():
        if digest(ROOT/rel)!=h:raise ValueError('Historical runtime source changed: '+rel)
    e=read(REFERENCE/'raw/requests/request_000013.json');obs=e['observation']
    with (REFERENCE/'robot_state.csv').open() as f:rows=list(csv.DictReader(f))
    tick=obs['tick']; a=np.array([float(rows[tick-6][k]) for k in ('x','y')]);b=np.array(obs['pose'][:2])
    delta=b-a;direction=float(np.arctan2(delta[1],delta[0]))
    assert np.linalg.norm(delta)>.02
    wheels=read(REFERENCE/'wheelchairs_asset.json');w=wheels['C10'];radius=cfg['spatial_turn_gate']['robot_conservative_radius_m']
    corners=np.array([[-.5,-.6],[.5,-.6],[.5,.6],[-.5,.6]])@rotation(direction).T+b
    trigger=dict(center_world_xy_m=b.tolist(),progress_yaw_rad=direction,reference_yaw_rad=obs['pose'][2],
        half_along_m=.5,half_lateral_m=.6,yaw_half_width_rad=float(np.pi/6),min_forward_speed_m_s=.05,
        world_corners_xy_m=corners.tolist(),reference_request_id=13,reference_observation=obs,
        direction_measured_ticks=[tick-6,tick],direction_measured_delta_xy_m=delta.tolist(),
        c10_center_xy_m=w['center_world_xyz'][:2],c10_forward_world=rotation(w['yaw_rad'])[:,0].tolist(),
        c10_complete_passage_offset_m=radius+max(w['footprint_half_m']),
        selection='First nonbootstrap eligible observation, no prior nonfloor callback, no prior complete C10 passage; no FRESH input')
    protocol=dict(schema=1,reference_run=REFERENCE.name,plan=[dict(index=i+1,mode=m,extra_delay_s=d,ticks=ticks_for(d)) for i,(m,d) in enumerate(PLAN)],
        trigger=trigger,stage_a_extra_delay_s=[0.,.25,.5,.75],dt_s=1/60,nominal_dt_s=.1,nominal_horizon_s=3.,
        initial_seed=cfg.get('seed'),first_four_must_all_pass=True,baseline_replacement_allowed=False,
        stop_positive_runs_on_hold_safety_stop=True,maximum_navigation_runs=6,
        baseline_failure_policy='Preserve fixed four zero-delay controls; never proceed to positive hold unless all four pass; no replacement/retry.',
        policy=dict(min_segment_m=.02,max_adjacent_heading_change_deg=45.,safe_proxy_margin_m=.02,
                    proxy_sample_step_m=.01,large_position_gap_m=radius/4,large_direction_gap_rad=float(np.pi/6)),
        classification='Contact during hold first; missing evidence E; FRESH not proxy-qualified D; new connector-only proxy risk with B0/Bdelta/FRESH clear -> POTENTIAL; clear connector and small errors -> ROBUST; else MISMATCH.',
        contact_policy='Any run-phase nonfloor callback during hold aborts before FRESH; existing abs(normal.z)<0.5 policy. Impulse and separation reported separately.',
        no_graph=True,config_semantically_identical=True,coordinates='Hospital original world origin, m, +Z up; yaw CCW rad; q wxyz; native observation-body forward/left->world by observed R, never reanchor.')
    before=snapshot(REFERENCE);raw=read(REFERENCE/'raw-sha256.json');assert all(before[k]==h for k,h in raw.items())
    suite.mkdir()
    write(suite/'protocol.json',protocol);write(suite/'config.json',cfg);write(suite/'reference-before.json',before)
    receipt=read(ROOT/meta['receipt_relative']/'receipt.json')
    write(suite/'runtime-reference.json',dict(launch=launch,changes=changes,
        isaac6_root=receipt['isaac6_root'],isaac6_version=receipt['isaac6_version'],
        controller_source=receipt['controller_source'],controller_sha256=receipt['controller_sha256'],
        weights=receipt['weights'],external_sha256=receipt['external_sha256'],wheelchairs_expected=wheels,
        historical_receipt_sha256=digest(ROOT/meta['receipt_relative']/'receipt.json')))
    write(suite/'start-provenance.json',dict(head=git('rev-parse','HEAD'),upstream_sha=git('rev-parse','upstream/main'),
        branch=git('branch','--show-current'),remote_head=git('rev-parse','origin/TIC-VLA-TEST'),
        dirty_status=git('status','--short'),config_diff=[],protocol_sha256=digest(suite/'protocol.json'),
        protected_reference_files=len(before),historical_source_hashes_match=True,raw_manifest_entries=len(raw)))
    print('PROTOCOL_FROZEN',suite, 'trigger center',b.tolist(),'progress yaw',direction,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);a=p.parse_args();prepare(a.suite.resolve())
