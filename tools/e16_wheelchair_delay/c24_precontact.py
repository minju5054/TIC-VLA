"""Saved-only C24 OLD/FRESH pre-contact audit; no simulator or optimization."""
import argparse
from pathlib import Path
import subprocess
import sys
import numpy as np

from research.audit_e16_wheelchair_handoffs import (
    read, write, digest, snapshot, verify_snapshot, to_world, remaining, crossing,
    proxy_clearance, is_clear, build_geometry, rotation,
)
from tools.e16_wheelchair_delay.common import contact_is_nonfloor, contact_kind, motion_direction, boundary_metrics
from tools.e16_wheelchair_delay.evidence import load_run

ROOT=Path(__file__).resolve().parents[2]
SUITE='e16-delay-suite-20261014T095206Z-14e2d1'
POLICY=dict(min_segment_m=.02,max_adjacent_heading_change_deg=45.,safe_proxy_margin_m=.02,proxy_sample_step_m=.01)


def assess_pair(old_proxy,fresh_proxy,connector_proxy,boundary_proxy,predicted_crossing):
    """Safety qualifications refer only to the existing conservative geometry proxy."""
    candidate=bool(old_proxy.get('proxy_overlap_observed') and is_clear(fresh_proxy,POLICY)
                   and predicted_crossing['status']=='CROSSES')
    return dict(old_conflict_to_fresh_clear=candidate,
        connector_and_boundary_clear=candidate and is_clear(connector_proxy,POLICY) and is_clear(boundary_proxy,POLICY),
        physical_feasibility='NOT_ESTABLISHED_BY_SAVED_POLYLINES')


def audit_run(source, index, out):
    before=snapshot(source);write(out/f'run-{index}-source-before.json',before)
    for p,h in read(source/'raw-sha256.json').items():
        if digest(source/p)!=h:raise ValueError('Raw mismatch: '+p)
    run=load_run(source,True);cfg=run['cfg'];events=run['events'];xy=run['xy'];times=run['times']
    from json import loads
    contacts=[loads(line) for line in (source/'raw/robot_contacts.jsonl').read_text().splitlines()]
    nonfloor=[c for c in contacts if contact_is_nonfloor(c,cfg['robot']['prim'])]
    cutoff=nonfloor[0]['sim_time'];wheelchairs=read(source/'wheelchairs_asset.json');w=wheelchairs['C24']
    if not any(nonfloor[0][k].startswith(w['copy_prim']) for k in ('collider0','collider1')):
        raise ValueError('Expected C24 as first nonfloor contact')
    positive=next(c for c in nonfloor if contact_kind(c)['positive_impulse'])
    inventory=ROOT/'outputs/hospital-native-light-audit-20261010-01/collision_geometry.json'
    if digest(inventory)!=cfg['spatial_turn_gate']['source_inventory_sha256']:raise ValueError('Geometry mismatch')
    boxes,_,_=build_geometry(cfg,wheelchairs,read(inventory));own=[b for b in boxes if b['name']==w['copy_prim']]
    radius=cfg['spatial_turn_gate']['robot_conservative_radius_m'];step=POLICY['proxy_sample_step_m']
    threshold=cfg['controller']['max_v']*3+radius+float(np.linalg.norm(w['footprint_half_m']))
    near=np.flatnonzero(np.linalg.norm(xy-np.array(w['center_world_xyz'][:2]),axis=1)<=threshold)
    near_start=float(times[near[0]]) if len(near) else None
    worlds=[to_world(c,e['observation']['pose']) for c,e in zip(run['chunks'],events)]
    nominal=[e['observation']['sim_time']+.1*np.arange(1,31) for e in events]
    rows=[]
    for j in range(1,len(events)):
        e=events[j];t=e['application']['sim_time']
        if t>=cutoff:continue
        B=np.array(e['application']['pose']);tick=e['application']['tick']
        _,op,_=remaining(nominal[j-1],worlds[j-1],t);_,fp,_=remaining(nominal[j],worlds[j],t)
        ids=np.flatnonzero(nominal[j]>t+1e-9)
        connector=np.vstack([B[:2],worlds[j][ids[0]]]) if len(ids) else np.empty((0,2))
        old=proxy_clearance(op,own,radius,step);fresh=proxy_clearance(fp,own,radius,step)
        global_fresh=proxy_clearance(fp,boxes,radius,step)
        bc=proxy_clearance([B[:2]],boxes,radius,step);cc=proxy_clearance(connector,boxes,radius,step)
        co=crossing(op,w['center_world_xyz'],w['yaw_rad'],w['footprint_half_m'][1],radius)
        cf=crossing(fp,w['center_world_xyz'],w['yaw_rad'],w['footprint_half_m'][1],radius)
        qualify=assess_pair(old,global_fresh,cc,bc,cf)
        old_center=proxy_clearance(op,own,0.,step);fresh_center=proxy_clearance(fp,own,0.,step)
        direction=motion_direction(xy,tick)
        metric=boundary_metrics(worlds[j],nominal[j],B,t,direction['yaw_rad'],POLICY,observation_yaw=e['observation']['pose'][2])
        end=events[j+1]['application']['tick'] if j+1<len(events) else len(xy)-1
        applied=[c for c in nonfloor if t<c['sim_time']<=float(times[end])+1e-8]
        actual_xy=xy[tick:end+1]
        rows.append(dict(old_id=j,fresh_id=j+1,observation_sim_s=e['observation']['sim_time'],switch_sim_s=t,
            time_to_first_contact_s=cutoff-t,in_c24_window=near_start is not None and t>=near_start,
            old_c24_proxy=old,fresh_c24_proxy=fresh,fresh_global_proxy=global_fresh,
            boundary_proxy=bc,connector_proxy=cc,old_crossing=co,fresh_crossing=cf,
            old_centerline_proxy=old_center,fresh_centerline_proxy=fresh_center,
            centerline_conflict_to_clear=bool(old_center.get('proxy_overlap_observed') and is_clear(fresh_center,POLICY) and cf['status']=='CROSSES'),
            qualification=qualify,boundary=metric,old_command=e['old_controller_command'],fresh_command=e['controller_command'],
            pending_old_cm=100*float(np.linalg.norm(np.diff(xy[e['observation']['tick']:tick+1],axis=0),axis=1).sum()),
            actual_execution_until_next_switch_s=float(times[end])-t,actual_execution_nonfloor_callbacks=len(applied),
            actual_execution_xy_m=actual_xy.tolist(),old_remaining_world_xy_m=op.tolist(),fresh_remaining_world_xy_m=fp.tolist(),
            raw_connector_world_xy_m=connector.tolist(),observation_pose=e['observation']['pose']))
    near_rows=[r for r in rows if r['in_c24_window']]
    result=dict(index=index,source=str(source),total_precontact_handoffs=len(rows),near_c24_precontact_handoffs=len(near_rows),
        first_contact_sim_s=cutoff,first_positive_impulse_sim_s=positive['sim_time'],first_contact=nonfloor[0],
        robot_radius_m=radius,wheelchair=w,window_start_s=near_start,window_radius_m=threshold,
        candidate_fresh_ids=[r['fresh_id'] for r in rows if r['qualification']['old_conflict_to_fresh_clear']],
        centerline_only_revision_fresh_ids=[r['fresh_id'] for r in rows if r['centerline_conflict_to_clear']],
        fresh_c24_clear_and_crosses_ids=[r['fresh_id'] for r in rows if is_clear(r['fresh_c24_proxy'],POLICY) and r['fresh_crossing']['status']=='CROSSES'],
        rows=rows,source_file_count=len(before))
    verify_snapshot(source,before);write(out/f'run-{index}.json',result)
    print('RUN',index,'counts',len(rows),len(near_rows),'strict candidates',result['candidate_fresh_ids'],
          'centerline revisions',result['centerline_only_revision_fresh_ids'],'clear crosses',result['fresh_c24_clear_and_crosses_ids'],flush=True)
    for r in near_rows:
        print(r['old_id'],r['fresh_id'],round(r['switch_sim_s'],3),
            'old/fresh cm',round(r['old_c24_proxy']['certified_lower_m']*100,2),round(r['fresh_c24_proxy']['certified_lower_m']*100,2),
            'cross',r['old_crossing'].get('side','NONE'),r['fresh_crossing'].get('side','NONE'),
            'center',round(r['old_centerline_proxy']['certified_lower_m']*100,2),round(r['fresh_centerline_proxy']['certified_lower_m']*100,2),
            'B/conn',round(r['boundary_proxy']['certified_lower_m']*100,2),round(r['connector_proxy']['certified_lower_m']*100,2),
            'gap',None if r['boundary']['position_error_m'] is None else round(r['boundary']['position_error_m']*100,3),
            'cmd',r['fresh_command'],flush=True)
    return result


def main(out):
    if out.parent!=ROOT/'outputs':raise ValueError('Fresh direct outputs child required')
    out.mkdir(exist_ok=False)
    protocol=dict(scope='Saved pre-contact handoffs in actual failed E16 trials 3 and 4 only',
        cutoff='switch strictly before first nonfloor callback (including zero-impulse proximity); no postcontact rescue claims',
        sources=[SUITE+'-run-3-zero_hook',SUITE+'-run-4-no_hook'],policy=POLICY,
        qualification='OLD remaining C24 proxy overlap, FRESH remaining globally proxy-clear and predicts C24 centre-plane crossing',
        connector='Actual switch XY to first strictly future original native knot; a virtual straight connector, not controller execution',
        centerline='radius=0 diagnostic only; does not establish robot-safe bypass',
        coordinate_contract='Original Hospital world origin, m, +Z up; yaw CCW rad from +X; q wxyz. Native forward/left observation-body to world using recorded observation pose; never reanchor. Nominal knots obs+0.1..3.0 s, no extrapolation.',
        threshold_provenance='Unchanged previous conservative radius, 1 cm sampling, >2 cm clearance and tangent policy',
        no_graph=True,no_inference=True,no_simulator=True,argv=sys.argv,
        analysis_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),source_sha256=digest(Path(__file__)))
    write(out/'protocol.json',protocol)
    results=[audit_run(ROOT/'outputs'/name,i,out) for name,i in zip(protocol['sources'],(3,4))]
    write(out/'summary.json',dict(status='SAVED_AUDIT_COMPLETE',new_navigation_runs=0,
        strict_candidates={str(r['index']):r['candidate_fresh_ids'] for r in results},
        physical_connection_feasibility='NOT_VALIDATED; needs robot geometry and controller-following counterfactual evidence',
        source_integrity='PASS',policy_changed=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();main(a.out.resolve())
