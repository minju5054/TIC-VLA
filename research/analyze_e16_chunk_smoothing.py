"""Saved-only E16 fitting residual / handoff sensitivity audit; no graph solves."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.compare_e16_se2_graph import load_sources, serial, csv_save
from research.analyze_e16_handoff_audit import phase_at, distribution, editable_overlap, CONFIG as AUDIT_CONFIG, RULES, INVENTORY
from research.route_geometry import temporal_overlap, boundary_seam, rotation
from research.e16_se2_metrics import execution_direction
from research.e16_curve_fitting import Curve, methods, chunk_metrics, chord_basis, tangent, angle_gap, rms
from research.records import write_json, provenance
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination

CONFIG = ROOT/'configs/research/e16_chunk_smoothing.json'
GRAPH_CONFIG = ROOT/'configs/research/e16_se2_graph_offline.json'


def boundary_metrics(curve, event, execution_heading, config):
    elapsed = event['application']['sim_time']-event['observation']['sim_time']
    pose, b = event['agent_pose_at_observation'], np.array(event['agent_pose_at_application'])
    point = curve.world(elapsed, pose, allow_bridge=True)
    d = curve.world(elapsed, pose, derivative=1, allow_bridge=True)
    _, valid, speed = tangent(d, config['minimum_derivative_m_per_nominal_s'])
    phi = float(np.arctan2(d[1], d[0])) if valid else None
    return dict(boundary_position_m=float(np.linalg.norm(b[:2]-point)),
        boundary_robot_heading_deg=float(angle_gap(phi, b[2])) if valid else None,
        boundary_executed_heading_deg=float(angle_gap(phi, execution_heading)) if valid and execution_heading is not None else None,
        boundary_derivative_valid=bool(valid), boundary_derivative_speed=float(speed),
        boundary_heading_rad=phi, boundary_world_xy=point, boundary_derivative_world=d,
        boundary_origin_bridge_used=bool(elapsed < curve.times[0]),
        boundary_invalid_reason=None if valid else 'Derivative below frozen nominal speed floor or nonfinite')


def overlap_metrics(old, fresh, oe, fe, q, basis_local, config):
    if not len(q):
        return dict(point_count=0, rmse_m=None, undefined_reason='No queries strictly inside common support'), {}
    ot, ft = oe['observation']['sim_time'], fe['observation']['sim_time']
    op = old.world(q-ot, oe['agent_pose_at_observation']); fp = fresh.world(q-ft, fe['agent_pose_at_observation'])
    od = old.world(q-ot, oe['agent_pose_at_observation'], 1); fd = fresh.world(q-ft, fe['agent_pose_at_observation'], 1)
    _, ov, _ = tangent(od, config['minimum_derivative_m_per_nominal_s'])
    _, fv, _ = tangent(fd, config['minimum_derivative_m_per_nominal_s']); both = ov&fv
    delta = fp-op; lengths = np.linalg.norm(delta, axis=1)
    basis = basis_local @ rotation(fe['agent_pose_at_observation'][2]).T
    normal = np.column_stack([-basis[:, 1], basis[:, 0]])
    forward = np.sum(delta*basis, axis=1); lateral = np.sum(delta*normal, axis=1)
    body = delta @ rotation(fe['agent_pose_at_observation'][2])
    gap = angle_gap(np.arctan2(fd[:, 1], fd[:, 0]), np.arctan2(od[:, 1], od[:, 0])); gap[~both] = np.nan
    row = dict(point_count=len(q), rmse_m=rms(lengths), max_m=float(lengths.max()),
        valid_revision_basis_count=int(np.isfinite(forward).sum()), valid_tangent_count=int(both.sum()),
        mean_abs_tangent_deg=float(np.nanmean(gap)) if both.any() else None,
        max_abs_tangent_deg=float(np.nanmax(gap)) if both.any() else None)
    for axis, array in [('forward', forward), ('lateral', lateral), ('body_forward', body[:, 0]), ('body_left', body[:, 1])]:
        good = array[np.isfinite(array)]
        row.update({f'mean_abs_{axis}_m':float(np.mean(np.abs(good))) if len(good) else None,
                    f'max_abs_{axis}_m':float(np.max(np.abs(good))) if len(good) else None,
                    f'mean_signed_{axis}_m':float(np.mean(good)) if len(good) else None,
                    f'{axis}_rmse_m':rms(good)})
    return row, dict(absolute_times=q, old_world_xy=op, fresh_world_xy=fp, delta_world_xy=delta,
        old_derivative_world=od, fresh_derivative_world=fd, reference_tangent_world=basis,
        forward_revision=forward, lateral_revision=lateral, tangent_gap_deg=gap)


def summarize(rows, kind):
    keys = sorted({k for r in rows for k,v in r.items() if isinstance(v, (int, float, np.number)) and not isinstance(v, (bool, np.bool_))}-{'request_id','old_request_id','fresh_request_id'})
    result = []
    for run in ['A', 'B', 'ALL']:
        for method in sorted({r['method'] for r in rows}):
            pool = [r for r in rows if r['method']==method and (run=='ALL' or r['run']==run)]
            groups = {'all':pool, 'nonbootstrap':[r for r in pool if not r['bootstrap']],
                'straight':[r for r in pool if r['phase'] in ['initial_travel','hallway_straight']],
                **{p:[r for r in pool if r['phase']==p] for p in ['initial_travel','corner_approach','left_turn','hallway_straight','near_goal']}}
            for group, subset in groups.items():
                for key in keys:
                    result.append(dict(kind=kind, run=run, method=method, group=group, sample_count=len(subset), metric=key,
                        **distribution([r.get(key) for r in subset])))
    return result


def pair_analysis(name, i, run, curves, chunk_rows, audit, graph, config, graph_config):
    oe, fe = run['events'][i-1:i+1]; oldw, freshw = run['worlds'][i-1:i+1]
    ot, ft, st = oe['observation']['sim_time'], fe['observation']['sim_time'], fe['application']['sim_time']
    full = temporal_overlap(oldw, freshw, ot, ft, config['native_dt_s'])
    edit = editable_overlap(full, oldw, freshw, ot, ft, st, config['native_dt_s'])
    queries = {'full':full['absolute_times'], 'editable':edit['absolute_times'][edit['absolute_times']>st+config['time_roundoff_tolerance_s']]}
    raw = curves[i+1]['RAW']; bases = {k:chord_basis(raw, q-ft, graph_config['tangent'])[0] for k,q in queries.items()}
    exec_heading, exec_diag = execution_direction(run, fe, graph_config)
    seam = boundary_seam(freshw, fe['observation'], fe['application'], run['times'], run['xy'])
    saved = audit['rows'][i-1]
    if abs(seam['raw_boundary_position_gap_m']-saved['boundary_position_m'])>1e-12:
        raise ValueError('Original boundary audit mismatch')
    ginput = graph['inputs']; gtimes = np.array(ginput['times']); gref = np.array(ginput['reference'], float)
    chord = gref[0, 2]
    prefix = dict(run=name, pair_id=f'{name}_C{i:02d}_C{i+1:02d}', old_request_id=i, fresh_request_id=i+1,
        bootstrap=i==1, phase=saved['phase'], old_observation_sim_time=ot, fresh_observation_sim_time=ft,
        switch_sim_time=st, observation_gap_s=ft-ot, existing_graph_eligibility=ginput['eligibility'])
    detail = dict(**prefix, old_observation_pose=oe['agent_pose_at_observation'], fresh_observation_pose=fe['agent_pose_at_observation'],
        boundary=fe['agent_pose_at_application'], execution_direction_diagnostic=exec_diag,
        measured_committed_times=run['times'][(run['times']<=st)&(run['times']>=st-1)],
        measured_committed_xy=run['xy'][(run['times']<=st)&(run['times']>=st-1)],
        original_audit_row=saved, original_graph_metrics=graph['metrics'], graph_reference_times=gtimes,
        graph_reference_poses=gref, graph_tangent_diagnostics=ginput['tangent_diagnostics'], methods={})
    rows = []
    for method, _ in methods(config):
        old, fresh = curves[i].get(method), curves[i+1].get(method)
        row = dict(**prefix, method=method, status='PASS' if old is not None and fresh is not None else 'FIT_FAILURE',
            raw_boundary_position_m=saved['boundary_position_m'], raw_audit_boundary_robot_heading_deg=saved['boundary_robot_heading_deg'],
            raw_audit_boundary_executed_heading_deg=saved['boundary_executed_heading_deg'],
            raw_graph_chord_robot_heading_deg=float(angle_gap(chord, fe['agent_pose_at_application'][2])) if np.isfinite(chord) else None,
            raw_graph_chord_executed_heading_deg=float(angle_gap(chord, exec_heading)) if np.isfinite(chord) and exec_heading is not None else None,
            raw_legacy_full_rmse_m=saved['full_rmse_m'], raw_legacy_boundary_inclusive_editable_rmse_m=saved['editable_rmse_m'])
        if old is None or fresh is None:
            rows.append(row); continue
        boundary = boundary_metrics(fresh, fe, exec_heading, config)
        row.update({k:v for k,v in boundary.items() if k not in ['boundary_world_xy','boundary_derivative_world']})
        data = dict(boundary=boundary, overlaps={})
        for scope,q in queries.items():
            values, coords = overlap_metrics(old, fresh, oe, fe, q, bases[scope], config)
            row.update({scope+'_'+k:v for k,v in values.items()}); data['overlaps'][scope]=coords
        if method == 'RAW':
            np.testing.assert_allclose(row['full_rmse_m'],saved['full_rmse_m'],atol=1e-12)
            np.testing.assert_allclose(row['full_mean_abs_body_left_m'],saved['full_mean_abs_lateral_m'],atol=1e-12)
        gp = fresh.world(gtimes-ft, fe['agent_pose_at_observation'], allow_bridge=True)
        gd = fresh.world(gtimes-ft, fe['agent_pose_at_observation'], 1, allow_bridge=True)
        _, gv, gs = tangent(gd, config['minimum_derivative_m_per_nominal_s'])
        headings = np.arctan2(gd[:,1],gd[:,0]); headings[~gv]=np.nan
        first = gp[1]-gp[0]; first_length = float(np.linalg.norm(first))
        row.update(reference_node_count=len(gv), reliable_reference_derivatives=int(gv.sum()), first_sampled_segment_m=first_length,
            sampled_first_executed_heading_deg=float(angle_gap(np.arctan2(first[1],first[0]),exec_heading)) if first_length>=.01 and exec_heading is not None else None)
        data.update(reference_world_xy=gp, reference_derivative_world=gd, reference_heading_rad=headings, reference_derivative_speed=gs)
        if method != 'RAW':
            cr_o,cr_f = chunk_rows[(i,method)],chunk_rows[(i+1,method)]
            for label,cr in [('old',cr_o),('fresh',cr_f)]:
                for key in ['forward_rmse_m','lateral_rmse_m','forward_mad_m','lateral_mad_m','xy_rmse_m','reliable_tangent_samples']:
                    row[label+'_fit_'+key]=cr.get(key)
            reliable = min(cr_o['reliable_tangent_samples'],cr_f['reliable_tangent_samples'])>=config['minimum_ratio_tangent_samples']
            denom_f = np.hypot(cr_f['forward_rmse_m'],cr_f['lateral_rmse_m']) if cr_f['forward_rmse_m'] is not None else 0.
            denom_o = np.hypot(cr_o['forward_rmse_m'],cr_o['lateral_rmse_m']) if cr_o['forward_rmse_m'] is not None else 0.
            scale = np.hypot(denom_f,denom_o)
            ok = reliable and min(denom_f,scale)>config['minimum_ratio_denominator_m']
            row.update(ratio_denominator_fresh_rms_m=float(denom_f),ratio_denominator_pair_quadrature_rms_m=float(scale),
                smoothed_boundary_over_fresh_fit_rms=row['boundary_position_m']/denom_f if ok else None,
                raw_boundary_over_fresh_fit_rms=saved['boundary_position_m']/denom_f if ok else None,
                smoothed_full_rmse_over_pair_fit_rms=row['full_rmse_m']/scale if ok else None,
                raw_full_rmse_over_pair_fit_rms=saved['full_rmse_m']/scale if ok else None,
                ratio_invalid_reason=None if ok else 'Too few reliable fitted tangents or near-zero fitting residual denominator')
        rows.append(row);detail['methods'][method]=data
    return rows, detail


def sensitivity(detail, rows, config):
    names=[m for m,_ in methods(config) if m.startswith('SPLINE')]
    by={r['method']:r for r in rows};out={k:detail[k] for k in ['run','pair_id','old_request_id','fresh_request_id','bootstrap','phase','existing_graph_eligibility']}
    def spread(values):
        v=np.array([x for x in values if x is not None],float)
        return float(np.max(angle_gap(v[:,None],v[None,:]))) if len(v)==3 else None
    out['boundary_heading_spread_deg']=spread([by[m].get('boundary_heading_rad') for m in names])
    for key in ['boundary_position_m','full_rmse_m','full_mean_abs_lateral_m','full_mean_abs_body_left_m']:
        vals=[by[m].get(key) for m in names]
        out[key+'_range']=float(max(vals)-min(vals)) if all(v is not None for v in vals) else None
        for m in names:out[m+'_'+key]=by[m].get(key)
    if all(m in detail['methods'] for m in names):
        h=np.array([detail['methods'][m]['reference_heading_rad'] for m in names]);good=np.isfinite(h).all(axis=0)
        out.update(common_reliable_reference_nodes=int(good.sum()),reference_node_count=h.shape[1],
            all_spline_reference_nodes_reliable=bool(good.all()),
            maximum_reference_heading_spread_deg=float(np.max([angle_gap(h[j,good],h[k,good]).max() for j,k in [(0,1),(0,2),(1,2)]])) if good.any() else None)
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--freeze',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    cfg=json.loads(CONFIG.read_text());receipt=json.loads(a.freeze.read_text())
    if hashlib.sha256(CONFIG.read_bytes()).hexdigest()!=receipt['config_sha256']:raise ValueError('Frozen fitting policy changed')
    verify_hashes(receipt['protected_sha256']);verify_hashes(receipt['existing_code_sha256'])
    graph_cfg=json.loads(GRAPH_CONFIG.read_text());runs,audits=load_sources(cfg)
    hashes=evidence_hashes([CONFIG,Path(__file__),ROOT/'research/e16_curve_fitting.py',ROOT/'research/plot_e16_chunk_smoothing.py',ROOT/'tests/test_e16_chunk_smoothing.py',a.freeze])
    out=validate_destination(a.output_dir,[r['path'] for r in runs.values()]+[ROOT/cfg['previous_graph_dir']]);out.mkdir(parents=True)
    for d in ['chunks','pairs','figures','validation']:(out/d).mkdir()
    chunk_rows=[];pair_rows=[];sensitivity_rows=[];pair_details={};fit_failures=[];rules=json.loads(RULES.read_text())
    for name,run in runs.items():
        cache={};lookup={}
        for e,native in zip(run['events'],run['chunks']):
            rid=e['request_id'];raw=Curve(native,'RAW',None,cfg);cache[rid]={'RAW':raw}
            phase=phase_at(e['observation'],run['cfg'],audits[name]['navigation'],rules['goal_region_radius_m'],cfg['native_dt_s'])
            detail=dict(run=name,request_id=rid,observation_pose=e['agent_pose_at_observation'],observation_sim_time=e['observation']['sim_time'],
                native_file=str(run['path']/f'raw/requests/request_{rid:06d}.npy'),native_local_xy=native,
                native_world_xy=run['worlds'][rid-1],phase=phase,fits={})
            for method,budget in methods(cfg)[1:]:
                row=dict(run=name,request_id=rid,method=method,budget_m=budget,phase=phase,bootstrap=rid==1)
                try:
                    curve=Curve(native,method,budget,cfg);values,fit_detail=chunk_metrics(curve,raw,cfg)
                    cache[rid][method]=curve;row.update(status='PASS',**values)
                    fit_detail['grid_world_xy']=curve.world(fit_detail['grid_nominal_times'],e['agent_pose_at_observation'])
                    detail['fits'][method]=fit_detail
                except (ValueError,FloatingPointError,np.linalg.LinAlgError) as exc:
                    row.update(status='FIT_FAILURE',reason=str(exc));fit_failures.append(row.copy())
                chunk_rows.append(row);lookup[(rid,method)]=row
            write_json(out/'chunks'/f'{name}_C{rid:02d}.json',serial(detail))
        for i in range(1,48):
            pair_id=f'{name}_C{i:02d}_C{i+1:02d}'
            graph=json.loads((ROOT/cfg['previous_graph_dir']/'pairs'/f'{pair_id}.json').read_text())
            rows,detail=pair_analysis(name,i,run,cache,lookup,audits[name],graph,cfg,graph_cfg)
            pair_rows.extend(rows);sensitivity_rows.append(sensitivity(detail,rows,cfg));pair_details[pair_id]=detail
            write_json(out/'pairs'/f'{pair_id}.json',serial(detail))
        print(name,'48 chunks / 47 pairs analyzed',flush=True)
    assert len(chunk_rows)==384 and len(pair_rows)==470 and len(pair_details)==94
    for label,rows in [('chunk_metrics',chunk_rows),('pair_metrics',pair_rows),('smoothing_sensitivity',sensitivity_rows)]:
        csv_save(out/(label+'.csv'),rows);write_json(out/(label+'.json'),serial(rows))
    stats=summarize(chunk_rows,'chunk')+summarize(pair_rows,'pair');csv_save(out/'statistics.csv',stats)
    write_json(out/'summary.json',serial(dict(chunks=96,pairs=94,independent_runs=2,fit_failures=fit_failures,statistics=stats)))
    from research.plot_e16_chunk_smoothing import figures
    selections=figures(out,chunk_rows,pair_rows,sensitivity_rows,pair_details,cfg)
    write_json(out/'figure_selection.json',selections)
    verify_hashes(receipt['protected_sha256']);verify_hashes(receipt['existing_code_sha256']);verify_hashes(hashes)
    for root in receipt['protected_roots']:
        expected={p:h for p,h in receipt['protected_sha256'].items() if Path(root) in Path(p).parents}
        if evidence_hashes([Path(root)])!=expected:raise ValueError('Prior source/output inventory changed')
    write_json(out/'metadata.json',dict(config=cfg,source_and_prior_output_sha256=receipt['protected_sha256'],analysis_source_sha256=hashes,
        previous_code_sha256=receipt['existing_code_sha256'],provenance=provenance(ROOT),command=sys.argv,
        numpy=np.__version__,scipy=scipy.__version__,python=sys.version,source_and_previous_outputs_unchanged=True,
        graph_solves=0,model_calls=0,simulation_launches=0,fit_policy_frozen_before_analysis=True))
    print(str(out),flush=True)


if __name__=='__main__':main()
