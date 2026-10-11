"""At most 12 preregistered Hospital carts: static queries and saved-pose renders only."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json, provenance
from research.reveal_window import evidence_hashes, verify_hashes, validate_destination
from research.analyze_nova_dynamic_handoff import load_saved
from research.obstacle_reveal import cart_at, temporal_proxy, bypass_paths, reveal_decision
from research.cart_geometry import outline, path_clearance
from research.hospital_cart import author, mask_stats, COPY

CONFIG = ROOT/'configs/research/obstacle_reveal_acquisition.json'


def read(path):
    return json.loads(Path(path).read_text())


def freeze(out):
    config = read(CONFIG); baseline = ROOT/config['baseline_run']; run = load_saved(baseline)
    if len(run['events']) != 48 or run['contacts'] or not run['complete']:
        raise ValueError('Validated 48-request contact-free baseline required')
    out = validate_destination(out, [baseline]); out.mkdir(parents=True)
    for name in ['raw', 'derived', 'figures']: (out/name).mkdir()
    prior = read(ROOT/config['asset_identity']); bounds = read(ROOT/config['asset_audit'])['source_local_bounds']
    candidates = []
    for rid in config['station_request_ids']:
        for offset in config['left_offsets_m']:
            candidates.append({'candidate_id': len(candidates)+1, 'station_request_id': rid,
                               'left_offset_m': offset, 'cart': cart_at(run['events'], bounds, rid, offset)})
    if len(candidates) > min(12, config['candidate_cap']): raise ValueError('Candidate cap exceeded')
    sources = [baseline, ROOT/config['asset_audit'], ROOT/config['asset_identity'],
               ROOT/config['collision_inventory'], ROOT/config['visibility_config'], CONFIG,
               ROOT/'outputs/e16-c26-preflight-20261011-01', ROOT/'outputs/nova-e16-c26-cart-20261011-01',
               ROOT/'outputs/e16-cart-preflight-20261010-01', ROOT/'outputs/nova-e16-cart-20261010-01']
    manifest = evidence_hashes(sources)
    code = evidence_hashes([p for p in (ROOT/'research').glob('*.py')]+[ROOT/'DynaNav/ticvla.py', ROOT/'DynaNav/ticvla_vlm.py', ROOT/'scripts/isaac6_python.sh'])
    write_json(out/'protocol.json', {'config': config, 'config_sha256': hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
        'candidates': candidates, 'source_evidence_sha256': manifest, 'code_sha256': code,
        'visibility_rules': prior['visibility_rules'], 'robot_radius_m': prior['robot_radius_m'],
        'baseline_run': str(baseline), 'model_calls': 0, 'candidate_selection_before_model': True,
        'limits': {'candidate_count': 12, 'additional_baselines': 0, 'obstacle_runs_if_qualified': 1, 'retries': 0},
        'lighting': 'Unchanged validated E16 fixture profile; original Hospital geometry/native lights/exposure retained',
        'baseline_navigation_evidence': {'finite_predictions': 48, 'nonfloor_contacts': 0,
             'source_summary_sha256': manifest[str(baseline/'summary.json')]},
        **provenance(ROOT)})
    print('PROTOCOL_FROZEN', len(candidates), flush=True)


def load_protocol(out):
    protocol = read(out/'protocol.json')
    verify_hashes(protocol['source_evidence_sha256']); verify_hashes(protocol['code_sha256'])
    return protocol, load_saved(Path(protocol['baseline_run']))


def physical(out):
    protocol, run = load_protocol(out); settings = protocol['config']
    if (out/'physical.json').exists(): raise FileExistsError('Physical results already exist')
    folder = out/'raw/physical'; folder.mkdir()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'disable_viewport_updates': True}); code = 1
    try:
        import omni.usd, omni.client, omni.timeline
        from pxr import UsdGeom
        from research.blind_corner_queries import Queries
        from research.audit_hospital_lights import inspect_stage
        timeline = omni.timeline.get_timeline_interface(); timeline.stop()
        stage = omni.usd.get_context().get_stage(); stage.GetRootLayer().subLayerPaths.append(run['cfg']['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(stage, 1.); UsdGeom.SetStageUpAxis(stage, 'Z')
        original_colliders = inspect_stage(stage)[-1]
        if original_colliders != read(ROOT/settings['collision_inventory']): raise ValueError('Hospital collider inventory changed')
        prior = read(ROOT/settings['asset_identity'])
        for url, expected in prior['asset_sha256'].items():
            result, _, data = omni.client.read_file(url)
            if result != omni.client.Result.OK or hashlib.sha256(bytes(data)).hexdigest() != expected:
                raise ValueError('Original USD asset changed')
        query = Queries(stage); results = []; radius = protocol['robot_radius_m']
        for candidate in protocol['candidates']:
            cid = candidate['candidate_id']; cart = candidate['cart']; xy = cart['center_world_xyz'][:2]
            support = query.q.raycast_closest(query.carb.Float3(*xy, .3), query.carb.Float3(0, 0, -1), .6)
            if not support['hit']: raise ValueError('No floor ray hit')
            cart = cart_at(run['events'], cart['local_bounds'], candidate['station_request_id'],
                           candidate['left_offset_m'], float(support['position'][2]))
            h = cart['footprint_half_m']; z = cart['height_m']/2
            floors = [query.floor(p) for p in outline(cart)[::17]]
            overlaps = query.overlap([*xy, cart['center_world_xyz'][2]+z+.001], extent=[*h, z-.001], yaw=cart['yaw_rad'])
            initial = path_clearance(run['xy'][:2], cart, radius)['minimum_clearance_m']
            temporal = [temporal_proxy(run, cart, i, radius, settings['native_waypoint_dt_s']) for i in range(2, 49)]
            traces = []; bypass = None
            if not overlaps and all(f[0] for f in floors):
                for proposal in bypass_paths(run, cart, radius, settings):
                    trace = query.robot_path(proposal['world_xy'], radius+settings['static_margin_m'])
                    okay = proposal['cart_clearance_m']>0 and all(q['floor'] and not q['overlap'] for q in trace)
                    traces.append({**proposal, 'pass': okay, 'query_samples': trace})
                    if okay and bypass is None: bypass = proposal
            frozen = {k: prior[k] for k in ['source_prim','asset_url','scene_usd','copy_prim','asset_sha256',
                       'source_prim_signature','source_world_matrix_row_vector','visibility_rules','robot_radius_m']}
            frozen.update(cart=cart, source_evidence_sha256=protocol['source_evidence_sha256'],
                          candidate_id=cid, coordinate_convention='World metres/+Z up/yaw CCW; row-vector USD; original geometry',
                          before_model_inference=True)
            copied = author(stage, frozen)
            stage.RemovePrim(COPY)  # Independent model-free candidate scene, never an executing navigation scene.
            checks = {'floor_support': all(f[0] for f in floors), 'no_static_overlap': not overlaps,
                      'initial_clearance_positive': initial>0, 'connected_local_bypass': bypass is not None}
            result = {**candidate, 'cart': cart, 'checks': checks, 'physical_pass': all(checks.values()),
                      'floor_checks': floors, 'overlap_paths': overlaps, 'initial_clearance_m': initial,
                      'bypass': bypass, 'temporal_pairs': temporal,
                      'temporal_pass_pair_ids': [p['fresh_request_id'] for p in temporal if p['pass']],
                      'frozen_asset': frozen, 'copied_asset_check': copied}
            result['render_permitted'] = result['physical_pass'] and bool(result['temporal_pass_pair_ids'])
            write_json(folder/f'candidate_{cid:02d}.json', {'result': result, 'bypass_tests': traces})
            results.append(result)
            print('PHYSICAL_CANDIDATE', cid, checks, 'temporal', result['temporal_pass_pair_ids'], flush=True)
        if timeline.is_playing() or timeline.get_current_time()!=0: raise ValueError('Static query advanced timeline')
        if inspect_stage(stage)[-1] != original_colliders: raise ValueError('Hospital geometry modified')
        write_json(out/'physical.json', {'candidates': results, 'collider_count': len(original_colliders),
                   'model_calls': 0, 'physics_steps': 0, 'rgb_renders': 0, 'collider_inventory_unchanged': True})
        verify_hashes(protocol['source_evidence_sha256']); code = 0
    finally:
        app.close(exit_code=code)


def semantic(out):
    protocol, run = load_protocol(out); physical = read(out/'physical.json')['candidates']
    folder = out/'raw/visibility'; folder.mkdir()
    survivors = [c for c in physical if c['render_permitted']]
    if not survivors:
        write_json(out/'visibility.json', []); return
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RayTracedLighting', 'width': 1920, 'height': 1080}); code = 1
    try:
        import omni.replicator.core as rep
        from PIL import Image
        from research.nova_render_saved import SavedRenderer
        from research.hospital_lights import apply_hospital_lights
        from research.hospital_scene import rotation_wxyz
        render = SavedRenderer(app, run['cfg']); apply_hospital_lights(render.stage, run['cfg']['hospital_lighting'])
        semantic = rep.AnnotatorRegistry.get_annotator('semantic_segmentation', init_params={'colorize': False})
        semantic.attach([render.product])
        extrinsic = np.asarray(read(Path(protocol['baseline_run'])/'robot_asset.json')['camera_body_transform'])
        results = []
        for candidate in survivors:
            cid = candidate['candidate_id']; dst = folder/f'candidate_{cid:02d}'; dst.mkdir()
            author(render.stage, candidate['frozen_asset']); rows = []
            for event in run['events']:
                matrix = render.pose(event); obs = event['observation']; body = np.eye(4)
                body[:3, :3] = rotation_wxyz(obs['quaternion_wxyz']).T; body[3, :3] = obs['position']
                np.testing.assert_allclose(matrix, extrinsic@body, atol=1e-6, rtol=0)
                rgb = render.capture(); packed = semantic.get_data()
                stats, mask = mask_stats(packed, protocol['visibility_rules']); rid = event['request_id']
                row = {'request_id': rid, 'sim_time': obs['sim_time'], 'observation': obs,
                       'camera_world_matrix_row_vector': matrix, **stats}
                for suffix, pixels in [('.png', rgb), ('.mask.png', (mask*255).astype('uint8'))]:
                    with (dst/f'C{rid:02d}{suffix}').open('xb') as f: Image.fromarray(pixels).save(f, format='PNG')
                with (dst/f'C{rid:02d}.semantic.npy').open('xb') as f: np.save(f, packed['data'], allow_pickle=False)
                write_json(dst/f'C{rid:02d}.semantic-labels.json', packed['info']['idToLabels'])
                write_json(dst/f'C{rid:02d}.json', row); rows.append(row)
                print('SEMANTIC_CANDIDATE', cid, 'C'+str(rid), stats['state'], stats['cart_visible_pixel_count'], flush=True)
                if stats['state']=='CLEAR': break  # First CLEAR fixed, later observations cannot repair prior history.
            result = {'candidate_id': cid, 'rows': rows,
                      **reveal_decision(rows, candidate['temporal_pairs'], candidate['physical_pass'])}
            write_json(dst/'result.json', result); results.append(result)
            render.stage.RemovePrim(COPY)
        write_json(out/'visibility.json', results)
        verify_hashes(protocol['source_evidence_sha256']); code = 0
    finally:
        app.close(exit_code=code)


def summarize(out):
    protocol, run = load_protocol(out); physical = read(out/'physical.json')['candidates']; visible = read(out/'visibility.json')
    indexed = {v['candidate_id']: v for v in visible}; rows = []
    for candidate in physical:
        v = indexed.get(candidate['candidate_id'])
        failures = [k.upper()+'_FAIL' for k, value in candidate['checks'].items() if not value]
        if not candidate['temporal_pass_pair_ids']: failures.append('NO_SAME_PAIR_TEMPORAL_OPPORTUNITY')
        if v: failures += v['failures']
        elif candidate['render_permitted']: raise ValueError('Missing permitted visibility result')
        rows.append({**candidate, 'visibility': v, 'failures': sorted(set(failures)), 'qualified': bool(v and v['qualified'])})
    passing = sorted([c for c in rows if c['qualified']], key=lambda c: (
        c['visibility']['temporal_at_first_clear']['switch_to_conflict_margin_s'],
        -c['visibility']['event']['first_clear']['cart_visible_fraction'],
        -c['bypass']['cart_clearance_m'], *c['cart']['center_world_xyz']))
    from collections import Counter
    summary = {'decision': 'STAGE A PASS; FREEZE ONE SCENARIO BEFORE MODEL' if passing else 'NO QUALIFYING PHYSICAL REVEAL SCENARIO',
        'candidate_count': len(rows), 'physical_pass_count': sum(c['physical_pass'] for c in rows),
        'rendered_candidate_count': len(visible), 'rgb_semantic_capture_count': sum(len(v['rows']) for v in visible),
        'qualified_count': len(passing), 'selected_candidate_id': passing[0]['candidate_id'] if passing else None,
        'rejection_counts_multilabel': dict(Counter(f for c in rows for f in c['failures'])),
        'obstacle_run_id': None, 'model_calls': 0, 'new_navigation_runs': 0, 'graph_solves': 0,
        'source_unchanged': True, 'actual_obstacle_conditioned_pair': None,
        'no_qualifying_candidate_stop': not passing}
    write_json(out/'derived/candidates.json', rows); write_json(out/'derived/selected_candidate.json', passing[0] if passing else None)
    write_json(out/'summary.json', summary)
    print(json.dumps(summary, indent=2), flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['freeze','physical','semantic','summarize'], required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args(); globals()[args.phase](Path(args.output_dir).resolve())
