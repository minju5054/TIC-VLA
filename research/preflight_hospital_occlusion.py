"""Model-free stationary placement using only the passing lights-on baseline.

Candidate scenes are separate stopped-time inspections, never navigation runs.
Only the first candidate satisfying the frozen visibility/leakage/space rule is
selected. No human inference is available to this program.
"""
import argparse
import json
import sys
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import RunRecords, write_json, provenance
from research.hospital_episode import digest
from research.analyze_nova_dynamic_handoff import load_saved
from research.nova_dynamic_geometry import remaining_curve, polyline_clearance
from research.nova_occlusion import VISIBILITY_RULES


def candidate_grid(run):
    """Quarter-metre grid in the new measured turn-exit corridor, east first."""
    gate = run['cfg']['spatial_turn_gate']
    corner = gate['source_south_corner']['world_aabb_max'][0]
    # Inspect the first four metres behind the actual corner. Near the measured
    # turn-exit corridor, including both wall-side bands. Do not exclude a
    # candidate merely because the executed centerline does not run through it.
    return [[float(x), float(y), 0.] for x in np.arange(np.floor(corner*4)/4, corner-4, -.25)
            for y in np.arange(np.ceil((gate['source_south_corner']['world_aabb_max'][1]+.25)*4)/4,
                               gate['source_north_wall']['world_aabb_min'][1]-.25, .25)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['baseline-run', 'baseline-analysis', 'output-dir']:
        parser.add_argument('--'+name, required=True)
    args = parser.parse_args()
    run = load_saved(args.baseline_run)
    analysis = Path(args.baseline_analysis)
    if json.loads((analysis/'summary.json').read_text())['baseline_spatial_gate_pass'] is not True:
        raise ValueError('Passing spatial baseline required before placement')
    cfg = run['cfg']; out = Path(args.output_dir)
    hashes = {str(p): digest(p) for root in [run['path'], analysis] for p in root.rglob('*') if p.is_file()}
    records = RunRecords(out, {'mode': 'model_free_occlusion_preflight', 'baseline_run': str(run['path']),
                              'source_evidence_sha256': hashes, 'model_calls': 0, **provenance(ROOT)})
    positions = candidate_grid(run)
    rule = {'visibility': VISIBILITY_RULES, 'candidate_positions': positions,
            'order': 'East to west, then south to north, first fully passing candidate',
            'candidate_source': 'New passing lights-on baseline measured corridor and actual fixed corner bounds',
            'selection': 'All pre-reveal observations hidden, no leakage; immediate OLD hidden, first clear FRESH; OLD remaining conflict; free footprint and bypass. Center rays only screen candidates, never establish visibility.',
            'human_radius_m': .25, 'robot_radius_m': cfg['spatial_turn_gate']['robot_conservative_radius_m'],
            'render_window': 'C1 through fixed spatial-gate crossing plus three observations',
            'no_navigation': True, 'model_calls': 0}
    write_json(out/'preregistered.json', rule)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RayTracedLighting', 'width': 1920, 'height': 1080})
    sim = None; code = 1
    try:
        import carb
        from omni.physx import get_physx_scene_query_interface
        from pxr import UsdGeom
        from research.hospital_lights_simulation import HospitalLightsSimulation
        sim = HospitalLightsSimulation(cfg, records)
        sim.world.pause()
        query = get_physx_scene_query_interface()
        radius = rule['robot_radius_m']
        human = {'prim': '/World/StationaryHuman', 'radius_m': .25, 'capsule_cylinder_height_m': 1.2,
                 'yaw': 0., 'asset': 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/male_adult_construction_03/male_adult_construction_03.usd'}

        def sphere_hits(point, r):
            hits = []
            def callback(hit):
                p = str(hit.collision)
                if p.startswith('/Root/'): hits.append(p)
                return True
            query.overlap_sphere(r, carb.Float3(*map(float, point)), callback, False)
            return sorted(set(hits))

        def space(position):
            x, y, _ = position
            floor = query.raycast_closest(carb.Float3(x,y,.3), carb.Float3(0,0,-1), .6)
            overlap = sorted(set(p for z in [.30,.65,1.,1.35,1.60] for p in sphere_hits([x,y,z], .25)))
            return {'floor_ok': bool(floor['hit'] and abs(floor['position'][2]) < .02 and abs(floor['normal'][2]) > .9),
                    'floor_prim': str(floor.get('collision', '')), 'overlap_paths': overlap}

        def bypass(position):
            x,y,_ = position; gate = cfg['spatial_turn_gate']; paths = []
            for by in [gate['center_y_min_m']+.05, gate['center_y_max_m']-.05]:
                if abs(by-y) <= radius+.25: continue
                points = [[float(bx), by, .65] for bx in np.arange(x-1.,x+1.01,.1)]
                hits = sorted(set(p for point in points for p in sphere_hits(point, radius)))
                paths.append({'centerline': points, 'wall_overlap_paths': hits,
                              'human_clearance_m': abs(by-y)-radius-.25, 'pass': not hits})
            return {'pass': any(p['pass'] for p in paths), 'paths': paths,
                    'note': 'Local two-metre bypass strip with conservative robot sphere; not navigation feasibility proof'}

        end = min(len(run['events']), json.loads((analysis/'summary.json').read_text())['spatial_gate']['crossing_request_id']+3)
        events = run['events'][:end]
        from research.hospital_scene import rotation_wxyz
        extrinsic = np.asarray(json.loads((out/'robot_asset.json').read_text())['camera_body_transform'])
        camera_matrices = []
        for e in events:
            body = np.eye(4); body[:3,:3] = rotation_wxyz(e['observation']['quaternion_wxyz']).T
            body[3,:3] = e['observation']['position']
            camera_matrices.append(extrinsic @ body)
        write_json(out/'camera_matrices.json', [m.tolist() for m in camera_matrices])
        camera = UsdGeom.Camera(sim.stage.GetPrimAtPath(cfg['camera']['prim']))
        hfov = float(camera.GetHorizontalApertureAttr().Get())/(2*float(camera.GetFocalLengthAttr().Get()))
        vfov = float(camera.GetVerticalApertureAttr().Get())/(2*float(camera.GetFocalLengthAttr().Get()))

        def sight(matrix, position):
            target = np.array(position)+[0,0,.9]; local = np.r_[target,1]@np.linalg.inv(matrix)
            frustum = bool(local[2]<0 and abs(local[0]/local[2])<hfov and abs(local[1]/local[2])<vfov)
            origin = matrix[3,:3]; delta = target-origin; length = float(np.linalg.norm(delta)); hits=[]
            def callback(hit):
                if str(hit.collision).startswith('/Root/'): hits.append({'prim':str(hit.collision),'distance_m':float(hit.distance)})
                return True
            query.raycast_all(carb.Float3(*map(float,origin)),carb.Float3(*map(float,delta/length)),length,callback,True)
            return {'frustum':frustum,'blocked':bool(hits),'first_wall':min(hits,key=lambda h:h['distance_m']) if hits else None}

        candidates = []
        for index,position in enumerate(positions,1):
            geometry = space(position); bp = bypass(position)
            candidate = {'id': index, 'position': position, 'geometry': geometry, 'bypass': bp}
            if geometry['floor_ok'] and not geometry['overlap_paths'] and bp['pass']:
                rays = [sight(m,position) for m in camera_matrices]; candidate['center_rays'] = rays
                first = next((i for i,r in enumerate(rays) if r['frustum'] and not r['blocked']),None)
                candidate['first_unblocked_center_request_id'] = first+1 if first is not None else None
                if first:
                    e0,e1=events[first-1:first+1]
                    _,remaining=remaining_curve(run['worlds'][first-1],e0['observation']['sim_time'],e1['observation']['sim_time'])
                    candidate['old_remaining_clearance_m'] = polyline_clearance(remaining,position,radius+.25) if remaining is not None else None
                clearances=[]
                for i in range(1,len(events)):
                    _,remaining=remaining_curve(run['worlds'][i-1],events[i-1]['observation']['sim_time'],events[i]['observation']['sim_time'])
                    clearances.append(polyline_clearance(remaining,position,radius+.25))
                candidate['remaining_clearance_by_fresh_request_2_onward_m']=clearances
                candidate['render_candidate']=bool(any(r['blocked'] for r in rays[:3]) and min(clearances)<0)
            candidates.append(candidate)
        write_json(out/'geometry_candidates.json',candidates)
        eligible = [c for c in candidates if c.get('render_candidate')]
        print('OCCLUSION_GEOMETRY_CANDIDATES',len(candidates),len(eligible),flush=True)
        write_json(out/'summary.json', {'geometry_preflight_complete':True, 'model_calls':0,
                    'baseline_spatial_gate_pass':True, 'candidate_count':len(candidates),
                    'render_eligible_count':len(eligible), 'render_window_count':end,
                    'human_template':human, 'occlusion_preflight_pass':False,
                    'note':'Geometry filter only; actual RGB/semantic rendering and visual review still required.'})
        if any(digest(p)!=h for p,h in hashes.items()):raise RuntimeError('Source evidence modified')
        print('OCCLUSION_GEOMETRY_COMPLETE',len(eligible),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(out/'failure.json',{'error':repr(exc),'model_calls':0})
    finally:
        if sim:sim.close()
        app.close(exit_code=code)


if __name__ == '__main__': main()
