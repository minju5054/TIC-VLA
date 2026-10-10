"""Small deterministic official-route screen, static geometry/rays only."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.blind_corner import reference_route,select_route
from research.records import write_json,provenance
from research.hospital_episode import digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);a=p.parse_args()
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=False)
    rules=json.loads(Path('configs/research/blind_corner_protocol.json').read_text())
    cfg=yaml.safe_load(Path(rules['base_config']).read_text());episode=yaml.safe_load(Path('DynaNav/configs/benchmark_full.yaml').read_text())[rules['official_episode_id']]
    custom=rules.get('selected_route_source')=='custom_existing_door27'
    if custom:episode=rules['custom_route']
    write_json(out/'preregistered.json',{'rules':rules,'protocol_sha256':digest('configs/research/blind_corner_protocol.json'),'base_config_sha256':digest(rules['base_config']),'official_episode':episode,**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'disable_viewport_updates':True});code=1
    try:
        import omni.usd,omni.timeline
        from pxr import UsdGeom
        from research.blind_corner_queries import Queries
        from research.audit_hospital_lights import inspect_stage
        timeline=omni.timeline.get_timeline_interface();timeline.stop();stage=omni.usd.get_context().get_stage()
        stage.GetRootLayer().subLayerPaths.append(cfg['scene']['usd']);UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        inventory=inspect_stage(stage)[-1]
        assert inventory==json.loads(Path('outputs/hospital-native-light-audit-20261010-01/collision_geometry.json').read_text())
        q=Queries(stage);candidates=[]
        variants=rules['custom_reference_radii_m'] if custom else rules['reference_lanes_x_m']
        for lane in variants:
            poses=reference_route(episode,lane,rules);dense=reference_route(episode,lane,rules,step=.1)
            route_trace=q.oriented_robot_path(dense)
            route_valid=all(r['floor'] and not r['overlap'] for r in route_trace)
            humans=rules['custom_human_positions'] if custom else rules.get('official_human_positions',[[lane,hy,0.] for hy in rules['human_y_m']])
            for human in humans:
                g=q.human(human,yaw=rules['human_yaw_rad']);b=q.bypass(human,rules['robot_radius_m'],heading=0. if custom else np.pi/2)
                valid=bool(route_valid and g['floor_ok'] and not g['overlap_paths'] and b['pass'])
                rays,groups=q.rays(human,poses,yaw=rules['human_yaw_rad']) if valid else ([],[]);possibilities=[]
                for i in range(3,len(rays)):
                    prior=rays[i-3:i];old=rays[i-1];fresh=rays[i]
                    # Approximate geometric proposal, never semantic HIDDEN/CLEAR.
                    hidden=all(not any(r['exposed']) and any(h is not None for h in r['hits']) for r in prior)
                    newly=sum(f and not o for o,f in zip(old['exposed'],fresh['exposed']))
                    if hidden and newly and poses[i,2]<poses[0,2]-1e-6:
                        possibilities.append({'fresh_reference_index':i,'new_samples':newly,'distance_m':float(np.linalg.norm(poses[i,:2]-human[:2])),'prior_occluders':sorted({h['prim'] for r in prior for h in r['hits'] if h})})
                candidate={'id':len(candidates)+1,'episode_id':'blind_door27' if custom else rules['official_episode_id'],'lane_x':lane,'variant_note':'turn radius' if custom else 'lane X','human_position':human,'human_yaw':rules['human_yaw_rad'],'geometry_pass':valid,'route_valid':route_valid,'route_trace':route_trace,'reference_poses':poses.tolist(),'geometry':g,'bypass':b,'rays':rays,'transition':possibilities[0] if possibilities else None}
                candidates.append(candidate)
        chosen=select_route(candidates)
        write_json(out/'candidates.json',candidates);write_json(out/'selected_route.json',chosen)
        write_json(out/'summary.json',{'candidate_segments':len(rules['reference_lanes_x_m']),'human_proposals':len(candidates),'geometry_valid':sum(c['geometry_pass'] for c in candidates),'ray_transition_possible':sum(c['transition'] is not None for c in candidates),'selected_id':chosen['id'] if chosen else None,'model_calls':0,'rgb_renders':0,'physics_steps':0})
        print('BLIND_ROUTE_SELECTION',json.dumps(json.loads((out/'summary.json').read_text())),flush=True);code=0
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
