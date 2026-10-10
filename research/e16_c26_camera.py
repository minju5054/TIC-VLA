"""Static camera/frustum and collision-ray audit; reuses archived C26 RGB/masks."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.e16_cart_preflight import BASELINE
from research.e16_c26_tools import camera_projection
from research.cart_geometry import rotation
from research.reveal_window import evidence_hashes,verify_hashes,validate_destination
from research.records import write_json,provenance


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--preflight-dir',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
    source=Path(a.preflight_dir).resolve();out=validate_destination(a.output_dir,[source,BASELINE]);out.mkdir(parents=True)
    frozen=json.loads((source/'freeze.json').read_text());physical=json.loads((source/'physical.json').read_text());vis=json.loads((source/'visibility.json').read_text())
    hashes=evidence_hashes([source,BASELINE]);cfg=json.loads((BASELINE/'metadata.json').read_text())['config']
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'disable_viewport_updates':True});code=1
    try:
        import omni.usd,omni.timeline
        from pxr import UsdGeom
        from research.hospital_cart import author
        from research.blind_corner_queries import Queries
        stage=omni.usd.get_context().get_stage();stage.GetRootLayer().subLayerPaths.append(cfg['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        robot=UsdGeom.Xform.Define(stage,cfg['robot']['prim']);robot.GetPrim().GetReferences().AddReference(cfg['robot']['asset'])
        camera=stage.GetPrimAtPath(cfg['camera']['prim']);attrs={a.GetName():str(a.Get()) for a in camera.GetAttributes()}
        def get(name):return camera.GetAttribute(name).Get()
        params={'model':get('cameraProjectionType') or 'pinhole','focal_length':get('focalLength'),
            'horizontal_aperture':get('horizontalAperture'),'vertical_aperture':get('verticalAperture'),'clipping_range':list(get('clippingRange'))}
        if params['model']=='fisheyePolynomial':
            params.update(nominal_resolution=[get('fthetaWidth'),get('fthetaHeight')],optical_center=[get('fthetaCx'),get('fthetaCy')],
                max_fov_deg=get('fthetaMaxFov'),polynomial=[get('fthetaPoly'+c) or 0. for c in 'ABCDEF'])
            poly=np.polynomial.Polynomial(params['polynomial']);cx=params['optical_center'][0];width=params['nominal_resolution'][0]
            hfov=float(np.degrees(poly(cx)+poly(width-cx)))
        else:hfov=float(np.degrees(2*np.arctan(params['horizontal_aperture']/(2*params['focal_length']))))
        # The copied cart participates in static rays. No World/reset/step/model.
        author(stage,frozen);query=Queries(stage)
        cart=frozen['cart'];center=np.array(cart['center_world_xyz']);r=rotation(cart['yaw_rad']);h=np.array(cart['footprint_half_m'])
        groups={}
        local=np.array([[x,y,z] for x in np.linspace(-h[0],h[0],5) for y in np.linspace(-h[1],h[1],3) for z in [.10,.55,1.0]])
        points=local.copy();points[:,:2]=local[:,:2]@r.T+center[:2];points[:,2]+=center[2];groups['cart']=points
        for side in ['north','south']:
            rows=[q for q in physical['bypass_checks'] if q['world_side']==side];chosen=next((q for q in rows if q['pass']),rows[0])
            xy=np.array([q['xy'] for q in chosen['trace']])[::5]
            groups[side]=np.array([[*p,z] for p in xy for z in [.1,.45,.9]])
        results=[]
        for v in vis:
            matrix=np.array(v['camera_world_matrix_row_vector']);origin=matrix[3,:3];row={'request_id':v['request_id'],'state':v['state'],'groups':{}}
            for name,points in groups.items():
                projected=camera_projection(points,matrix,params);hits=[]
                for point in points:
                    delta=point-origin;length=np.linalg.norm(delta);found=[]
                    def cb(hit):
                        path=str(hit.collision)
                        # Ignore display robot; retain all original Hospital and new cart surfaces.
                        if (path.startswith('/Root/') or path.startswith('/World/E16Cart')) and hit.distance<length-1e-5:
                            found.append({'prim':path,'distance_m':float(hit.distance)})
                        return True
                    query.q.raycast_all(query.carb.Float3(*map(float,origin)),query.carb.Float3(*map(float,delta/length)),float(length),cb,True)
                    hits.append(min(found,key=lambda h:h['distance_m']) if found else None)
                clear=[bool(inside and hit is None) for inside,hit in zip(projected['inside_fov'],hits)]
                row['groups'][name]={**projected,'world_xyz':points.tolist(),'first_ray_hits':hits,'unoccluded_inside_fov':clear,
                    'inside_count':sum(projected['inside_fov']),'unoccluded_count':sum(clear),'sample_count':len(points)}
            results.append(row)
        write_json(out/'camera.json',{'parameters':params,'horizontal_fov_deg':hfov,'raw_camera_attributes':attrs,
            'camera_body_transform':json.loads((BASELINE/'robot_asset.json').read_text())['camera_body_transform'],
            'convention':'USD row-vector world matrix; camera +X right,+Y up,-Z forward; polynomial theta in radians; nominal image scaled to 1920x1080',
            'limitations':'Rays test collision meshes at sampled passage heights, not full rendered optical surfaces or traversability. Semantic cart masks remain the visibility oracle.',
            'model_calls':0,'navigation_physics_reexecuted':False,'new_rgb_renders':0})
        write_json(out/'projections.json',results);verify_hashes(hashes)
        assert not omni.timeline.get_timeline_interface().is_playing()
        write_json(out/'metadata.json',{'source_evidence_sha256':hashes,'source_files_unchanged':True,**provenance(ROOT)})
        print('CAMERA_AUDIT_COMPLETE',params['model'],hfov,flush=True);code=0
    finally:app.close(exit_code=code)


if __name__=='__main__':main()
