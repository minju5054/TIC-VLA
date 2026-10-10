"""Pure saved-data gates and camera geometry for the single prescribed C26 cart."""
import numpy as np


def preflight_gate(physical, visibility, run):
    """C1 must have exactly zero pixels; useful CLEAR/application precedes entry."""
    if len(visibility)!=len(run['events']) or [v['request_id'] for v in visibility]!=list(range(1,len(visibility)+1)):
        raise ValueError('Complete ordered camera evidence required')
    first=next((v for v in visibility if v['state']=='CLEAR'),None)
    conflict=physical['baseline']['first_conflict']
    entry=conflict['sim_time'] if conflict else None
    app=run['events'][first['request_id']-1]['application']['sim_time'] if first else None
    useful=bool(first and entry is not None and first['sim_time']<entry and app<entry)
    reasons=[]
    if not physical['pass']:reasons.append('PHYSICAL_PLACEMENT_INVALID')
    if not physical['checks']['local_bypass']:reasons.append('NO_FEASIBLE_BYPASS')
    if visibility[0]['cart_visible_pixel_count']!=0:reasons.append('C1_CART_VISIBLE_INCLUDING_MARGINAL')
    if not useful:reasons.append('NO_USEFUL_CLEAR_REVEAL_BEFORE_BASELINE_ENTRY')
    return {'model_run_permitted':not reasons,'visibility_diagnostic_only':False,'stop_reasons':reasons,
            'baseline_region_entry_sim_time':entry,'first_clear_baseline_application_sim_time':app,
            'clear_to_baseline_entry_s':entry-first['sim_time'] if first and entry is not None else None,
            'useful_reveal_rule':'First CLEAR observation and its saved baseline application precede first baseline inflated-cart entry; C1 has zero cart pixels'}


def camera_projection(world_xyz, camera_world, camera, resolution=(1920,1080)):
    """USD row matrices: camera +X right,+Y up,-Z forward; world metres.

    Pixel mapping uses the authored perspective apertures. The render product
    may conform vertical aperture to its aspect ratio; caller must supply the
    effective aperture read from the render product when available.
    """
    xyz=np.asarray(world_xyz,float).reshape(-1,3)
    local=np.column_stack([xyz,np.ones(len(xyz))])@np.linalg.inv(camera_world)
    depth=-local[:,2]
    if camera.get('model')=='fisheyePolynomial':
        # NVIDIA Replicator writer uses theta = sum(a_i*r**i), in radians,
        # nominal sensor pixels, then scales each image axis to output size.
        coeff=camera['polynomial'];nominal=np.array(camera['nominal_resolution']);center=np.array(camera['optical_center'])
        upper=float(np.linalg.norm(np.maximum(center,nominal-center)))
        poly=np.polynomial.Polynomial(coeff)
        if np.min(poly.deriv()(np.linspace(0,upper,1024)))<=0:raise ValueError('Nonmonotone camera polynomial')
        radial=np.linalg.norm(local[:,:2],axis=1);theta=np.arctan2(radial,depth)
        low=np.zeros(len(xyz));high=np.full(len(xyz),upper)
        for _ in range(60):
            mid=(low+high)/2;below=poly(mid)<theta;low=np.where(below,mid,low);high=np.where(below,high,mid)
        direction=np.divide(local[:,:2],radial[:,None],out=np.zeros((len(xyz),2)),where=radial[:,None]>1e-12)
        uv=(center+(low+high)[:,None]/2*direction*[1,-1])*np.array(resolution)/nominal
        inside=(theta<=poly(upper))&(theta<=np.radians(camera['max_fov_deg']/2))&(depth>0)&(uv>=0).all(1)&(uv<resolution).all(1)
        return {'pixels':uv.tolist(),'depth_m':depth.tolist(),'inside_fov':inside.tolist()}
    if camera.get('model','pinhole')!='pinhole':raise ValueError('Unsupported original camera model')
    slopes=np.array([camera['horizontal_aperture'],camera['vertical_aperture']])/(2*camera['focal_length'])
    safe=np.where(np.abs(depth)>1e-12,depth,np.nan)
    uv=np.array(resolution)*(.5+local[:,:2]/(2*safe[:,None]*slopes)*[1,-1])
    near,far=camera.get('clipping_range',[0.,np.inf])
    inside=(depth>=near)&(depth<=far)&(uv>=0).all(1)&(uv<resolution).all(1)
    return {'pixels':uv.tolist(),'depth_m':depth.tolist(),'inside_fov':inside.tolist()}


def instruction_audit(runs):
    texts={name:r['cfg']['instruction'] for name,r in runs.items()}
    reference=next(iter(texts.values()))
    if any(t!=reference for t in texts.values()):raise ValueError('Runtime instructions differ')
    official={name:r['cfg']['official_episode']['episode']['instruction'] for name,r in runs.items()}
    if any(t!=reference for t in official.values()):raise ValueError('Official instruction differs')
    expected='Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.'
    if reference!=expected:raise ValueError('Unexpected E16 instruction: inspect meaning before reporting direction')
    return {'exact_text':reference,'runtime_instructions':texts,'all_equal':True,'official_equal':True,
            'right_on_obstacle_directive':False,
            'interpretation':'The right-side phrase locates the goal bed; the explicit turn is left to enter the hallway. No right-on-obstacle instruction.'}


def precontact(row, first_contact_time):
    """Both observation and application must precede contact; equality excluded."""
    return first_contact_time is None or max(row['fresh_observation_sim_time'],row['application_sim_time'])<first_contact_time
