"""Two static original Hospital wheelchairs at exact C10/C24; lower forward limit."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from research.records import write_json, clocks
from research.hospital_lights_simulation import HospitalLightsSimulation
from research.hospital_cart import signature
from research.cart_geometry import rotation
from tools.exact_two_humans.scene import (
    exact_config, query_scene,
    grounding_views as human_grounding_views)

SOURCE = '/Root/SM_WheelChair_01a_37'
IDS = ('C10', 'C24')
MAX_V = 1.0
PREDICTIONS = 72


def read(path):
    return json.loads(Path(path).read_text())


INSTRUCTION = ('Move forward toward the staircase, then turn left to enter the hallway. '
               'Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.')


def scenario_config(root, cfg, reference):
    if cfg['instruction'] != INSTRUCTION:
        raise ValueError('Original instruction required')
    if any(k in cfg for k in ('fixed_humans', 'fixed_wheelchair', 'fixed_wheelchairs', 'stationary_cart', 'pedestrian')):
        raise ValueError('Unexpected additional actor')
    original = read(root/reference/'metadata.json')['config']
    if any(k in original for k in ('stationary_human', 'stationary_cart', 'pedestrian', 'fixed_humans', 'fixed_wheelchair', 'fixed_wheelchairs')):
        raise ValueError('The coordinate reference must have no added obstacle')
    cfg = exact_config(root, cfg, reference)
    humans = cfg.pop('fixed_humans')
    cfg['fixed_wheelchairs'] = [
        {'id': h['id'], 'prim': '/World/ExactWheelchair'+h['id'], 'source_prim': SOURCE,
         'label': 'stationary_wheelchair_'+h['id'].lower(), 'position': h['position'],
         'reference_run': str(reference), 'reference_request_id': h['reference_request_id'],
         'reference_observation': h['reference_observation'],
         'reference_tangent_rad': h['reference_tangent_rad'], 'xy_offset_m': [0., 0.],
         'motion': 'static original collision mesh; no rigid body or pose updates',
         'placement': 'Actual visible mesh bounds centre at exact original reference XY; lowest vertex at floor; long axis parallel to adjacent-observation tangent.'}
        for h in humans]
    if cfg['controller']['max_v'] != 1.5 or cfg['controller']['max_w'] != 1.2 or cfg['simulation']['predictions'] != 48:
        raise ValueError('Expected original 1.5 m/s, 1.2 rad/s, 48-call baseline before override')
    cfg['controller']['max_v'] = MAX_V
    cfg['simulation']['predictions'] = PREDICTIONS
    return cfg


def wheel_points(stage, copy_path):
    from pxr import Usd, UsdGeom
    meshes=[]
    for prim in Usd.PrimRange(stage.GetPrimAtPath(copy_path),Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):continue
        mesh=UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility()=='invisible':continue
        points=np.asarray(mesh.GetPointsAttr().Get())
        matrix=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
        meshes.append((np.column_stack([points,np.ones(len(points))])@matrix)[:,:3])
    if not meshes:raise ValueError('No visible wheelchair meshes, including instance proxies')
    return np.vstack(meshes)


def author_wheelchair(sim, cfg, w):
    import omni.client
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf
    from isaacsim.core.utils.semantics import add_labels
    stage=sim.stage; copy_path=w['prim']; before=signature(stage,SOURCE)
    if stage.GetPrimAtPath(copy_path): raise ValueError('Existing wheelchair copy')
    root=UsdGeom.Xform.Define(stage,copy_path)
    root.GetPrim().GetReferences().AddReference(cfg['scene']['usd'],SOURCE)
    root.ClearXformOpOrder(); op=root.AddTransformOp();op.Set(Gf.Matrix4d(1.))
    points=wheel_points(stage, copy_path);lo,hi=points.min(0),points.max(0)
    long_axis=int(np.argmax((hi-lo)[:2]));yaw=w['reference_tangent_rad']-long_axis*np.pi/2
    matrix=np.eye(4);matrix[:2,:2]=rotation(yaw).T
    pivot=np.array([*(lo[:2]+hi[:2])/2,lo[2]])
    matrix[3,:3]=np.array(w['position'])-pivot@matrix[:3,:3]
    op.Set(Gf.Matrix4d(*matrix.ravel()))
    # An asymmetric rotated mesh can have a shifted world bounding-box centre.
    # Centre the actual world footprint exactly, retaining the selected yaw.
    rotated=wheel_points(stage, copy_path)
    matrix[3,:2]+=np.array(w['position'][:2])-(rotated[:,:2].min(0)+rotated[:,:2].max(0))/2
    op.Set(Gf.Matrix4d(*matrix.ravel()))
    colliders=[]
    for p in Usd.PrimRange(root.GetPrim(),Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Wheelchair must remain static')
        if p.HasAPI(UsdPhysics.CollisionAPI):
            original=stage.GetPrimAtPath(str(p.GetPath()).replace(copy_path,SOURCE,1))
            for attr in ('points','faceVertexCounts','faceVertexIndices','physics:collisionEnabled','physics:approximation'):
                if str(p.GetAttribute(attr).Get())!=str(original.GetAttribute(attr).Get()):raise ValueError('Wheelchair collision geometry changed')
            a=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            b=UsdShade.MaterialBindingAPI(original).ComputeBoundMaterial()[0]
            if str(a.GetPath()).replace(copy_path,'ASSET')!=str(b.GetPath()).replace(SOURCE,'ASSET'):raise ValueError('Wheelchair materials changed')
            colliders.append(str(p.GetPath()))
    if not colliders:raise ValueError('No wheelchair collider')
    add_labels(root.GetPrim(),[w['label']])
    if signature(stage,SOURCE)!=before:raise ValueError('Original wheelchair changed')
    points=wheel_points(stage, copy_path);low,high=points.min(0),points.max(0)
    centre=(low[:2]+high[:2])/2
    np.testing.assert_allclose(centre,w['position'][:2],rtol=0,atol=1e-9)
    np.testing.assert_allclose(low[2],0.,rtol=0,atol=1e-9)
    hashes={}
    for url in (cfg['scene']['usd'],cfg['scene']['usd'].rsplit('/',1)[0]+'/Props/SM_WheelChair_01a.usd'):
        status,_,data=omni.client.read_file(url)
        if status!=omni.client.Result.OK:raise ValueError('Asset bytes unavailable')
        hashes[url]=hashlib.sha256(bytes(data)).hexdigest()
    result={'copy_prim':copy_path,'source_prim':SOURCE,'source_signature':before,
        'asset_sha256':hashes,'colliders':colliders,'world_bounds':[low.tolist(),high.tolist()],
        'local_bounds':[lo.tolist(),hi.tolist()],'footprint_half_m':((hi-lo)[:2]/2).tolist(),
        'center_world_xyz':[*centre,low[2]],'yaw_rad':float(yaw),'matrix_row_vector':matrix.tolist(),
        'source_unchanged':True,'coordinate_contract':'Hospital original world origin, metres,+Z up. Mesh/skeleton points mapped to world by USD row-vector transforms. Root rotation has yaw CCW from+X. Bounds centre maps to the specified exact reference XY; geometry lowest Z maps to floor0. No scaling.'}
    write_json(sim.records.path/(w['id']+'-wheelchair_asset.json'),result)
    return result


class WheelchairActor:
    def __init__(self, sim, cfg):
        self.sim,self.cfg=sim,cfg
        if any(k in cfg for k in ('fixed_humans','stationary_human','pedestrian','fixed_wheelchair')):
            raise ValueError('Two-wheelchair scene must contain no people or legacy actor')
        self.wheels={w['id']:author_wheelchair(sim,cfg,w) for w in cfg['fixed_wheelchairs']}
        write_json(sim.records.path/'wheelchairs_asset.json',self.wheels)
        self.file=(sim.records.path/'raw/wheelchairs_state.csv').open('x')
        self.writer=csv.writer(self.file)
        self.writer.writerow(['tick','sim_time','id','x','y','z','yaw','matrix_max_error','motion_active'])

    def update(self,sim_time): pass

    def snapshot(self):
        from pxr import UsdGeom
        wheels={}
        for name,w in self.wheels.items():
            actual=np.array(UsdGeom.Xformable(self.sim.stage.GetPrimAtPath(w['copy_prim'])).ComputeLocalToWorldTransform(0))
            error=float(np.abs(actual-w['matrix_row_vector']).max())
            if error>1e-12:raise ValueError('Static wheelchair moved: '+name)
            wheels[name]={'position':w['center_world_xyz'],'yaw_rad':w['yaw_rad'],
                'matrix_max_error':error,'motion_active':False,**clocks()}
        return {'humans':{},'object_type':'fixed_C10_C24_wheelchairs_only','wheelchairs':wheels,
            'event_source':'Verified USD static-collider world transforms; not dynamic PhysX body poses',
            'legacy_field_note':'human_at_* is a legacy log key: zero humans, two static wheelchairs'}

    def measure(self,sim_time):
        for name,w in self.snapshot()['wheelchairs'].items():
            self.writer.writerow([self.sim.tick,sim_time,name,*w['position'],w['yaw_rad'],w['matrix_max_error'],False])

    def close(self):self.file.close()


class WheelchairSimulation(HospitalLightsSimulation):
    def capture(self,name):
        from research.nova_occlusion import segmentation_stats, visibility_state
        from PIL import Image
        rgb,obs,path=super().capture(name);packed=self.segmentation.get_data()
        folder=self.records.path/'raw/object_visibility';folder.mkdir(exist_ok=True)
        # Retain ID rasters and maps so identity matching can be audited later.
        np.save(folder/(name+'.ids.npy'),np.asarray(packed['data']),allow_pickle=False)
        labels=packed['info']['idToLabels'];rows={}
        for w in self.cfg['fixed_wheelchairs']:
            key,label=w['id']+'_wheelchair',w['label']
            stats,mask=segmentation_stats(packed['data'],labels,label)
            rows[key]={**stats,'state':visibility_state(stats,self.cfg['occlusion']['visibility_rule'])}
            Image.fromarray((mask*255).astype('uint8')).save(folder/(name+'.'+key+'.mask.png'))
        write_json(folder/(name+'.json'),{'idToLabels':labels,'objects':rows,'observation_sim_time':obs['sim_time'],'model_input':False})
        return rgb,obs,path


def wheelchair_gate(sim,cfg,receipt,query,w):
    a=sim.pedestrian.wheels[w['id']]
    points=wheel_points(sim.stage,w['prim']);lo,hi=points.min(0),points.max(0)
    hits=[]
    def floor_hit(hit):
        if str(hit.collision).startswith('/Root/'):
            hits.append({'prim':str(hit.collision),'position':list(map(float,hit.position)),'distance_m':float(hit.distance),'normal':list(map(float,hit.normal))})
        return True
    query.q.raycast_all(query.carb.Float3(*w['position'][:2],.3),query.carb.Float3(0,0,-1),.6,floor_hit,True)
    floor=min(hits,key=lambda h:h['distance_m']) if hits else None
    radius=cfg['spatial_turn_gate']['robot_conservative_radius_m'];half=np.array(a['footprint_half_m'])
    overlap=query.overlap([*w['position'][:2],(lo[2]+hi[2])/2+.002],extent=[*half,(hi[2]-lo[2])/2-.002],yaw=a['yaw_rad'])
    paths=[];tangent=w['reference_tangent_rad'];center=np.array(w['position'][:2])
    for side in (-1,1):
        off=side*(min(half)+radius+.10)
        vertices=np.array([[-2.1,0],[-1.7,off],[1.7,off],[2.1,0]])@rotation(tangent).T+center
        xy=np.vstack([u+np.linspace(0,1,55)[:,None]*(v-u) for u,v in zip(vertices[:-1],vertices[1:])])
        local=(xy-center)@rotation(a['yaw_rad']);delta=np.maximum(np.abs(local)-half,0.)
        clearance=float(np.linalg.norm(delta,axis=1).min()-radius)
        trace=query.robot_path(xy,radius+.05)
        other_clear=[]
        for other in sim.pedestrian.wheels.values():
            local_other=(xy-np.array(other['center_world_xyz'][:2]))@rotation(other['yaw_rad'])
            gap=np.maximum(np.abs(local_other)-other['footprint_half_m'],0.)
            other_clear.append(float(np.linalg.norm(gap,axis=1).min()-radius))
        paths.append({'pass':bool(min(other_clear)>0 and all(r['floor'] and not r['overlap'] for r in trace)),
            'clearance_m':min(other_clear),'vertices_world_xy':vertices.tolist(),'trace':trace})
    expected=(receipt.get('wheelchairs_expected') or {}).get(w['id'])
    checks={
        'wheelchair_source_unchanged':signature(sim.stage,SOURCE)==a['source_signature'],
        'wheelchair_preflight_identity':expected is None or a==expected,
        'wheelchair_exact_reference_xy':bool(np.allclose((lo[:2]+hi[:2])/2,w['reference_observation']['pose'][:2],rtol=0,atol=1e-9)),
        'wheelchair_grounded':bool(floor and floor['normal'][2]>.9 and abs(lo[2]-floor['position'][2])<.001),
        'wheelchair_no_hospital_overlap':not overlap,
        'wheelchair_static':sim.pedestrian.snapshot()['wheelchairs'][w['id']]['matrix_max_error']==0.,
        'wheelchair_initial_robot_separation':bool(np.linalg.norm(np.array(sim.state()['position'][:2])-center)>radius+np.linalg.norm(half)),
        'wheelchair_local_bypass':any(p['pass'] for p in paths)}
    return {'checks':checks,'floor':floor,'mesh_min_z':float(lo[2]),'overlap_paths':overlap,'bypass_probes':paths}


def grounding_views(sim,cfg):
    # Existing camera helper only reads target positions; it creates no actors.
    diagnostic=copy.deepcopy(cfg);diagnostic['fixed_humans']=cfg['fixed_wheelchairs']
    human_grounding_views(sim,diagnostic)
    import omni.replicator.core as rep
    from research.nova_occlusion import segmentation_stats
    results={}
    for w in cfg['fixed_wheelchairs']:
        before=sim.state();product=rep.create.render_product('/World/GroundingCamera_'+w['id']+'_body',(960,720))
        annotator=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False});annotator.attach([product])
        sim.world.render();rep.orchestrator.step(rt_subframes=4,delta_time=0.,pause_timeline=False,wait_for_render=True)
        packed=annotator.get_data();stats,_=segmentation_stats(packed['data'],packed['info']['idToLabels'],w['label'])
        results[w['id']]={'stats':stats,'idToLabels':packed['info']['idToLabels'],'model_calls':0}
        if stats['human_visible_pixel_count']<100 or before['tick']!=sim.tick or before['sim_time']!=sim.state()['sim_time']:
            raise ValueError('Wheelchair rendered-label/frozen-time check failed: '+w['id'])
        annotator.detach([product]);product.destroy()
    write_json(sim.records.path/'wheelchair-semantic-preflight.json',results)


def pre_model_gate(sim, cfg, receipt, baseline):
    before=sim.state(); _,obs,_=sim.capture('preflight.png')
    robot=read(sim.records.path/'robot_asset.json');old_robot=read(baseline/'robot_asset.json')
    lights=read(sim.records.path/'hospital_lighting_applied.json');old_lights=read(baseline/'hospital_lighting_applied.json')
    forbidden=('/World/PreflightHuman','/World/ExactHumanC10','/World/ExactHumanC24',
               '/World/ExactHumanC30','/World/E16Cart')
    checks={'wheelchair_only_config':[w['id'] for w in cfg['fixed_wheelchairs']]==list(IDS) and not any(k in cfg for k in ('fixed_humans','stationary_human','pedestrian','stationary_cart')),
        'no_human_or_old_obstacle_prims':all(not sim.stage.GetPrimAtPath(p) for p in forbidden),
        'original_instruction':cfg['instruction']==INSTRUCTION,
        'lower_speed':cfg['controller']['max_v']==MAX_V and cfg['controller']['max_w']==1.2,
        'extended_budget':cfg['simulation']['predictions']==PREDICTIONS,
        'robot_camera_wheels':all(robot[k]==old_robot[k] for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'current_robot_source':robot['source']==receipt['changes']['current_robot_source'],
        'spawn':read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting':{k:v for k,v in lights.items() if not k.startswith('collision_')}=={k:v for k,v in old_lights.items() if not k.startswith('collision_')},
        'lighting_preserves_colliders':lights['collision_inventory_unchanged'],
        'completed_rgb_no_physics':before['tick']==obs['tick']==0 and before['position']==obs['position'],
        'initial_brightness':read(sim.records.path/'raw/brightness/preflight.png.json')['pass']}
    query=query_scene();wheelchair={}
    for w in cfg['fixed_wheelchairs']:
        details=wheelchair_gate(sim,cfg,receipt,query,w);wheelchair[w['id']]=details
        checks.update({w['id']+'_'+k:v for k,v in details['checks'].items()})
    a,b=sim.pedestrian.wheels.values()
    checks['added_wheelchairs_separate']=bool(np.linalg.norm(np.array(a['center_world_xyz'][:2])-b['center_world_xyz'][:2])>np.linalg.norm(a['footprint_half_m'])+np.linalg.norm(b['footprint_half_m']))
    checks['C10_initial_semantics']=read(sim.records.path/'raw/object_visibility/preflight.png.json')['objects']['C10_wheelchair']['human_visible_pixel_count']>0
    record={'pass':all(checks.values()),'checks':checks,'people':{},'wheelchairs':wheelchair,'model_calls':0,
        'grounding_tolerance_m':.001,'exact_xy_tolerance_m':1e-9,
        'note':'Two static wheelchairs at original C10/C24 XY. No people. Diagnostic bypass probes never command the robot.'}
    write_json(sim.records.path/'pre-model-gate.json',record)
    if not record['pass']:raise RuntimeError('TWO_WHEELCHAIRS_SLOW_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))
