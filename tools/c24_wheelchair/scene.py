"""Exact C10 human plus a static copy of the original Hospital wheelchair."""
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
    exact_config, TwoGroundedHumans, mesh_world_points, mesh_bounds, query_scene,
    grounding_views as human_grounding_views)

SOURCE = '/Root/SM_WheelChair_01a_37'
COPY = '/World/ExactWheelchairC24'
LABEL = 'stationary_wheelchair'


def read(path):
    return json.loads(Path(path).read_text())


def scenario_config(root, cfg, reference):
    cfg = exact_config(root, cfg, reference)
    c24 = cfg['fixed_humans'].pop()
    assert c24['id'] == 'C24'
    cfg['fixed_wheelchair'] = {'id':'C24', 'prim':COPY, 'source_prim':SOURCE,
        'position':c24['position'], 'reference_observation':c24['reference_observation'],
        'reference_tangent_rad':c24['reference_tangent_rad'], 'xy_offset_m':[0.,0.],
        'motion':'static original collision mesh; no rigid body or pose updates',
        'placement':'Actual visible mesh footprint centre at original C24 XY; lowest mesh vertex on floor. Longest local horizontal axis aligned with original adjacent-observation tangent.'}
    return cfg


def wheel_points(stage):
    from pxr import Usd, UsdGeom
    meshes=[]
    for prim in Usd.PrimRange(stage.GetPrimAtPath(COPY),Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):continue
        mesh=UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility()=='invisible':continue
        points=np.asarray(mesh.GetPointsAttr().Get())
        matrix=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
        meshes.append((np.column_stack([points,np.ones(len(points))])@matrix)[:,:3])
    if not meshes:raise ValueError('No visible wheelchair meshes, including instance proxies')
    return np.vstack(meshes)


def author_wheelchair(sim, cfg):
    import omni.client
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf
    from isaacsim.core.utils.semantics import add_labels
    stage=sim.stage; w=cfg['fixed_wheelchair']; before=signature(stage,SOURCE)
    if stage.GetPrimAtPath(COPY): raise ValueError('Existing wheelchair copy')
    root=UsdGeom.Xform.Define(stage,COPY)
    root.GetPrim().GetReferences().AddReference(cfg['scene']['usd'],SOURCE)
    root.ClearXformOpOrder(); op=root.AddTransformOp();op.Set(Gf.Matrix4d(1.))
    points=wheel_points(stage);lo,hi=points.min(0),points.max(0)
    long_axis=int(np.argmax((hi-lo)[:2]));yaw=w['reference_tangent_rad']-long_axis*np.pi/2
    matrix=np.eye(4);matrix[:2,:2]=rotation(yaw).T
    pivot=np.array([*(lo[:2]+hi[:2])/2,lo[2]])
    matrix[3,:3]=np.array(w['position'])-pivot@matrix[:3,:3]
    op.Set(Gf.Matrix4d(*matrix.ravel()))
    # An asymmetric rotated mesh can have a shifted world bounding-box centre.
    # Centre the actual world footprint exactly, retaining the selected yaw.
    rotated=wheel_points(stage)
    matrix[3,:2]+=np.array(w['position'][:2])-(rotated[:,:2].min(0)+rotated[:,:2].max(0))/2
    op.Set(Gf.Matrix4d(*matrix.ravel()))
    colliders=[]
    for p in Usd.PrimRange(root.GetPrim(),Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Wheelchair must remain static')
        if p.HasAPI(UsdPhysics.CollisionAPI):
            original=stage.GetPrimAtPath(str(p.GetPath()).replace(COPY,SOURCE,1))
            for attr in ('points','faceVertexCounts','faceVertexIndices','physics:collisionEnabled','physics:approximation'):
                if str(p.GetAttribute(attr).Get())!=str(original.GetAttribute(attr).Get()):raise ValueError('Wheelchair collision geometry changed')
            a=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            b=UsdShade.MaterialBindingAPI(original).ComputeBoundMaterial()[0]
            if str(a.GetPath()).replace(COPY,'ASSET')!=str(b.GetPath()).replace(SOURCE,'ASSET'):raise ValueError('Wheelchair materials changed')
            colliders.append(str(p.GetPath()))
    if not colliders:raise ValueError('No wheelchair collider')
    add_labels(root.GetPrim(),[LABEL])
    if signature(stage,SOURCE)!=before:raise ValueError('Original wheelchair changed')
    points=wheel_points(stage);low,high=points.min(0),points.max(0)
    centre=(low[:2]+high[:2])/2
    np.testing.assert_allclose(centre,w['position'][:2],rtol=0,atol=1e-9)
    np.testing.assert_allclose(low[2],0.,rtol=0,atol=1e-9)
    hashes={}
    for url in (cfg['scene']['usd'],cfg['scene']['usd'].rsplit('/',1)[0]+'/Props/SM_WheelChair_01a.usd'):
        status,_,data=omni.client.read_file(url)
        if status!=omni.client.Result.OK:raise ValueError('Asset bytes unavailable')
        hashes[url]=hashlib.sha256(bytes(data)).hexdigest()
    result={'copy_prim':COPY,'source_prim':SOURCE,'source_signature':before,
        'asset_sha256':hashes,'colliders':colliders,'world_bounds':[low.tolist(),high.tolist()],
        'local_bounds':[lo.tolist(),hi.tolist()],'footprint_half_m':((hi-lo)[:2]/2).tolist(),
        'center_world_xyz':[*centre,low[2]],'yaw_rad':float(yaw),'matrix_row_vector':matrix.tolist(),
        'source_unchanged':True,'coordinate_contract':'Hospital original world origin, metres,+Z up. Mesh/skeleton points mapped to world by USD row-vector transforms. Root rotation has yaw CCW from+X. Bounds centre maps to exact C24 XY; geometry lowest Z maps to floor0. No scaling.'}
    write_json(sim.records.path/'wheelchair_asset.json',result)
    return result


class MixedActors:
    def __init__(self,sim,cfg):
        from isaacsim.core.utils.semantics import add_labels
        self.sim,self.cfg=sim,cfg
        self.human=TwoGroundedHumans(sim,cfg)
        add_labels(sim.stage.GetPrimAtPath(cfg['fixed_humans'][0]['prim']),['occluded_human'])
        self.wheel=author_wheelchair(sim,cfg)
        self.file=(sim.records.path/'raw/wheelchair_state.csv').open('x')
        self.writer=csv.writer(self.file);self.writer.writerow(['tick','sim_time','x','y','z','yaw','matrix_max_error','motion_active'])

    def update(self,sim_time): pass

    def snapshot(self):
        from pxr import UsdGeom
        actual=np.array(UsdGeom.Xformable(self.sim.stage.GetPrimAtPath(COPY)).ComputeLocalToWorldTransform(0))
        error=float(np.abs(actual-self.wheel['matrix_row_vector']).max())
        if error>1e-12:raise ValueError('Static wheelchair moved')
        human=self.human.snapshot()
        return {**human,'object_type':'fixed_C10_human_and_C24_wheelchair',
            'wheelchair':{'position':self.wheel['center_world_xyz'],'yaw_rad':self.wheel['yaw_rad'],
            'matrix_max_error':error,'motion_active':False,'event_source':'verified USD static-collider world transform; not a measured dynamic rigid body',**clocks()},
            'legacy_field_note':'human_at_* contains one measured C10 human plus separately typed wheelchair'}

    def measure(self,sim_time):
        self.human.measure(sim_time);w=self.snapshot()['wheelchair']
        self.writer.writerow([self.sim.tick,sim_time,*w['position'],w['yaw_rad'],w['matrix_max_error'],False])

    def close(self):self.human.close();self.file.close()


class MixedSimulation(HospitalLightsSimulation):
    def capture(self,name):
        from research.nova_occlusion import segmentation_stats, visibility_state
        from PIL import Image
        rgb,obs,path=super().capture(name);packed=self.segmentation.get_data()
        folder=self.records.path/'raw/object_visibility';folder.mkdir(exist_ok=True)
        # Retain ID rasters and maps so identity matching can be audited later.
        np.save(folder/(name+'.ids.npy'),np.asarray(packed['data']),allow_pickle=False)
        labels=packed['info']['idToLabels'];rows={}
        for key,label in [('C10_human','occluded_human'),('C24_wheelchair',LABEL)]:
            stats,mask=segmentation_stats(packed['data'],labels,label)
            rows[key]={**stats,'state':visibility_state(stats,self.cfg['occlusion']['visibility_rule'])}
            Image.fromarray((mask*255).astype('uint8')).save(folder/(name+'.'+key+'.mask.png'))
        write_json(folder/(name+'.json'),{'idToLabels':labels,'objects':rows,'observation_sim_time':obs['sim_time'],'model_input':False})
        return rgb,obs,path


def wheelchair_gate(sim,cfg,receipt,query):
    w=cfg['fixed_wheelchair'];a=sim.pedestrian.wheel
    points=wheel_points(sim.stage);lo,hi=points.min(0),points.max(0)
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
        paths.append({'pass':bool(clearance>0 and all(r['floor'] and not r['overlap'] for r in trace)),
            'clearance_m':clearance,'vertices_world_xy':vertices.tolist(),'trace':trace})
    expected=receipt.get('wheelchair_expected')
    checks={'C24_human_absent':not sim.stage.GetPrimAtPath('/World/ExactHumanC24'),
        'wheelchair_source_unchanged':signature(sim.stage,SOURCE)==a['source_signature'],
        'wheelchair_preflight_identity':expected is None or a==expected,
        'wheelchair_exact_C24_xy':bool(np.allclose((lo[:2]+hi[:2])/2,w['reference_observation']['pose'][:2],rtol=0,atol=1e-9)),
        'wheelchair_grounded':bool(floor and floor['normal'][2]>.9 and abs(lo[2]-floor['position'][2])<.001),
        'wheelchair_no_hospital_overlap':not overlap,
        'wheelchair_static':sim.pedestrian.snapshot()['wheelchair']['matrix_max_error']==0.,
        'wheelchair_initial_robot_separation':bool(np.linalg.norm(np.array(sim.state()['position'][:2])-center)>radius+np.linalg.norm(half)),
        'wheelchair_local_bypass':any(p['pass'] for p in paths),
        'human_semantics_rendered':read(sim.records.path/'raw/object_visibility/preflight.png.json')['objects']['C10_human']['human_visible_pixel_count']>0}
    return {'checks':checks,'floor':floor,'mesh_min_z':float(lo[2]),'overlap_paths':overlap,'bypass_probes':paths}


def grounding_views(sim,cfg):
    # Reuse the established frozen-time diagnostic cameras for both objects.
    diagnostic=copy.deepcopy(cfg);diagnostic['fixed_humans'].append(cfg['fixed_wheelchair'])
    human_grounding_views(sim,diagnostic)
    # Independent camera sees the wheelchair and validates its single class label.
    import omni.replicator.core as rep
    from research.nova_occlusion import segmentation_stats
    before=sim.state();product=rep.create.render_product('/World/GroundingCamera_C24_body',(960,720))
    annotator=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False});annotator.attach([product])
    sim.world.render();rep.orchestrator.step(rt_subframes=4,delta_time=0.,pause_timeline=False,wait_for_render=True)
    packed=annotator.get_data();stats,_=segmentation_stats(packed['data'],packed['info']['idToLabels'],LABEL)
    write_json(sim.records.path/'wheelchair-semantic-preflight.json',{'stats':stats,'idToLabels':packed['info']['idToLabels'],'model_calls':0})
    if stats['human_visible_pixel_count']<100 or before['tick']!=sim.tick or before['sim_time']!=sim.state()['sim_time']:raise ValueError('Wheelchair rendered-label/frozen-time check failed')
    annotator.detach([product]);product.destroy()


def pre_model_gate(sim, cfg, receipt, baseline):
    import omni.client
    from pxr import UsdGeom, UsdPhysics
    from isaacsim.core.utils.semantics import get_labels
    before = sim.state(); _, obs, _ = sim.capture('preflight.png')
    robot = read(sim.records.path/'robot_asset.json'); old_robot = read(baseline/'robot_asset.json')
    lights = read(sim.records.path/'hospital_lighting_applied.json'); old_lights = read(baseline/'hospital_lighting_applied.json')
    checks = {'one_C10_person': [h['id'] for h in cfg['fixed_humans']] == ['C10'],
        'no_legacy_human_or_cart': not sim.stage.GetPrimAtPath('/World/PreflightHuman') and not sim.stage.GetPrimAtPath('/World/E16Cart') and not sim.stage.GetPrimAtPath('/World/ExactHumanC24'),
        'robot_camera_wheels': all(robot[k] == old_robot[k] for k in ('cameras','camera_body_transform','verified_drive_wheels','verified_wheel_base','controller_radius','controller_base')),
        'current_robot_source': robot['source'] == receipt['changes']['current_robot_source'],
        'spawn': read(sim.records.path/'nova_spawn_audit.json')['placement_valid'],
        'lighting': {k:v for k,v in lights.items() if not k.startswith('collision_')} == {k:v for k,v in old_lights.items() if not k.startswith('collision_')},
        'lighting_preserves_colliders': lights['collision_inventory_unchanged'],
        'completed_rgb_no_physics': before['tick'] == obs['tick'] == 0 and before['position'] == obs['position'],
        'initial_brightness': read(sim.records.path/'raw/brightness/preflight.png.json')['pass']}
    query = query_scene(); measured = sim.pedestrian.snapshot()['humans']; details = {}
    radius = cfg['spatial_turn_gate']['robot_conservative_radius_m']
    for h in cfg['fixed_humans']:
        root = sim.stage.GetPrimAtPath(h['prim']); matrix = np.array(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0))
        capsule = UsdGeom.Capsule(sim.stage.GetPrimAtPath(h['prim']+'/Collider'))
        body = UsdPhysics.RigidBodyAPI(root); current = mesh_bounds(mesh_world_points(sim.stage,h['prim']))
        floor_hits=[]
        def floor_hit(hit):
            # A ray starting inside the human capsule can hit that capsule at
            # distance zero. Only original Hospital colliders establish floor.
            if str(hit.collision).startswith('/Root/'):
                floor_hits.append({'prim':str(hit.collision),'distance_m':float(hit.distance),
                    'position':list(map(float,hit.position)),'normal':list(map(float,hit.normal))})
            return True
        query.q.raycast_all(query.carb.Float3(*h['position'][:2], .3), query.carb.Float3(0,0,-1), .6, floor_hit, True)
        floor=min(floor_hits,key=lambda r:r['distance_m']) if floor_hits else None
        floor_z = floor['position'][2] if floor else None
        cap_matrix=np.array(UsdGeom.Xformable(capsule).ComputeLocalToWorldTransform(0))
        capsule_bottom=float(cap_matrix[3,2]-capsule.GetHeightAttr().Get()/2-capsule.GetRadiusAttr().Get())
        source, _, data = omni.client.read_file(h['asset'])
        asset_sha = hashlib.sha256(bytes(data)).hexdigest() if source == omni.client.Result.OK else None
        lo,hi = np.array(current['visible_min_world']),np.array(current['visible_max_world'])
        overlaps = query.overlap(((lo+hi)/2).tolist(), extent=[*(hi[:2]-lo[:2])/2, (hi[2]-lo[2])/2-.001])
        paths=[]; center=np.array(h['position'][:2]); yaw=h['reference_tangent_rad']
        for side in (-1,1):
            off=side*(radius+h['radius_m']+.05)
            vertices=np.array([[-1.8,0],[-1.35,off],[1.35,off],[1.8,0]])@rotation(yaw).T+center
            points=np.vstack([a+np.linspace(0,1,46)[:,None]*(b-a) for a,b in zip(vertices[:-1],vertices[1:])])
            trace=query.robot_path(points,radius+.05)
            actor_clear = min(float(np.linalg.norm(points-np.array(other['position'][:2]),axis=1).min()-radius-other['radius_m']) for other in cfg['fixed_humans'])
            paths.append({'pass':bool(actor_clear>0 and all(r['floor'] and not r['overlap'] for r in trace)),
                'vertices_world_xy':vertices.tolist(),'minimum_person_proxy_clearance_m':actor_clear,'trace':trace})
        per={'source_hash':asset_sha==receipt['human_source_sha256'],
            'authored_exact_reference_xy':matrix[3,:2].tolist()==h['reference_observation']['pose'][:2],
            'measured_exact_xy_within_1um':bool(np.allclose(measured[h['id']]['position'][:2],h['reference_observation']['pose'][:2],rtol=0,atol=1e-6)),
            'root_zero_yaw':bool(np.allclose(matrix[:3,:3],np.eye(3),atol=1e-9,rtol=0)),
            'kinematic':bool(body.GetKinematicEnabledAttr().Get()),
            'capsule_size':capsule.GetRadiusAttr().Get()==.25 and capsule.GetHeightAttr().Get()==1.2 and capsule.GetAxisAttr().Get()=='Z',
            'soles_grounded_within_1mm':bool(floor_z is not None and max(abs(z-floor_z) for z in current['sole_minima_world_z']) < .001),
            'capsule_grounded_within_1mm':bool(floor_z is not None and abs(capsule_bottom-floor_z)<.001),
            'floor_support':bool(floor and floor['normal'][2]>.9 and abs(floor_z)<.001),
            'no_visual_structure_overlap':not overlaps,
            'initial_robot_separation':bool(np.linalg.norm(np.array(obs['position'][:2])-center)>radius+h['radius_m']),
            'semantic_labels':set(get_labels(root).get('class',[]))=={'occluded_human'},
            'local_bypass_exists':any(p['pass'] for p in paths)}
        checks.update({h['id']+'_'+k:v for k,v in per.items()})
        details[h['id']]={'checks':per,'current_mesh_bounds':current,'floor_z_m':floor_z,'measured':measured[h['id']],
            'floor_rays':floor_hits,'capsule_bottom_world_m':capsule_bottom,
            'authored_matrix_row_vector':matrix.tolist(),'overlap_paths':overlaps,'bypass_probes':paths}
    wheelchair = wheelchair_gate(sim, cfg, receipt, query)
    checks.update(wheelchair['checks'])
    record={'wheelchair':wheelchair,'pass':all(checks.values()),'checks':checks,'people':details,'model_calls':0,
        'grounding_tolerance_m':.001,'measured_XY_tolerance_m':1e-6,
        'note':'XY authored exactly from original no-added-obstacle C10/C24; only visual/collider vertical placement corrected. Local bypass probes never command the robot.'}
    write_json(sim.records.path/'pre-model-gate.json',record)
    if not record['pass']:raise RuntimeError('MIXED_ACTORS_GATE_FAILED: '+str([k for k,v in checks.items() if not v]))

