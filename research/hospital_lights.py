"""Scene-derived Hospital lighting overrides; no trajectory/model input."""
import copy
import numpy as np


def native_profile(audit_lights, renderer, multiplier):
    selected=[l for l in audit_lights if l['type'] not in ['DomeLight','DistantLight']]
    return {'lighting_mode':'hospital_native_lights','native_multiplier':float(multiplier),
        'native_lights':[{'path':l['path'],'type':l['type'],'base_intensity':l['attributes']['inputs:intensity']} for l in selected],
        'fallback_fixture_lights':[],'renderer_settings':copy.deepcopy(renderer),
        'camera_exposure_unchanged':True,'policy':'All authored indoor lights uniformly enabled/scaled; original DomeLight unchanged'}


def fixture_sources(fixtures):
    # Exact scene asset families found by audit. No camera/robot/route argument.
    return sorted([f for f in fixtures if '/Geo_M_Light2_low/' in f['mesh'] or '/Light_test/Light_test' in f['mesh']],key=lambda f:f['mesh'])


def apply_hospital_lights(stage,profile):
    import carb
    from pxr import UsdGeom,UsdLux,UsdPhysics,Gf
    settings=carb.settings.get_settings()
    for k,v in profile['renderer_settings'].items():
        if v is not None:settings.set(k,v)
    previous_edit_target=stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    records=[]
    for item in profile['native_lights']:
        p=stage.GetPrimAtPath(item['path'])
        if not p or not p.HasAPI(UsdLux.LightAPI):raise ValueError('Audited native light missing')
        p.SetActive(True);UsdGeom.Imageable(p).MakeVisible()
        UsdLux.LightAPI(p).GetIntensityAttr().Set(item['base_intensity']*profile['native_multiplier'])
        records.append({'path':item['path'],'intensity':UsdLux.LightAPI(p).GetIntensityAttr().Get()})
    for item in profile['fallback_fixture_lights']:
        fixture=stage.GetPrimAtPath(item['source_mesh'])
        if not fixture or not fixture.IsA(UsdGeom.Mesh):raise ValueError('Source fixture missing')
        actual=np.array(UsdGeom.Xformable(fixture).ComputeLocalToWorldTransform(0))
        if not np.allclose(actual,item['fixture_world_matrix_row_vector'],atol=1e-9,rtol=0):raise ValueError('Fixture transform changed')
        cls=UsdLux.DiskLight if item['type']=='DiskLight' else UsdLux.RectLight
        light=cls.Define(stage,item['path'])
        if item['type']=='DiskLight':light.CreateRadiusAttr(item['radius_m'])
        else:light.CreateWidthAttr(item['width_m']);light.CreateHeightAttr(item['height_m'])
        light.CreateIntensityAttr(item['intensity']);light.CreateExposureAttr(0.)
        light.CreateColorAttr(Gf.Vec3f(1,1,1));light.CreateNormalizeAttr(False)
        x=UsdGeom.Xformable(light);x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(item['light_world_matrix_row_vector']).reshape(-1)))
        if light.GetPrim().HasAPI(UsdPhysics.CollisionAPI) or light.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Light has physics API')
        records.append({'path':str(light.GetPath()),'source_mesh':item['source_mesh'],'intensity':light.GetIntensityAttr().Get(),
            'collision_api':False,'rigid_body_api':False,'world_matrix_row_vector':np.array(x.ComputeLocalToWorldTransform(0)).tolist()})
    stage.SetEditTarget(previous_edit_target)
    return {'lights':records,'renderer_readback':{k:settings.get(k) for k in profile['renderer_settings']},'edit_layer':'anonymous session layer; original layers never saved'}
