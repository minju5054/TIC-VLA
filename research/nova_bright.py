"""Fixed renderer profile and pure image audits for Bright Hospital."""
import copy
import numpy as np

# Preregistered before calibration/model calls. All luminance uses Rec.709 RGB.
BRIGHT_RULES = dict(min_mean=40., max_mean=215., min_std=15., min_p50=25.,
                    min_p95=100., max_dark_fraction=.35, max_saturated_fraction=.05,
                    dark_cutoff=15., saturated_cutoff=250.)
EXPOSURE_STOPS = [2, 4, 6, 8]


def image_stats(rgb):
    a=np.asarray(rgb)
    if a.ndim!=3 or a.shape[2]!=3 or not np.isfinite(a).all():raise ValueError('Finite RGB required')
    y=a.astype(float)@np.array([.2126,.7152,.0722])
    return {'mean':float(y.mean()),'std':float(y.std()),
            **{f'p{q:02d}':float(np.percentile(y,q)) for q in [5,50,95,99]},
            'dark_pixel_fraction':float(np.mean(y<15.)),
            'saturated_pixel_fraction':float(np.mean(np.max(a,axis=2)>=250)),
            'note':'Rec.709 luminance on archived 8-bit RGB; saturation = any channel >=250'}


def bright_pass(s,r=BRIGHT_RULES):
    return bool(r['min_mean']<=s['mean']<=r['max_mean'] and s['std']>=r['min_std']
                and s['p50']>=r['min_p50'] and s['p95']>=r['min_p95']
                and s['dark_pixel_fraction']<=r['max_dark_fraction']
                and s['saturated_pixel_fraction']<=r['max_saturated_fraction'])


def rgb_difference(a,b):
    if np.shape(a)!=np.shape(b):raise ValueError('Same pixels required')
    d=np.abs(np.asarray(a,float)-np.asarray(b,float));p=d.max(axis=2)
    return {'mean_abs_0_255':float(d.mean()),'p95_abs_0_255':float(np.percentile(d,95)),
            'max_abs_0_255':float(d.max()),'fraction_any_channel_gt10':float(np.mean(p>10))}


def apply_profile(profile,stage=None):
    import carb
    settings=carb.settings.get_settings()
    for key,value in profile['renderer_settings'].items():settings.set(key,value)
    if profile.get('lights'):
        from pxr import UsdLux,UsdGeom,Gf
        if stage is None:raise ValueError('Stage required for fixed lights')
        for i,light in enumerate(profile['lights']):
            obj=UsdLux.SphereLight.Define(stage,f'/World/BrightHospital/FixedLight{i:02d}')
            obj.CreateIntensityAttr(light['intensity']);obj.CreateRadiusAttr(light['radius_m'])
            UsdGeom.Xformable(obj).AddTranslateOp().Set(Gf.Vec3d(*light['world_xyz']))
    return {k:settings.get(k) for k in profile['renderer_settings']}


def validate_profile_equal(a,b):
    if a['bright_profile']!=b['bright_profile']:raise ValueError('Bright baseline/human profile mismatch')
    for key in ['robot','scene','official_episode','instruction','camera','controller','inference','simulation','seed']:
        if a[key]!=b[key]:raise ValueError('Frozen execution condition changed: '+key)


def calibrated_config(base,profile):
    cfg=copy.deepcopy(base);cfg['bright_profile']=profile
    cfg['bright_validation']=dict(BRIGHT_RULES)
    return cfg
