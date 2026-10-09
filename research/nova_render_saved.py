"""Model-free render of saved Nova root/camera poses; physics stays stopped."""
import numpy as np


class SavedRenderer:
    def __init__(self,app,cfg):
        import omni.usd,omni.timeline,omni.replicator.core as rep
        from pxr import UsdGeom,UsdPhysics,Usd,Gf
        self.app,self.cfg=app,cfg;self.rep=rep;self.Gf=Gf
        self.timeline=omni.timeline.get_timeline_interface();self.timeline.stop()
        self.stage=omni.usd.get_context().get_stage()
        self.stage.GetRootLayer().subLayerPaths.append(cfg['scene']['usd'])
        UsdGeom.SetStageMetersPerUnit(self.stage,1.);UsdGeom.SetStageUpAxis(self.stage,'Z')
        root=UsdGeom.Xform.Define(self.stage,cfg['robot']['prim']);root.GetPrim().GetReferences().AddReference(cfg['robot']['asset'])
        root.ClearXformOpOrder();self.position=root.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'recorded')
        self.orientation=root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble,'recorded')
        for prim in Usd.PrimRange(root.GetPrim(),Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.IsInstanceProxy():UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        self.product=rep.create.render_product(cfg['camera']['prim'],tuple(cfg['camera']['resolution']))
        self.rgb=rep.AnnotatorRegistry.get_annotator('rgb');self.rgb.attach([self.product])
        for _ in range(20):app.update()

    def pose(self,event):
        from pxr import UsdGeom
        state=event['observation'];Gf=self.Gf
        self.position.Set(Gf.Vec3d(*state['position']));q=state['quaternion_wxyz']
        self.orientation.Set(Gf.Quatd(q[0],Gf.Vec3d(*q[1:])))
        body=UsdGeom.Xformable(self.stage.GetPrimAtPath(self.cfg['robot']['body'])).ComputeLocalToWorldTransform(0)
        if not np.allclose(body.ExtractTranslation(),state['position'],atol=1e-6):raise ValueError('Root/body pose mismatch')
        return np.array(UsdGeom.Xformable(self.stage.GetPrimAtPath(self.cfg['camera']['prim'])).ComputeLocalToWorldTransform(0)).tolist()

    def capture(self):
        import carb
        before=self.timeline.get_current_time()
        for _ in range(2):self.app.update()
        carb.settings.get_settings().set('/rtx-transient/post/dlss/forceParamReset',True)
        self.rep.orchestrator.step(rt_subframes=self.cfg['camera']['capture_subframes'],delta_time=0.,pause_timeline=True,wait_for_render=True)
        if self.timeline.is_playing() or self.timeline.get_current_time()!=before:raise RuntimeError('Saved render advanced time')
        a=np.asarray(self.rgb.get_data())[:,:,:3].copy()
        if a.shape!=(self.cfg['camera']['resolution'][1],self.cfg['camera']['resolution'][0],3):raise ValueError('Bad RGB shape')
        return a
