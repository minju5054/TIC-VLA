"""One static reference to an existing Hospital cart; no invented collider."""
import hashlib,json,csv
from pathlib import Path
import numpy as np
from research.records import write_json,clocks

SOURCE='/Root/SM_SupplyCart_01e_21'
COPY='/World/E16Cart'
LABEL='stationary_hospital_cart'


def signature(stage,path):
    from pxr import Usd
    rows=[]
    for p in Usd.PrimRange(stage.GetPrimAtPath(path)):
        rows.append([str(p.GetPath()).replace(path,'CART'),p.GetTypeName(),p.GetAppliedSchemas(),
            [(a.GetName(),str(a.Get()).replace(path,'CART')) for a in p.GetAttributes()],
            [(r.GetName(),str(r.GetTargets()).replace(path,'CART')) for r in p.GetRelationships()]])
    return hashlib.sha256(json.dumps(rows,sort_keys=True).encode()).hexdigest()


def author(stage,frozen):
    from pxr import UsdGeom,UsdPhysics,UsdShade,Usd,Gf
    from isaacsim.core.utils.semantics import add_labels
    source=stage.GetPrimAtPath(frozen['source_prim']);before=signature(stage,frozen['source_prim'])
    if before!=frozen['source_prim_signature']:raise ValueError('Original cart source differs from frozen audit')
    if stage.GetPrimAtPath(COPY):raise ValueError('Exactly one fresh cart copy required')
    root=UsdGeom.Xform.Define(stage,COPY);root.GetPrim().GetReferences().AddReference(frozen['scene_usd'],frozen['source_prim'])
    root.ClearXformOpOrder();root.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(frozen['cart']['matrix_row_vector']).ravel()))
    colliders=[]
    for p in Usd.PrimRange(root.GetPrim(),Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Expected original static cart collision geometry')
        if p.HasAPI(UsdPhysics.CollisionAPI):
            original=stage.GetPrimAtPath(str(p.GetPath()).replace(COPY,frozen['source_prim'],1))
            for attr in ['points','faceVertexCounts','faceVertexIndices','physics:collisionEnabled','physics:approximation']:
                if str(p.GetAttribute(attr).Get())!=str(original.GetAttribute(attr).Get()):raise ValueError('Copied mesh/collider differs')
            sm=UsdShade.MaterialBindingAPI(original).ComputeBoundMaterial()[0];cm=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            if str(cm.GetPath()).replace(COPY,'CART')!=str(sm.GetPath()).replace(frozen['source_prim'],'CART'):raise ValueError('Material binding changed')
            colliders.append(str(p.GetPath()))
    if not colliders:raise ValueError('Cart has no usable original collision geometry')
    add_labels(root.GetPrim(),[LABEL])
    if signature(stage,frozen['source_prim'])!=before:raise ValueError('Original cart modified')
    bounds=UsdGeom.BBoxCache(0,['default','render','proxy']).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange()
    lo,hi=np.array(bounds.GetMin()),np.array(bounds.GetMax())
    np.testing.assert_allclose((lo[:2]+hi[:2])/2,frozen['cart']['center_world_xyz'][:2],atol=1e-12,rtol=0)
    np.testing.assert_allclose(lo[2],frozen['cart']['center_world_xyz'][2],atol=1e-10,rtol=0)
    return {'copy_prim':COPY,'world_bounds':[lo.tolist(),hi.tolist()],'collision_prims':colliders,'source_unchanged':True}


def mask_stats(packed,rules):
    from research.nova_occlusion import segmentation_stats,visibility_state
    s,mask=segmentation_stats(packed['data'],packed['info']['idToLabels'],LABEL)
    state=visibility_state(s,rules);s={k.replace('human_','cart_'):v for k,v in s.items()}
    return {**s,'state':'CLEAR' if state=='VISIBLE' else state,'visible':s['cart_visible_pixel_count']>0},mask


class StationaryCart:
    def __init__(self,sim,frozen):
        self.sim,self.frozen=sim,frozen;info=author(sim.stage,frozen)
        write_json(sim.records.path/'cart_asset.json',{**info,'frozen':frozen})
        self.file=(sim.records.path/'raw/cart_state.csv').open('x');self.writer=csv.writer(self.file)
        self.writer.writerow(['tick','sim_time','x','y','z','yaw','stationary'])

    def update(self,sim_time):pass

    def snapshot(self):
        from pxr import UsdGeom
        matrix=np.array(UsdGeom.Xformable(self.sim.stage.GetPrimAtPath(COPY)).ComputeLocalToWorldTransform(0))
        np.testing.assert_allclose(matrix,self.frozen['cart']['matrix_row_vector'],atol=1e-12,rtol=0)
        c=self.frozen['cart']
        return {'object_type':'stationary_hospital_cart','position':c['center_world_xyz'],'yaw':c['yaw_rad'],
            'sim_time':self.sim.state()['sim_time'],'motion_active':False,'velocity_world':[0.,0.,0.],
            'event_source':'unchanged USD static-collider world transform; not a dynamic PhysX rigid-body measurement',**clocks()}

    def measure(self,sim_time):
        s=self.snapshot();self.writer.writerow([self.sim.tick,sim_time,*s['position'],s['yaw'],True])

    def close(self):self.file.close()
