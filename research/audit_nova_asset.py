"""Model-free inspection of the exact Nova asset named by the official runner."""
import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.nova_source import source_audit
from research.records import write_json, provenance


def inspect(stage, root, source):
    from pxr import Usd, UsdGeom, UsdPhysics
    import numpy as np
    cache = UsdGeom.BBoxCache(0, ["default", "render", "proxy"])
    joints, wheels, cameras = [], [], []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if prim.IsA(UsdPhysics.RevoluteJoint):
            j = UsdPhysics.RevoluteJoint(prim)
            joints.append({"path": path, "axis": str(j.GetAxisAttr().Get()),
                "local_pos0": list(j.GetLocalPos0Attr().Get()), "local_pos1": list(j.GetLocalPos1Attr().Get()),
                "body0": list(map(str, j.GetBody0Rel().GetTargets())), "body1": list(map(str, j.GetBody1Rel().GetTargets())),
                "local_rot0": str(j.GetLocalRot0Attr().Get()), "local_rot1": str(j.GetLocalRot1Attr().Get()),
                "drives": {a.GetName(): str(a.Get()) for a in prim.GetAttributes() if "drive:" in a.GetName()}})
        if "wheel" in path.lower() and (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cylinder)):
            b = cache.ComputeLocalBound(prim).ComputeAlignedRange()
            wheels.append({"path": path, "type": prim.GetTypeName(), "bounds_min": list(b.GetMin()),
                "bounds_max": list(b.GetMax()), "collision_enabled": prim.HasAPI(UsdPhysics.CollisionAPI),
                "radius_attr": prim.GetAttribute("radius").Get() if prim.HasAttribute("radius") else None,
                "height_attr": prim.GetAttribute("height").Get() if prim.HasAttribute("height") else None,
                "axis_attr": str(prim.GetAttribute("axis").Get()) if prim.HasAttribute("axis") else None,
                "authored_extent": str(prim.GetAttribute("extent").Get()),
                "world_transform": np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)).tolist()})
        if prim.IsA(UsdGeom.Camera):
            cam = UsdGeom.Camera(prim)
            cameras.append({"path": path, "world_transform": np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)).tolist(),
                "focal_length": cam.GetFocalLengthAttr().Get(), "horizontal_aperture": cam.GetHorizontalApertureAttr().Get(),
                "vertical_aperture": cam.GetVerticalApertureAttr().Get(), "clipping_range": list(cam.GetClippingRangeAttr().Get())})
    return {"source": source, "root_prim": root, "stage_meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
            "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)), "joints": joints, "wheel_geometry": wheels,
            "cameras": cameras, "expected_camera_exists": bool(stage.GetPrimAtPath(source["front_camera"])),
            "rigid_bodies": [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(); out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=False)
    source = source_audit(); write_json(out/"metadata.json", {"source": source, **provenance(ROOT)})
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})
    code = 1
    try:
        import omni.usd
        from isaacsim.core.utils.stage import add_reference_to_stage
        root = source["front_camera"].split("/chassis_link/")[0]
        add_reference_to_stage(source["asset"], root)
        for _ in range(10): app.update()
        result = inspect(omni.usd.get_context().get_stage(), root, source)
        write_json(out/"asset_audit.json", result)
        print("NOVA_ASSET_AUDIT", json.dumps({"joints": len(result["joints"]), "camera_exists": result["expected_camera_exists"]}), flush=True)
        code = 0
    except Exception as exc:
        traceback.print_exc(); write_json(out/"failure.json", {"error": repr(exc)})
    finally:
        app.close(exit_code=code)


if __name__ == "__main__": main()
