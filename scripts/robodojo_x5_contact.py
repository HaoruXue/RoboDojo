"""Create an X5 USD override with finger friction 1.2 and maximum combining.

Run with Isaac's USD/PhysX Python environment. Point the X5 spawn configuration
at the output file and restart the simulator to load the contact experiment.
"""

import argparse
from pathlib import Path


def build_contact_override(source: Path, output: Path) -> None:
    """Layer over the original robot and override both finger contact materials."""
    from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

    source, output = source.resolve(strict=True), output.resolve()
    if source == output or output.exists():
        raise ValueError("output must be a new USD file, separate from the source")
    original = Usd.Stage.Open(str(source))
    root = original.GetDefaultPrim()
    fingers = [root.GetPath().AppendChild(name) for name in ("link7", "link8")]
    for path in fingers:
        finger = original.GetPrimAtPath(path)
        if not finger or not finger.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"missing X5 finger rigid body: {path}")
        if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(finger)):
            raise ValueError(f"missing X5 finger collider: {path}")

    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    for name in ("upAxis", "metersPerUnit", "kilogramsPerUnit"):
        if original.HasAuthoredMetadata(name):
            stage.SetMetadata(name, original.GetMetadata(name))
    stage.SetDefaultPrim(stage.GetPrimAtPath(root.GetPath()))
    material = UsdShade.Material.Define(stage, root.GetPath().AppendPath("Looks/x5_finger_contact"))
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(1.2)
    physics.CreateDynamicFrictionAttr(1.2)
    physics.CreateRestitutionAttr(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr("max")
    physx.CreateRestitutionCombineModeAttr("average")
    for path in fingers:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
    stage.Save()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build_contact_override(args.source, args.output)
