"""Build the YAM robot assets for RoboDojo (URDF + USD) from the i2rt YAM model.

Runs in the RoboDojo conda env (needs IsaacLab for the USD step)::

    cd RoboDojo
    OMNI_KIT_ACCEPT_EULA=YES ~/miniconda3/envs/RoboDojo/bin/python \\
        scripts/robodojo_yam_assets.py --i2rt .cache/i2rt --out Assets/Robots/yam

The i2rt v1 URDF (``i2rt/robot_models/arm/yam/v1/yam.urdf``) has the same
kinematics as the MuJoCo YAM the policy was trained against
(deploy-anything ``data/mujoco/yam/assets/i2rt_yam/yam.xml``): with identical
joint values the URDF ``gripper`` origin sits 0.3 mm along the local +z axis
from MuJoCo's ``tcp_site``. The added ``grasp`` frame compensates for that
offset and matches MuJoCo's ``grasp_site`` to <0.01 mm / <0.001 deg. The URDF
``base`` sits 9.5 mm below MuJoCo's arm body. This script edits the URDF so
RoboDojo sees that robot:

* arm joint limits are replaced by the MuJoCo ones the policy was trained with;
* finger joints ``joint7``/``joint8`` are flipped to ``[0, 0.04695]`` with
  larger = more open (i2rt uses ``[-0.04695, 0]``), because RoboDojo's
  ``is_all_gripper_open`` assumes ``gripper_scale[0]`` is closed;
* fixed links ``grasp`` (MuJoCo ``grasp_site``: 0.1344 m along the gripper
  z axis) and ``wrist_camera`` (MuJoCo D405 wrist camera pose) are added;
* every visual mesh is reused as a collision mesh (the i2rt URDF has none);
* visual materials reproduce the white upper-arm and forearm covers and dark
  joints, base and gripper (the URDF importer drops mesh material colours).
* finger colliders use static and dynamic friction 1.2, with maximum combining.
* finger convex decomposition uses shrink-wrap and an explicit 32-hull budget
  to fit the collision surfaces to the visible gripping pads.
* virtual grasp/camera bodies use explicit negligible inertials so PhysX does
  not substitute its default mass for empty links.
"""

import argparse
import os
import shutil
import xml.etree.ElementTree as ET

import numpy as np

# MuJoCo YAM joint limits (deploy-anything i2rt_yam/yam.xml).
ARM_LIMITS = {
    "joint1": (-2.61799, 3.05433),
    "joint2": (0.0, 3.66519),
    "joint3": (0.0, 3.66519),
    "joint4": (-1.5708, 1.5708),
    "joint5": (-1.5708, 1.5708),
    "joint6": (-2.0944, 2.0944),
}
FINGER_TRAVEL = 0.04695
# Near-black albedo: under RoboDojo's bright dome light this renders the dark
# grey of the YAM hardware and of the training station model (0.3 albedo
# already washes out to near-white).
ARM_RGB = (0.02, 0.02, 0.02)
WHITE_RGB = (0.9, 0.9, 0.9)
FINGER_FRICTION = 1.2
# RoboDojo resolves grasp poses through articulation body indices. These
# numerical frame inertials retain those bodies without adding physical payload.
FRAME_MASS = 1e-6
FRAME_INERTIA = 1e-9
# Axial cover extents in each i2rt v1 link frame, in metres. The station's
# white model2__5/model2__8 covers share each URDF mesh with dark motor housings.
WHITE_LINK_Z = {"link2": (0.0305, 0.2335), "link3": (-0.2146, -0.0265)}
# MuJoCo grasp_site is 0.1347 m from tcp_site; the URDF gripper origin is
# 0.3 mm further along z than tcp_site.
GRASP_XYZ = "0 0 0.1344"
# MuJoCo wrist camera (put_bottle.xml ``left``/``right``) in the URDF gripper
# frame. URDF camera links follow the MuJoCo/USD camera convention (looking
# down -z, +y up). robot_config.yml sets the camera prim's local pose.
WRIST_CAMERA_XYZ = "0.0017 -0.095 0.0617"
WRIST_CAMERA_RPY = "2.26798 0 0"


def _fixed_link(robot, name, parent, xyz, rpy="0 0 0"):
    link = ET.SubElement(robot, "link", name=name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(FRAME_MASS))
    ET.SubElement(
        inertial,
        "inertia",
        ixx=str(FRAME_INERTIA),
        iyy=str(FRAME_INERTIA),
        izz=str(FRAME_INERTIA),
        ixy="0",
        ixz="0",
        iyz="0",
    )
    joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
    ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=name)


def build_urdf(src_urdf, out_urdf):
    tree = ET.parse(src_urdf)
    robot = tree.getroot()
    for link in robot.iter("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
        for visual in link.findall("visual"):
            collision = ET.SubElement(link, "collision")
            for child in visual:
                if child.tag in ("origin", "geometry"):
                    collision.append(child)
    for joint in robot.iter("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in ARM_LIMITS:
            lower, upper = ARM_LIMITS[name]
            limit.set("lower", str(lower))
            limit.set("upper", str(upper))
            limit.set("effort", "100")
            limit.set("velocity", "5")
        elif name in ("joint7", "joint8"):
            axis = joint.find("axis")
            axis.set("xyz", " ".join(str(-float(v) + 0.0) for v in axis.get("xyz").split()))
            limit.set("lower", "0")
            limit.set("upper", str(FINGER_TRAVEL))
            limit.set("effort", "100")
            limit.set("velocity", "1")
    _fixed_link(robot, "grasp", "gripper", GRASP_XYZ)
    _fixed_link(robot, "wrist_camera", "gripper", WRIST_CAMERA_XYZ, WRIST_CAMERA_RPY)
    ET.indent(tree)
    tree.write(out_urdf)


def convert_usd(urdf_path, usd_path):
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True).app
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(usd_path),
        usd_file_name=os.path.basename(usd_path),
        fix_base=True,
        merge_fixed_joints=False,
        force_usd_conversion=True,
        collider_type="convex_decomposition",
        self_collision=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None, damping=None),
        ),
    )
    UrdfConverter(cfg)
    set_virtual_frame_inertials(usd_path)
    bind_arm_material(usd_path)
    bind_finger_contact_material(usd_path)
    configure_finger_collisions(usd_path)
    print("YAM assets written to", os.path.dirname(usd_path), flush=True)
    app.close()


def set_virtual_frame_inertials(usd_path):
    """Give empty frame bodies explicit numerical mass and rotational inertia."""
    from pxr import Gf, Usd, UsdPhysics

    stage = Usd.Stage.Open(usd_path)
    root = stage.GetDefaultPrim().GetPath()
    for name in ("grasp", "wrist_camera"):
        frame = stage.GetPrimAtPath(root.AppendChild(name))
        if not frame or not frame.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"missing YAM frame rigid body: {name}")
        mass = UsdPhysics.MassAPI.Apply(frame)
        mass.CreateMassAttr(FRAME_MASS)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(FRAME_INERTIA))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    stage.Save()


def bind_arm_material(usd_path):
    """Bind the YAM's dark housings and white arm-cover visual materials.

    The converter makes each link's ``visuals`` prim instanceable and binds its
    own material inside the prototype, which a binding on the instance cannot
    override; de-instance the visuals and bind every mesh directly.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    stage = Usd.Stage.Open(usd_path)
    root = stage.GetDefaultPrim().GetPath()
    materials = {}
    for name, rgb in (("dark_grey", ARM_RGB), ("white", WHITE_RGB)):
        material = UsdShade.Material.Define(stage, root.AppendPath(f"Looks/yam_{name}"))
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Shader"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials[name] = material
    for prim in list(stage.Traverse()):
        if prim.GetName() == "visuals" and prim.IsInstanceable():
            prim.SetInstanceable(False)
    bound = 0
    white_links = set()
    transforms = UsdGeom.XformCache()
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdGeom.Gprim) and "/visuals" in str(prim.GetPath()):
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(materials["dark_grey"], UsdShade.Tokens.weakerThanDescendants)
            bound += 1
            link_name = str(prim.GetPath().MakeRelativePath(root)).split("/")[0]
            if link_name in WHITE_LINK_Z:
                mesh = UsdGeom.Mesh(prim)
                counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
                if not np.all(counts == 3):
                    raise ValueError(f"expected triangular YAM cover mesh: {prim.GetPath()}")
                points = np.asarray(mesh.GetPointsAttr().Get())
                faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
                link = stage.GetPrimAtPath(root.AppendChild(link_name))
                transform, _ = transforms.ComputeRelativeTransform(prim, link)
                centers = points[faces].mean(axis=1)
                matrix = np.asarray(transform)
                z = centers @ matrix[:3, 2] + matrix[3, 2]
                lower, upper = WHITE_LINK_Z[link_name]
                white = (z >= lower) & (z <= upper)
                if not white.any() or white.all():
                    raise ValueError(f"expected white cover and dark housing faces: {prim.GetPath()}")
                for name, mask in (("white", white), ("dark_grey", ~white)):
                    subset = binding.CreateMaterialBindSubset(f"yam_{name}", np.flatnonzero(mask).tolist())
                    UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[name])
                binding.SetMaterialBindSubsetsFamilyType(UsdGeom.Tokens.partition)
                white_links.add(link_name)
    if bound == 0:
        raise RuntimeError(f"no visual meshes found in {usd_path}")
    if white_links != set(WHITE_LINK_Z):
        raise RuntimeError(f"missing YAM white cover meshes in {usd_path}: {set(WHITE_LINK_Z) - white_links}")
    print(f"bound {bound} visual meshes; white covers on {sorted(white_links)}", flush=True)
    stage.Save()


def bind_finger_contact_material(usd_path):
    """Assign the finger contact material without changing collision geometry."""
    from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

    stage = Usd.Stage.Open(usd_path)
    root = stage.GetDefaultPrim().GetPath()
    material = UsdShade.Material.Define(stage, root.AppendPath("Looks/yam_finger_contact"))
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(FINGER_FRICTION)
    physics.CreateDynamicFrictionAttr(FINGER_FRICTION)
    physics.CreateRestitutionAttr(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr("max")
    physx.CreateRestitutionCombineModeAttr("average")
    for name in ("tip_left", "tip_right"):
        finger = stage.GetPrimAtPath(root.AppendChild(name))
        if not finger or not finger.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"missing YAM finger rigid body: {name}")
        UsdShade.MaterialBindingAPI.Apply(finger).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
    stage.Save()


def configure_finger_collisions(usd_path):
    """Fit finger convex hulls to the STL with shrink-wrap.

    Only collision instances are expanded; visuals and link frames stay intact.
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    root = stage.GetDefaultPrim().GetPath()
    for name in ("tip_left", "tip_right"):
        collisions = stage.GetPrimAtPath(root.AppendPath(f"{name}/collisions"))
        if not collisions:
            raise ValueError(f"missing YAM finger collisions: {name}")
        collisions.SetInstanceable(False)
        colliders = [prim for prim in Usd.PrimRange(collisions) if prim.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders:
            raise ValueError(f"missing YAM finger collider: {name}")
        for prim in colliders:
            if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != "convexDecomposition":
                raise ValueError(f"expected convex decomposition: {prim.GetPath()}")
            settings = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            settings.CreateShrinkWrapAttr(True)
            settings.CreateMaxConvexHullsAttr(32)
            settings.CreateVoxelResolutionAttr(500000)
    stage.Save()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--i2rt", required=True, help="i2rt checkout")
    parser.add_argument("--out", required=True, help="output robot dir, e.g. Assets/Robots/yam")
    args = parser.parse_args()

    src_dir = os.path.join(args.i2rt, "i2rt", "robot_models", "arm", "yam", "v1")
    os.makedirs(args.out, exist_ok=True)
    shutil.copytree(os.path.join(src_dir, "assets"), os.path.join(args.out, "assets"), dirs_exist_ok=True)
    urdf_path = os.path.abspath(os.path.join(args.out, "yam.urdf"))
    build_urdf(os.path.join(src_dir, "yam.urdf"), urdf_path)
    shutil.copy2(
        os.path.join(os.path.dirname(__file__), "..", "env_cfg", "robot", "yam_robot_config.yml"),
        os.path.join(args.out, "robot_config.yml"),
    )
    # Isaac's app.close() ends the process, so this is the last step.
    convert_usd(urdf_path, os.path.abspath(os.path.join(args.out, "yam.usd")))


if __name__ == "__main__":
    main()
