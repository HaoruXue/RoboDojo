# YAM and X5 environment fixes

This fork contains the existing YAM/X5 robot and object-physics changes used
by the one-model integration. It retains RoboDojo's directory structure,
tasks and success checks. No new test suite or simulator framework is added.
These are modified environment conditions, not an upstream benchmark result.

## Environment configurations

| Configuration | Robot/control | Camera condition |
| --- | --- | --- |
| `arx_x5` | Native X5, native drives and 25 Hz target interpolation | Original mounts, 640×480, square-pixel head correction |
| `arx_x5_contact` | Same X5 controller, finger friction 1.2/max USD layer | Same native cameras |
| `yam` | YAM station, 30 Hz held arm/gripper targets, finite PD plus gravity compensation | ZED wide approximation, head 864×540, wrists 384×240 |
| `yam_station` | Same YAM articulation | Saved head zoom125 condition, same fitted wrist mounts |

YAM J1–3 use Kp=80, Kd=5, torque limit 28 N·m; J4–6 use Kp=10,
Kd=1.5, torque limit 10 N·m. Fingers use Kp=2300, Kd=100 and static/dynamic
friction 1.2 with maximum combining. Grasp/camera links have negligible
explicit inertials. Finger convex decomposition uses shrink-wrap, up to
32 hulls and voxel resolution 500000. Long arm covers are white.

The shared YAM home is [0,0,10,-10,0,0] degrees with open fingers. The
one-model live launch can select its saved raised twelve-angle pose without
changing this default. Camera resolutions can be set per camera through
`camera.resolution: [width, height]`; wrist mount defaults are combined with
explicit camera configuration. Undistorted head zoom125 uses fx=fy=412.5 at
864×540; nominal head fx=fy=330 and wrist fx=fy=146.666667 at 384×240.
These optics/mounts are approximations, not factory calibration.

## Assets

Use a private writable asset checkout. Keep the upstream asset revision
and the correction bundle together when recording the environment identity.
The bundle contains 6,557 corrected JSON files and one toaster USDZ; all
non-mass JSON fields are preserved. `asset_overrides/rigid_masses.json`
records per-asset reference masses. Layout masses include the previously
reviewed scale dependence. Values are engineering estimates, not measured
real-object weights. Wood reference: 35 mm cube = 25 g. Garments use
175 g/m² cotton; fluids use waterlike density 1000 kg/m³. Changes to particle
spacing or scale require new mass calculations. Unrelated collision,
material, geometry and task-placement fields remain unchanged.

Download upstream assets at revision
`63fcc045f2e974902eb7800d29de48004cf86016`, then apply the reviewed bundle:

```bash
HF_REVISION=63fcc045f2e974902eb7800d29de48004cf86016 bash scripts/init_assets.sh
gh release download audited-assets-20261116 --repo HaoruXue/RoboDojo \
  --pattern audited-masses-20261116.tar.gz
sha256sum --check asset_overrides/audited-masses-20261116.sha256
tar --unlink-first -xzf audited-masses-20261116.tar.gz
```

The rigid loader requires positive mass before initialization, without a
500 g cap. It retains collider mass properties while making the wrapper the
single rigid body. The fluid loader consumes explicit particle mass. The
bundle changes the toaster slider mass to 7.481368 g. Existing cloth gravity,
conveyor configuration and other physics settings remain unchanged.

Build YAM from i2rt revision `120c3c81400171174604e503943f8d1ebc891058` with
RoboDojo's Isaac Python; run from this checkout root:

```bash
python scripts/robodojo_yam_assets.py --i2rt /path/to/pinned/i2rt --out Assets/Robots/yam
python scripts/robodojo_x5_contact.py --source Assets/Robots/x5/ARX.usd \
  --output Assets/Robots/x5/ARX_friction_max.usda
```

The X5 override sublayers the original asset so external prototype/material
relationships remain valid. `arx_x5` retains the original asset; select
`arx_x5_contact` to use the contact override. Keep generated USD/URDF and
meshes with their source revisions; do not modify a shared upstream cache.

Use one-model's existing local Isaac inspector or headless worker with the
selected environment YAML. Model serving, canonical policy-frame conversion,
virtual-leader IK and shared Viser/chat controls remain outside this fork.
