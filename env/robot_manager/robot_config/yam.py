"""YAM CAD dynamics with finite arm torques and nominal gravity compensation."""

from math import radians

from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import Articulation, ArticulationCfg
import isaaclab.sim as sim_utils

from env.global_configs import ROBOTS_PATH


class YamArticulation(Articulation):
    """Refresh arm gravity feedforward before each physics substep."""

    def write_data_to_sim(self):
        # PhysX uses the current articulation pose, scene gravity, and imported
        # masses, including distal fingers. Free objects are not part of this model.
        arm_joint_ids = self.actuators["arm"].joint_indices
        gravity = self.root_physx_view.get_gravity_compensation_forces()
        # Replace the prior substep's feedforward; finger drives remain unchanged.
        self.set_joint_effort_target(gravity[:, arm_joint_ids], joint_ids=arm_joint_ids)
        # IdealPDActuator clips the combined PD + gravity torque to effort_limit.
        super().write_data_to_sim()


def get_robot_config():
    return ArticulationCfg(
        class_type=YamArticulation,
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ROBOTS_PATH}/yam/yam.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                fix_root_link=True,
            ),
        ),
        # Shared home/reset pose for both arms, grippers open.
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": radians(10.0),
                "joint4": radians(-10.0),
                "joint5": 0.0,
                "joint6": 0.0,
                "joint7": 0.04695,
                "joint8": 0.04695,
            },
            pos=(0.0, 0.0, 0.0),
            rot=(0.707, 0, 0, 0.707),
        ),
        actuators={
            "arm": IdealPDActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                effort_limit={"joint[1-3]": 28.0, "joint[4-6]": 10.0},
                effort_limit_sim={"joint[1-3]": 28.0, "joint[4-6]": 10.0},
                velocity_limit_sim=5.0,
                stiffness={"joint[1-3]": 80.0, "joint[4-6]": 10.0},
                damping={"joint[1-3]": 5.0, "joint[4-6]": 1.5},
                # Provisional reflected motor inertia from the reference model.
                armature={"joint[1-3]": 0.032, "joint[4-6]": 0.0018},
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["joint7", "joint8"],
                effort_limit_sim=100.0,
                stiffness=2300,
                damping=100,
            ),
        },
    )
