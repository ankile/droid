import numpy as np
from dm_control import mjcf
from dm_robotics.moma.effectors import arm_effector, cartesian_6d_velocity_effector

from droid.robot_ik.arm import FrankaArm


# Nullspace posture references, selectable per controller-launch via the
# `nullspace_reference` arg threaded through FrankaRobot.launch_robot ->
# ServerInterface -> RobotEnv (GPU). On a 0-velocity (hold) command the
# nullspace bias gain*(ref - q) is the only joint drive, so this choice
# determines whether the arm droops.
NULLSPACE_REFERENCES = {
    # LEGACY config used to collect all pre-fix data. 0 is OUTSIDE the joint
    # range for j4/j6, so the bias drags the wrist toward an at-limit target
    # -> the ~5cm hold-time EE droop. Kept to reproduce the data-collection
    # distribution when deploying policies trained under it.
    "legacy_zero": [0.0] * 7,
    # FEASIBLE canonical work posture (robot_env reset_joints) -> no droop,
    # gentle posture centering.
    "home": [0.0, -1.0 / 5.0 * np.pi, 0.0, -4.0 / 5.0 * np.pi, 0.0, 3.0 / 5.0 * np.pi, 0.0],
}
DEFAULT_NULLSPACE_REFERENCE = "home"


class RobotIKSolver:
    def __init__(self, nullspace_reference=None):
        # Resolve the nullspace posture reference: None -> default mode; a
        # str selects a named mode in NULLSPACE_REFERENCES (KeyError if
        # unknown, by design); a length-7 sequence is used verbatim.
        if nullspace_reference is None:
            nullspace_reference = DEFAULT_NULLSPACE_REFERENCE
        if isinstance(nullspace_reference, str):
            self.nullspace_reference_name = nullspace_reference
            nullspace_ref = list(NULLSPACE_REFERENCES[nullspace_reference])
        else:
            self.nullspace_reference_name = "custom"
            nullspace_ref = list(nullspace_reference)
            assert len(nullspace_ref) == 7, "nullspace_reference must have 7 entries"
        print("[RobotIKSolver] nullspace reference: %s = %s"
              % (self.nullspace_reference_name, nullspace_ref))
        self.relative_max_joint_delta = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.max_joint_delta = self.relative_max_joint_delta.max()
        self.max_gripper_delta = 0.25
        self.max_lin_delta = 0.075
        self.max_rot_delta = 0.15
        self.control_hz = 15

        self._arm = FrankaArm()
        self._physics = mjcf.Physics.from_mjcf_model(self._arm.mjcf_model)
        self._effector = arm_effector.ArmEffector(arm=self._arm, action_range_override=None, robot_name=self._arm.name)

        self._effector_model = cartesian_6d_velocity_effector.ModelParams(self._arm.wrist_site, self._arm.joints)

        self._effector_control = cartesian_6d_velocity_effector.ControlParams(
            control_timestep_seconds=1 / self.control_hz,
            max_lin_vel=self.max_lin_delta,
            max_rot_vel=self.max_rot_delta,
            joint_velocity_limits=self.relative_max_joint_delta,
            # Selectable nullspace posture reference (resolved in __init__ from
            # the nullspace_reference arg; see NULLSPACE_REFERENCES). Default
            # "home" fixes the hold-time droop; "legacy_zero" is the old [0]*7.
            nullspace_joint_position_reference=nullspace_ref,
            nullspace_gain=0.025,
            regularization_weight=1e-2,
            enable_joint_position_limits=True,
            minimum_distance_from_joint_position_limit=0.3,
            joint_position_limit_velocity_scale=0.95,
            max_cartesian_velocity_control_iterations=300,
            max_nullspace_control_iterations=300,
        )

        self._cart_effector_6d = cartesian_6d_velocity_effector.Cartesian6dVelocityEffector(
            self._arm.name, self._effector, self._effector_model, self._effector_control
        )
        self._cart_effector_6d.after_compile(self._arm.mjcf_model, self._physics)

    ### Inverse Kinematics ###
    def cartesian_velocity_to_joint_velocity(self, cartesian_velocity, robot_state):
        cartesian_delta = self.cartesian_velocity_to_delta(cartesian_velocity)
        qpos = np.array(robot_state["joint_positions"])
        qvel = np.array(robot_state["joint_velocities"])

        self._arm.update_state(self._physics, qpos, qvel)
        self._cart_effector_6d.set_control(self._physics, cartesian_delta)
        joint_delta = self._physics.bind(self._arm.actuators).ctrl.copy()
        np.any(joint_delta)

        joint_velocity = self.joint_delta_to_velocity(joint_delta)

        return joint_velocity

    ### Velocity To Delta ###
    def gripper_velocity_to_delta(self, gripper_velocity):
        gripper_vel_norm = np.linalg.norm(gripper_velocity)

        if gripper_vel_norm > 1:
            gripper_velocity = gripper_velocity / gripper_vel_norm

        gripper_delta = gripper_velocity * self.max_gripper_delta

        return gripper_delta

    def cartesian_velocity_to_delta(self, cartesian_velocity):
        if isinstance(cartesian_velocity, list):
            cartesian_velocity = np.array(cartesian_velocity)

        lin_vel, rot_vel = cartesian_velocity[:3], cartesian_velocity[3:6]

        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)

        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm

        lin_delta = lin_vel * self.max_lin_delta
        rot_delta = rot_vel * self.max_rot_delta

        return np.concatenate([lin_delta, rot_delta])

    def joint_velocity_to_delta(self, joint_velocity):
        if isinstance(joint_velocity, list):
            joint_velocity = np.array(joint_velocity)

        relative_max_joint_vel = self.joint_delta_to_velocity(self.relative_max_joint_delta)
        max_joint_vel_norm = (np.abs(joint_velocity) / relative_max_joint_vel).max()

        if max_joint_vel_norm > 1:
            joint_velocity = joint_velocity / max_joint_vel_norm

        joint_delta = joint_velocity * self.max_joint_delta

        return joint_delta

    ### Delta To Velocity ###
    def gripper_delta_to_velocity(self, gripper_delta):
        return gripper_delta / self.max_gripper_delta

    def cartesian_delta_to_velocity(self, cartesian_delta):
        if isinstance(cartesian_delta, list):
            cartesian_delta = np.array(cartesian_delta)

        cartesian_velocity = np.zeros_like(cartesian_delta)
        cartesian_velocity[:3] = cartesian_delta[:3] / self.max_lin_delta
        cartesian_velocity[3:6] = cartesian_delta[3:6] / self.max_rot_delta

        return cartesian_velocity

    def joint_delta_to_velocity(self, joint_delta):
        if isinstance(joint_delta, list):
            joint_delta = np.array(joint_delta)

        return joint_delta / self.max_joint_delta
