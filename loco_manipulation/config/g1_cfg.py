"""Unitree G1 articulation configuration."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

PROJECT_ROOT = Path(__file__).resolve().parents[2]
G1_USD_PATH = (PROJECT_ROOT / "assets/robots/Unitree/G1/g1_real_arm.usda").as_posix()


G1_NOMINAL_STANDING_JOINT_POS = {
    ".*_hip_pitch_joint": -0.20,
    ".*_hip_roll_joint": 0.0,
    ".*_hip_yaw_joint": 0.0,
    ".*_knee_joint": 0.42,
    ".*_ankle_pitch_joint": -0.23,
    ".*_ankle_roll_joint": 0.0,
    ".*_elbow_joint": 0.87,
    "left_shoulder_roll_joint": 0.18,
    "left_shoulder_pitch_joint": 0.35,
    "right_shoulder_roll_joint": -0.18,
    "right_shoulder_pitch_joint": 0.35,
    ".*_hand_.*_joint": 0.0,
}


def _implicit_actuator_cfg(
    joints: list[str],
    effort: float | dict[str, float],
    velocity: float | dict[str, float],
    stiffness: float | dict[str, float],
    damping: float | dict[str, float],
) -> ImplicitActuatorCfg:
    return ImplicitActuatorCfg(
        joint_names_expr=joints,
        effort_limit_sim=effort,
        velocity_limit_sim=velocity,
        stiffness=stiffness,
        damping=damping,
        armature=0.01,
    )


# Adapted from /home/ubuntu/Sia/HSDE/pipeline/robots/g1.py. The asset path is
# repository-local and the hand defaults use a regex to avoid depending on HSDE.
G1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=G1_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=False,
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        joint_pos=G1_NOMINAL_STANDING_JOINT_POS,
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.90,
    actuators={
        "legs_and_waist": _implicit_actuator_cfg(
            joints=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
                ".*waist.*",
            ],
            effort={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_pitch_joint": 88.0,
                ".*_knee_joint": 139.0,
                ".*waist_yaw_joint": 88.0,
                ".*waist_roll_joint": 35.0,
                ".*waist_pitch_joint": 35.0,
            },
            velocity={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 32.0,
                ".*_knee_joint": 20.0,
                ".*waist_yaw_joint": 32.0,
                ".*waist_roll_joint": 30.0,
                ".*waist_pitch_joint": 30.0,
            },
            stiffness={
                ".*_hip_yaw_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 200.0,
                ".*waist.*": 200.0,
            },
            damping=5.0,
        ),
        "feet": _implicit_actuator_cfg(
            joints=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort=35.0,
            velocity=30.0,
            stiffness=20.0,
            damping=2.0,
        ),
        "shoulders": _implicit_actuator_cfg(
            joints=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
            effort=25.0,
            velocity=37.0,
            stiffness=100.0,
            damping=2.0,
        ),
        "arms": _implicit_actuator_cfg(
            joints=[".*_shoulder_yaw_joint", ".*_elbow_joint"],
            effort=25.0,
            velocity=37.0,
            stiffness=50.0,
            damping=2.0,
        ),
        "wrists": _implicit_actuator_cfg(
            joints=[".*_wrist_.*"],
            effort={
                ".*_wrist_yaw_joint": 5.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
            },
            velocity={
                ".*_wrist_yaw_joint": 22.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
            },
            stiffness=40.0,
            damping=2.0,
        ),
        "hands": _implicit_actuator_cfg(
            joints=[
                ".*_hand_thumb_[0-2]_joint",
                ".*_hand_middle_[0-1]_joint",
                ".*_hand_index_[0-1]_joint",
            ],
            effort=2.0,
            velocity=37.0,
            stiffness=0.5,
            damping=0.1,
        ),
    },
)
