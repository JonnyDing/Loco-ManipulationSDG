"""Manager-based Unitree G1 scene and environment configuration."""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_G1_USD_PATH = REPOSITORY_ROOT / "assets/robots/Unitree/G1/g1_real_arm.usda"


def resolve_g1_usd_path() -> str:
    """Resolve the project-local G1 asset, with an optional environment override."""

    override = os.environ.get("G1_USD_PATH")
    path = Path(override).expanduser() if override else DEFAULT_G1_USD_PATH
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Unitree G1 USD asset not found: {path}")
    return path.as_posix()


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


def implicit_actuator(
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
        usd_path=resolve_g1_usd_path(),
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
        "legs_and_waist": implicit_actuator(
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
        "feet": implicit_actuator(
            joints=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort=35.0,
            velocity=30.0,
            stiffness=20.0,
            damping=2.0,
        ),
        "shoulders": implicit_actuator(
            joints=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
            effort=25.0,
            velocity=37.0,
            stiffness=100.0,
            damping=2.0,
        ),
        "arms": implicit_actuator(
            joints=[".*_shoulder_yaw_joint", ".*_elbow_joint"],
            effort=25.0,
            velocity=37.0,
            stiffness=50.0,
            damping=2.0,
        ),
        "wrists": implicit_actuator(
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
        "hands": implicit_actuator(
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


@configclass
class G1SceneCfg(InteractiveSceneCfg):
    """Scene containing a G1 articulation, ground plane, and dome light."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(20.0, 20.0)),
    )

    robot: ArticulationCfg = G1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


@configclass
class EmptyManagerCfg:
    """Placeholder manager configuration until task-specific MDP terms are added."""


@configclass
class G1EnvCfg(ManagerBasedEnvCfg):
    """Base manager-based environment used to load and inspect the G1 robot."""

    scene: G1SceneCfg = G1SceneCfg(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
        clone_in_fabric=False,
    )
    actions: EmptyManagerCfg = EmptyManagerCfg()
    observations: EmptyManagerCfg = EmptyManagerCfg()

    def __post_init__(self) -> None:
        self.decimation = 2
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation

        # Frame the complete standing robot without requiring an initial zoom-out.
        self.viewer.eye = (4.5, 4.5, 2.8)
        self.viewer.lookat = (0.0, 0.0, 0.95)
