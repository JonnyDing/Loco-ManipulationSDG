"""Manager-based G1 environment in the captured lab scene."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCollectionCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from loco_manipulation.config import (
    G1_CFG,
    LAB_WORLD_POS,
    LAB_WORLD_ROT_XYZW,
    OBJECTS_CFG,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCENE_USD_PATH = (PROJECT_ROOT / "assets/scene/lab/scene.usd").as_posix()
CAMERA_EYE = (15.0, 15.0, 10.0)
CAMERA_LOOKAT = (0.0, -3.57, 0.7)


@configclass
class LocoManipulationSceneCfg(InteractiveSceneCfg):
    """Captured lab scene with one G1 articulation per environment."""

    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(visible=False),
    )
    lab = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Lab",
        spawn=sim_utils.UsdFileCfg(usd_path=SCENE_USD_PATH),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=LAB_WORLD_POS,
            rot=LAB_WORLD_ROT_XYZW,
        ),
    )
    robot: ArticulationCfg = G1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    objects: RigidObjectCollectionCfg = OBJECTS_CFG
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera",
        update_period=0.0,
        height=360,
        width=640,
        data_types=["rgb"],
        offset=CameraCfg.OffsetCfg(
            pos=CAMERA_EYE,
            rot=(-0.1772688, -0.0876014, 0.8788065, -0.4342822),
            convention="world",
        ),
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=20.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 100.0),
        ),
    )


@configclass
class ActionsCfg:
    """Action terms placeholder."""


@configclass
class ObservationsCfg:
    """Observation terms placeholder."""


@configclass
class LocoManipulationEnvCfg(ManagerBasedEnvCfg):
    """Manager-based G1 environment using the captured lab scene."""

    scene: LocoManipulationSceneCfg = LocoManipulationSceneCfg(
        num_envs=1,
        env_spacing=10,
        replicate_physics=True,
        clone_in_fabric=False,
    )
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.sim.dt = 1.0 / 200.0
        self.sim.render_interval = self.decimation
        self.viewer.eye = CAMERA_EYE
        self.viewer.lookat = CAMERA_LOOKAT
