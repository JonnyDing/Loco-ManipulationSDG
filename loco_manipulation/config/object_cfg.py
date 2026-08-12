"""Rigid objects used by the loco-manipulation environment."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg

from .scene_coordinates import (
    BAMBOO_BASKET_WORLD_POS,
    BAMBOO_BASKET_WORLD_ROT_XYZW,
    CUP_WORLD_POS,
    CUP_WORLD_ROT_XYZW,
)

OBJECTS_ROOT = Path(__file__).resolve().parents[2] / "assets/objects"

# HSDE source poses use WXYZ; Isaac Lab 3.0 InitialStateCfg uses XYZW.
OBJECTS_CFG = RigidObjectCollectionCfg(
    rigid_objects={
        "cup": RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cup",
            spawn=sim_utils.UsdFileCfg(
                usd_path=(OBJECTS_ROOT / "cup/cup.usd").as_posix()
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=CUP_WORLD_POS,
                rot=CUP_WORLD_ROT_XYZW,
            ),
        ),
        "bamboo_basket": RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/BambooBasket",
            spawn=sim_utils.UsdFileCfg(
                usd_path=(OBJECTS_ROOT / "bamboo_basket/bamboo_basket.usd").as_posix()
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=BAMBOO_BASKET_WORLD_POS,
                rot=BAMBOO_BASKET_WORLD_ROT_XYZW,
            ),
        ),
    }
)
