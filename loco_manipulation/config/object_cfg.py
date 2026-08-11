"""Rigid objects used by the loco-manipulation environment."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg

OBJECTS_ROOT = Path(__file__).resolve().parents[2] / "assets/objects"

# Source poses use Isaac Sim 5.x (w, x, y, z); InitialStateCfg uses (x, y, z, w).
OBJECTS_CFG = RigidObjectCollectionCfg(
    rigid_objects={
        "cup": RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cup",
            spawn=sim_utils.UsdFileCfg(
                usd_path=(OBJECTS_ROOT / "cup/cup.usd").as_posix()
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(-0.27, -3.54, 0.64),
                rot=(1.0, 0.0, 0.0, 0.0),
            ),
        ),
        "bamboo_basket": RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/BambooBasket",
            spawn=sim_utils.UsdFileCfg(
                usd_path=(OBJECTS_ROOT / "bamboo_basket/bamboo_basket.usd").as_posix()
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(0.0, -3.61, 0.7),
                rot=(0.0, 0.0, -0.7071068, 0.7071068),
            ),
        ),
    }
)
