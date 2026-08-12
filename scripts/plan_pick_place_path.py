"""Generate the pick-place occupancy map and collision-free navigation path."""

from __future__ import annotations

import argparse
import asyncio
import sys
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

parser = argparse.ArgumentParser(
    description="Generate a collider occupancy map and pick-place navigation waypoints."
)
parser.add_argument(
    "--output_dir",
    type=Path,
    default=REPOSITORY_ROOT / "outputs/pick_place/navigation",
)
parser.add_argument(
    "--navigation_cfg",
    type=Path,
    default=REPOSITORY_ROOT / "loco_manipulation/task/pick_place/navigation.yaml",
)
parser.add_argument("--start_world_xyz", type=float, nargs=3, default=(0.0, 0.0, 0.0))
parser.add_argument(
    "--start_rot_wxyz",
    type=float,
    nargs=4,
    default=(1.0, 0.0, 0.0, 0.0),
)
parser.add_argument(
    "--goal_world_xyz",
    type=float,
    nargs=3,
    default=(-0.1, -2.65, 0.0),
)
parser.add_argument(
    "--goal_rot_wxyz",
    type=float,
    nargs=4,
    default=(0.7071, 0.0, 0.0, -0.7071),
)
parser.add_argument("--load_timeout", type=float, default=60.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

output_dir = args_cli.output_dir.expanduser().resolve()
output_dir.mkdir(parents=True, exist_ok=True)
status_path = output_dir / "status.txt"
error_path = output_dir / "error.txt"
error_path.unlink(missing_ok=True)
status_path.write_text("starting Isaac Sim\n", encoding="utf-8")
print(f"[navigation] output_dir={output_dir}", flush=True)

default_excepthook = sys.excepthook


def record_failure(exc_type, exc, tb) -> None:
    error_path.write_text(
        "".join(traceback.format_exception(exc_type, exc, tb)), encoding="utf-8"
    )
    status_path.write_text(f"failed; see {error_path}\n", encoding="utf-8")
    print(f"[navigation] failed; traceback saved to {error_path}", flush=True)
    default_excepthook(exc_type, exc, tb)


sys.excepthook = record_failure
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Kit-dependent imports must follow AppLauncher construction.
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.usd
from isaaclab.envs import ManagerBasedEnv

status_path.write_text("enabling Isaac Sim extensions\n", encoding="utf-8")
if not app_utils.enable_extension("isaacsim.replicator.experimental.mobility_gen"):
    raise RuntimeError("failed to enable isaacsim.replicator.experimental.mobility_gen")

from loco_manipulation.env import LocoManipulationEnvCfg
from loco_manipulation.task.pick_place.navigator import PickPlaceNavigator


async def wait_for_stage_loading_async() -> None:
    """Keep Kit responsive until all referenced USD assets are ready."""

    async with asyncio.timeout(args_cli.load_timeout):
        await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()


def main() -> None:
    """Build the scene, generate its occupancy grid, and plan the route."""

    status_path.write_text("loading Isaac Lab scene\n", encoding="utf-8")
    env_cfg = LocoManipulationEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.scene.camera = None
    env_cfg.sim.device = args_cli.device
    env = ManagerBasedEnv(cfg=env_cfg)
    try:
        simulation_app.run_coroutine(wait_for_stage_loading_async())
        env.reset()

        navigator = PickPlaceNavigator.from_yaml(args_cli.navigation_cfg)
        stage = omni.usd.get_context().get_stage()
        env_root = "/World/envs/env_0"
        robot_root = f"{env_root}/Robot"
        status_path.write_text("planning navigation path\n", encoding="utf-8")
        result = navigator.plan(
            stage,
            robot_root=robot_root,
            excluded_prim_roots=(
                f"{env_root}/Cup",
                f"{env_root}/BambooBasket",
            ),
            start_world_xyz=args_cli.start_world_xyz,
            start_rot_wxyz=args_cli.start_rot_wxyz,
            goal_world_xyz=args_cli.goal_world_xyz,
            goal_rot_wxyz=args_cli.goal_rot_wxyz,
        )
        artifacts = navigator.save(output_dir, result)

        print(
            f"[navigation] grid={result.occupancy.dimensions_xy} "
            f"resolution={navigator.settings.resolution:.3f}m "
            f"clearance={result.plan.clearance_m:.3f}m "
            f"waypoints={len(result.plan.waypoints_xyz_wxyz)}"
        )
        print(
            f"[navigation] footprint={result.footprint.size_xyz} "
            f"radius={result.footprint.radius:.3f}m"
        )
        for name, path in artifacts.items():
            print(f"[navigation] {name}={path}", flush=True)
        status_path.write_text("complete\n", encoding="utf-8")
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
