"""Run cloned loco-manipulation environments with Isaac Lab."""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

parser = argparse.ArgumentParser(description="Load the Lab/G1 Isaac Lab environment.")
parser.add_argument(
    "--max_steps", type=int, default=0, help="Zero runs until the app closes."
)
parser.add_argument(
    "--load_timeout", type=float, default=60.0, help="USD load timeout in seconds."
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.max_steps < 0:
    parser.error("--max_steps must be non-negative")
if args_cli.load_timeout <= 0.0:
    parser.error("--load_timeout must be positive")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Isaac Lab environment modules must be imported after AppLauncher starts Kit.
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import torch
from isaaclab.envs import ManagerBasedEnv

from loco_manipulation.env import LocoManipulationEnvCfg


async def wait_for_stage_loading_async() -> None:
    """Wait for all USD dependencies without blocking Kit updates."""

    async with asyncio.timeout(args_cli.load_timeout):
        await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()


def main() -> None:
    """Run the environment without overriding robot root or joint state."""

    env_cfg = LocoManipulationEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.sim.device = args_cli.device
    if not args_cli.enable_cameras:
        env_cfg.scene.camera = None
    env = ManagerBasedEnv(cfg=env_cfg)
    try:
        simulation_app.run_coroutine(wait_for_stage_loading_async())
        env.reset()

        robot, objects = env.scene["robot"], env.scene["objects"]
        print(
            f"[ready] envs={env.num_envs} joints={robot.num_joints} "
            f"objects={objects.num_bodies} device={env.device}"
        )

        empty_action = torch.zeros_like(env.action_manager.action)
        step_count = 0
        while simulation_app.is_running():
            with torch.inference_mode():
                env.step(empty_action)
            step_count += 1
            if args_cli.max_steps and step_count >= args_cli.max_steps:
                break
        print(f"[ready] stopped steps={step_count}")
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
