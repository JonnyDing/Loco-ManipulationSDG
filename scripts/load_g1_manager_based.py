"""Load Unitree G1 through Isaac Lab's manager-based workflow."""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))


parser = argparse.ArgumentParser(
    description="Load Unitree G1 in an Isaac Lab ManagerBasedEnv."
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of cloned G1 environments."
)
parser.add_argument(
    "--max_steps",
    type=int,
    default=0,
    help="Stop after this many environment steps; zero runs until the app is closed.",
)
parser.add_argument(
    "--load_timeout",
    type=float,
    default=60.0,
    help="Maximum seconds to wait for USD dependencies to finish loading.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.num_envs < 1:
    parser.error("--num_envs must be at least 1")
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

from loco_manipulation.config.g1_env_cfg import (
    G1EnvCfg,
    resolve_g1_usd_path,
)


async def wait_for_stage_loading_async() -> None:
    """Wait for USD dependencies using Kit's supported asynchronous update API."""

    loop = asyncio.get_running_loop()
    deadline = loop.time() + args_cli.load_timeout

    # Always yield at least one frame so startup and USD callbacks can run.
    await app_utils.update_app_async()
    while stage_utils.is_stage_loading():
        if loop.time() >= deadline:
            raise TimeoutError(
                f"Timed out after {args_cli.load_timeout:.1f}s waiting for G1 USD dependencies"
            )
        await app_utils.update_app_async()


def create_environment() -> ManagerBasedEnv:
    """Create the environment synchronously, then await outstanding USD work."""

    env_cfg = G1EnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device

    # IsaacLab 3.0 owns ManagerBasedEnv synchronously. Keeping construction and
    # reset outside Kit's coroutine prevents GUI event-loop re-entry at shutdown.
    env = ManagerBasedEnv(cfg=env_cfg)
    try:
        simulation_app.run_coroutine(wait_for_stage_loading_async())
        env.reset()
        return env
    except BaseException:
        env.close()
        raise


def main() -> None:
    """Load the environment, then advance it with empty actions."""

    env: ManagerBasedEnv | None = None
    try:
        env = create_environment()

        robot = env.scene["robot"]
        print(f"[ready] asset={resolve_g1_usd_path()}")
        print(
            f"[ready] envs={env.num_envs} bodies={robot.num_bodies} "
            f"joints={robot.num_joints} device={env.device}"
        )

        empty_action = torch.zeros_like(env.action_manager.action)
        step_count = 0
        while simulation_app.is_running():
            with torch.inference_mode():
                env.step(empty_action)
            step_count += 1
            if args_cli.max_steps and step_count >= args_cli.max_steps:
                break
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
