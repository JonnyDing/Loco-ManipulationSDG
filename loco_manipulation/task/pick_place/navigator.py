"""Reusable pick-place occupancy-map and path-planning facade."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from pxr import Usd

from .artifacts import save_navigation_artifacts
from .navigation import (
    NavigationPlan,
    RobotFootprint,
    plan_navigation_path,
)
from .navigation_config import NavigationSettings
from .occupancy import (
    OccupancyMapResult,
    compute_robot_footprint,
    generate_stage_occupancy_map,
)


@dataclass(frozen=True)
class NavigationResult:
    occupancy: OccupancyMapResult
    plan: NavigationPlan
    footprint: RobotFootprint


class PickPlaceNavigator:
    """Plan and serialize navigation without owning the Isaac Sim application."""

    def __init__(self, settings: NavigationSettings):
        self.settings = settings

    @classmethod
    def from_yaml(cls, path: str | Path) -> PickPlaceNavigator:
        return cls(NavigationSettings.from_yaml(path))

    def plan(
        self,
        stage: Usd.Stage,
        *,
        robot_root: str,
        excluded_prim_roots: Sequence[str] = (),
        start_world_xyz: Sequence[float],
        start_rot_wxyz: Sequence[float],
        goal_world_xyz: Sequence[float],
        goal_rot_wxyz: Sequence[float],
    ) -> NavigationResult:
        footprint = compute_robot_footprint(stage, robot_root)
        occupancy = generate_stage_occupancy_map(
            stage,
            self.settings,
            excluded_prim_roots=(robot_root, *excluded_prim_roots),
        )
        plan = plan_navigation_path(
            occupancy.occupancy_map,
            self.settings,
            footprint,
            start_world_xyz=start_world_xyz,
            start_rot_wxyz=start_rot_wxyz,
            goal_world_xyz=goal_world_xyz,
            goal_rot_wxyz=goal_rot_wxyz,
        )
        return NavigationResult(occupancy, plan, footprint)

    @staticmethod
    def save(output_dir: str | Path, result: NavigationResult) -> dict[str, Path]:
        return save_navigation_artifacts(output_dir, result.occupancy, result.plan)
