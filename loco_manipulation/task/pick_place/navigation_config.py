"""YAML-backed settings for pick-place navigation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_NAVIGATION_CONFIG_PATH = Path(__file__).with_name("navigation.yaml")


@dataclass(frozen=True)
class NavigationSettings:
    """Occupancy-map and planner settings."""

    map_slice_world_z: float
    map_min_world_xy: tuple[float, float]
    map_max_world_xy: tuple[float, float]
    resolution: float
    safety_margin: float
    waypoint_spacing: float
    max_endpoint_snap_distance: float

    def __post_init__(self) -> None:
        for name, value, length in (
            ("map_min_world_xy", self.map_min_world_xy, 2),
            ("map_max_world_xy", self.map_max_world_xy, 2),
        ):
            if len(value) != length or not all(math.isfinite(item) for item in value):
                raise ValueError(f"{name} must contain {length} finite numbers")
        limits = {
            "resolution": (self.resolution, False),
            "safety_margin": (self.safety_margin, True),
            "waypoint_spacing": (self.waypoint_spacing, False),
            "max_endpoint_snap_distance": (self.max_endpoint_snap_distance, True),
        }
        for name, (value, allow_zero) in limits.items():
            invalid = not math.isfinite(value) or value < 0.0
            if invalid or (not allow_zero and value == 0.0):
                qualifier = "non-negative" if allow_zero else "positive"
                raise ValueError(f"{name} must be finite and {qualifier}")
        if not math.isfinite(self.map_slice_world_z):
            raise ValueError("map_slice_world_z must be finite")
        if any(
            lo >= hi for lo, hi in zip(self.map_min_world_xy, self.map_max_world_xy)
        ):
            raise ValueError("each map minimum must be smaller than its maximum")

    @classmethod
    def from_yaml(
        cls, path: str | Path = DEFAULT_NAVIGATION_CONFIG_PATH
    ) -> NavigationSettings:
        """Create settings from the project's YAML schema."""

        config_path = Path(path).expanduser().resolve()
        with config_path.open("r", encoding="utf-8") as stream:
            payload = yaml.safe_load(stream)
        map_cfg, plan_cfg = payload["map"], payload["planning"]
        return cls(
            map_slice_world_z=float(map_cfg["slice_world_z"]),
            map_min_world_xy=tuple(map(float, map_cfg["min_world_xy"])),
            map_max_world_xy=tuple(map(float, map_cfg["max_world_xy"])),
            resolution=float(map_cfg["resolution"]),
            safety_margin=float(plan_cfg["safety_margin"]),
            waypoint_spacing=float(plan_cfg["waypoint_spacing"]),
            max_endpoint_snap_distance=float(plan_cfg["max_endpoint_snap_distance"]),
        )
