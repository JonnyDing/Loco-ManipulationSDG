"""Serialization and previews for navigation results."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

from PIL import ImageDraw

from .navigation import NavigationPlan

if TYPE_CHECKING:
    from .occupancy import OccupancyMapResult


def save_navigation_artifacts(
    output_dir: str | Path,
    occupancy: OccupancyMapResult,
    plan: NavigationPlan,
) -> dict[str, Path]:
    """Write the ROS map, path preview, and machine-readable waypoints."""

    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    occupancy.occupancy_map.save_ros(str(output))
    plan.buffered_occupancy_map.ros_image().save(output / "map_buffered.png")

    preview = plan.buffered_occupancy_map.ros_image().convert("RGB")
    pixels = plan.buffered_occupancy_map.world_to_pixel_numpy(
        plan.waypoints_xyz_wxyz[:, :2]
    )
    line = [tuple(map(round, point)) for point in pixels]
    draw = ImageDraw.Draw(preview)
    if len(line) > 1:
        draw.line(line, fill=(32, 128, 255), width=3, joint="curve")
    for point, color in ((line[0], (0, 180, 0)), (line[-1], (220, 30, 30))):
        x, y = point
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=color)
    preview.save(output / "path.png")

    payload = {
        "coordinate_convention": {
            "position": "Isaac World XYZ in metres",
            "rotation": "Isaac World WXYZ quaternion",
        },
        "requested_start_world_xyz": plan.requested_start_world_xyz,
        "start_rot_wxyz": plan.start_rot_wxyz,
        "requested_goal_world_xyz": plan.requested_goal_world_xyz,
        "planned_start_world_xyz": plan.planned_start_world_xyz,
        "planned_goal_world_xyz": plan.planned_goal_world_xyz,
        "goal_rot_wxyz": plan.goal_rot_wxyz,
        "start_was_snapped": plan.start_was_snapped,
        "goal_was_snapped": plan.goal_was_snapped,
        "clearance_m": plan.clearance_m,
        "map": {
            "resolution_m_per_pixel": occupancy.occupancy_map.resolution,
            "dimensions_xy": occupancy.dimensions_xy,
            "min_bound_world_xyz": occupancy.min_bound_world_xyz,
            "max_bound_world_xyz": occupancy.max_bound_world_xyz,
        },
        "waypoints_xyz_wxyz": plan.waypoints_xyz_wxyz.tolist(),
    }
    (output / "path.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    return {
        name: output / filename
        for name, filename in {
            "map_png": "map.png",
            "map_yaml": "map.yaml",
            "map_buffered_png": "map_buffered.png",
            "path_png": "path.png",
            "path_json": "path.json",
        }.items()
    }
