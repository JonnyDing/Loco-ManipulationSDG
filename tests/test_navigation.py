"""Pure-Python regression tests for navigation planning."""

from __future__ import annotations

import unittest

import numpy as np

from loco_manipulation.task.pick_place.navigation import (
    RobotFootprint,
    astar,
    plan_navigation_path,
)
from loco_manipulation.task.pick_place.navigation_config import NavigationSettings


class FakeMap:
    resolution = 1.0

    def __init__(self, free: np.ndarray, origin: tuple[float, float] = (0.0, 0.0)):
        self.free = free
        self.origin = origin

    def buffered(self, distance_pixels: int) -> FakeMap:
        self.last_buffer = distance_pixels
        return self

    def freespace_mask(self) -> np.ndarray:
        return self.free.copy()

    def pixel_to_world_numpy(self, points: np.ndarray) -> np.ndarray:
        x = points[:, 0] + self.origin[0]
        y = self.free.shape[0] - points[:, 1] + self.origin[1]
        return np.column_stack((x, y))

    def world_to_pixel_numpy(self, points: np.ndarray) -> np.ndarray:
        x = points[:, 0] - self.origin[0]
        y = self.free.shape[0] - (points[:, 1] - self.origin[1])
        return np.column_stack((x, y))


class AStarTests(unittest.TestCase):
    def test_finds_shortest_open_grid_path(self) -> None:
        path = astar(np.ones((5, 5), dtype=bool), (0, 0), (4, 4))
        np.testing.assert_array_equal(path, np.arange(5)[:, None].repeat(2, axis=1))

    def test_does_not_cut_blocked_diagonal_corner(self) -> None:
        free = np.ones((3, 3), dtype=bool)
        free[0, 1] = free[1, 0] = False
        with self.assertRaisesRegex(RuntimeError, "unreachable"):
            astar(free, (0, 0), (2, 2))

    def test_rejects_out_of_bounds_endpoint(self) -> None:
        with self.assertRaisesRegex(ValueError, "in-bounds"):
            astar(np.ones((2, 2), dtype=bool), (0, 0), (2, 1))


class ConfigTests(unittest.TestCase):
    def test_loads_project_yaml(self) -> None:
        settings = NavigationSettings.from_yaml()
        self.assertEqual(settings.map_slice_world_z, 0.5)
        self.assertEqual(settings.map_min_world_xy, (-4.0, -6.0))


class PlannerTests(unittest.TestCase):
    def test_preserves_endpoint_poses_and_uses_configured_clearance(self) -> None:
        grid = FakeMap(np.ones((8, 8), dtype=bool))
        settings = NavigationSettings(
            map_slice_world_z=0.5,
            map_min_world_xy=(0.0, 0.0),
            map_max_world_xy=(8.0, 8.0),
            resolution=1.0,
            safety_margin=0.25,
            waypoint_spacing=0.5,
            max_endpoint_snap_distance=1.0,
        )
        plan = plan_navigation_path(
            grid,
            settings,
            RobotFootprint((0.5, 0.5, 1.8), 0.5),
            start_world_xyz=(1.0, 7.0, 0.0),
            start_rot_wxyz=(2.0, 0.0, 0.0, 0.0),
            goal_world_xyz=(6.0, 2.0, 0.0),
            goal_rot_wxyz=(1.0, 0.0, 0.0, -1.0),
        )
        self.assertEqual(grid.last_buffer, 1)
        self.assertAlmostEqual(plan.clearance_m, 0.75)
        np.testing.assert_allclose(plan.waypoints_xyz_wxyz[0, 3:], (1, 0, 0, 0))
        np.testing.assert_allclose(
            plan.waypoints_xyz_wxyz[-1, 3:],
            (2**-0.5, 0, 0, -(2**-0.5)),
        )


if __name__ == "__main__":
    unittest.main()
