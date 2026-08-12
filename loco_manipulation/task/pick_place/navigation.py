"""Pure grid planning for pick-place navigation."""

from __future__ import annotations

import heapq
import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from typing import Protocol, Self

import numpy as np

from .navigation_config import NavigationSettings

Vec3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Cell = tuple[int, int]


class GridMap(Protocol):
    """Subset of the MobilityGen occupancy-map API used by the planner."""

    resolution: float

    def buffered(self, distance_pixels: int) -> Self: ...
    def freespace_mask(self) -> np.ndarray: ...
    def ros_image(self): ...
    def pixel_to_world_numpy(self, points: np.ndarray) -> np.ndarray: ...
    def world_to_pixel_numpy(self, points: np.ndarray) -> np.ndarray: ...


@dataclass(frozen=True)
class RobotFootprint:
    size_xyz: Vec3
    radius: float


@dataclass(frozen=True)
class NavigationPlan:
    waypoints_xyz_wxyz: np.ndarray
    buffered_occupancy_map: GridMap
    clearance_m: float
    requested_start_world_xyz: Vec3
    start_rot_wxyz: Quaternion
    requested_goal_world_xyz: Vec3
    planned_start_world_xyz: Vec3
    planned_goal_world_xyz: Vec3
    goal_rot_wxyz: Quaternion
    start_was_snapped: bool
    goal_was_snapped: bool


def _pose(
    position: Sequence[float], rotation: Sequence[float], name: str
) -> tuple[Vec3, Quaternion]:
    xyz = np.asarray(position, dtype=float)
    wxyz = np.asarray(rotation, dtype=float)
    finite = np.all(np.isfinite(xyz)) and np.all(np.isfinite(wxyz))
    if xyz.shape != (3,) or wxyz.shape != (4,) or not finite:
        raise ValueError(f"{name} pose requires finite XYZ and WXYZ values")
    norm = float(np.linalg.norm(wxyz))
    if norm == 0.0:
        raise ValueError(f"{name}_rot_wxyz must be non-zero")
    return tuple(map(float, xyz)), tuple(map(float, wxyz / norm))


def _nearest_free(
    grid: GridMap, world_xyz: Vec3, max_distance: float
) -> tuple[Cell, Vec3, bool]:
    point_xy = np.asarray([world_xyz[:2]], dtype=float)
    pixel_xy = grid.world_to_pixel_numpy(point_xy)[0]
    requested = (round(pixel_xy[1]), round(pixel_xy[0]))
    free = grid.freespace_mask()
    row, col = requested
    if 0 <= row < free.shape[0] and 0 <= col < free.shape[1] and free[row, col]:
        return requested, world_xyz, False

    cells = np.argwhere(free)
    if not len(cells):
        raise RuntimeError("occupancy map contains no navigable cells")
    nearest = cells[np.argmin(np.sum((cells - requested) ** 2, axis=1))]
    cell = int(nearest[0]), int(nearest[1])
    nearest_xy = grid.pixel_to_world_numpy(np.asarray([[cell[1], cell[0]]]))[0]
    distance = float(np.linalg.norm(nearest_xy - point_xy[0]))
    if distance > max_distance:
        raise RuntimeError(
            f"endpoint {world_xyz[:2]} is blocked; nearest free point is "
            f"{tuple(nearest_xy)} ({distance:.3f} m away)"
        )
    return cell, (float(nearest_xy[0]), float(nearest_xy[1]), world_xyz[2]), True


def astar(free: np.ndarray, start: Cell, goal: Cell) -> np.ndarray:
    """Return an optimal 8-connected path without diagonal corner cutting."""

    if free.ndim != 2 or any(
        not (0 <= row < free.shape[0] and 0 <= col < free.shape[1])
        for row, col in (start, goal)
    ):
        raise ValueError("A* requires a 2D grid with in-bounds endpoints")
    if not free[start] or not free[goal]:
        raise RuntimeError("A* endpoints must be in free space")
    height, width = free.shape
    cost = np.full(free.shape, np.inf)
    cost[start] = 0.0
    parent = np.full((*free.shape, 2), -1, dtype=np.int32)
    queue = [(math.dist(start, goal), 0.0, start)]
    moves = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(tuple(parent[path[-1]]))
            return np.asarray(path[::-1], dtype=np.int32)
        if current_cost != cost[current]:
            continue
        row, col = current
        for dr, dc in moves:
            neighbor = row + dr, col + dc
            nr, nc = neighbor
            if not (0 <= nr < height and 0 <= nc < width and free[neighbor]):
                continue
            if dr and dc and (not free[row + dr, col] or not free[row, col + dc]):
                continue
            candidate = current_cost + math.hypot(dr, dc)
            if candidate >= cost[neighbor]:
                continue
            cost[neighbor] = candidate
            parent[neighbor] = current
            heapq.heappush(
                queue, (candidate + math.dist(neighbor, goal), candidate, neighbor)
            )
    raise RuntimeError(f"goal cell {goal} is unreachable from {start}")


def _turning_points(path: np.ndarray) -> np.ndarray:
    if len(path) < 3:
        return path
    directions = np.diff(path, axis=0)
    turns = np.flatnonzero(np.any(directions[1:] != directions[:-1], axis=1)) + 1
    return path[np.r_[0, turns, len(path) - 1]]


def _resample(path: np.ndarray, spacing: float) -> np.ndarray:
    pieces = []
    for start, end in pairwise(path):
        count = max(1, math.ceil(float(np.linalg.norm(end - start)) / spacing))
        pieces.append(np.linspace(start, end, count, endpoint=False))
    return np.concatenate([*pieces, path[-1:]]) if pieces else path.copy()


def _waypoints(
    path_xy: np.ndarray,
    start: tuple[Vec3, Quaternion],
    goal: tuple[Vec3, Quaternion],
) -> np.ndarray:
    if len(path_xy) == 1:
        path_xy = np.repeat(path_xy, 2, axis=0)
    start_xyz, start_rot = start
    goal_xyz, goal_rot = goal
    delta = np.diff(path_xy, axis=0)
    yaw = np.r_[np.arctan2(delta[:, 1], delta[:, 0]), 0.0]
    poses = np.zeros((len(path_xy), 7))
    poses[:, :2] = path_xy
    poses[:, 2] = np.linspace(start_xyz[2], goal_xyz[2], len(path_xy))
    poses[:, 3] = np.cos(yaw / 2)
    poses[:, 6] = np.sin(yaw / 2)
    poses[0, 3:], poses[-1, 3:] = start_rot, goal_rot
    return poses


def plan_navigation_path(
    occupancy_map: GridMap,
    settings: NavigationSettings,
    footprint: RobotFootprint,
    *,
    start_world_xyz: Sequence[float],
    start_rot_wxyz: Sequence[float],
    goal_world_xyz: Sequence[float],
    goal_rot_wxyz: Sequence[float],
) -> NavigationPlan:
    """Plan a collision-free A* route between two Isaac World poses."""

    start = _pose(start_world_xyz, start_rot_wxyz, "start")
    goal = _pose(goal_world_xyz, goal_rot_wxyz, "goal")
    clearance = footprint.radius + settings.safety_margin
    buffer_pixels = math.ceil(clearance / occupancy_map.resolution)
    buffered = occupancy_map.buffered(buffer_pixels) if buffer_pixels else occupancy_map
    start_cell, planned_start, start_snapped = _nearest_free(
        buffered, start[0], settings.max_endpoint_snap_distance
    )
    goal_cell, planned_goal, goal_snapped = _nearest_free(
        buffered, goal[0], settings.max_endpoint_snap_distance
    )

    raw_cells = astar(buffered.freespace_mask(), start_cell, goal_cell)
    controls = occupancy_map.pixel_to_world_numpy(_turning_points(raw_cells)[:, ::-1])
    controls[[0, -1]] = planned_start[:2], planned_goal[:2]
    poses = _waypoints(
        _resample(controls, settings.waypoint_spacing),
        (planned_start, start[1]),
        (planned_goal, goal[1]),
    )
    return NavigationPlan(
        waypoints_xyz_wxyz=poses,
        buffered_occupancy_map=buffered,
        clearance_m=clearance,
        requested_start_world_xyz=start[0],
        start_rot_wxyz=start[1],
        requested_goal_world_xyz=goal[0],
        planned_start_world_xyz=planned_start,
        planned_goal_world_xyz=planned_goal,
        goal_rot_wxyz=goal[1],
        start_was_snapped=start_snapped,
        goal_was_snapped=goal_snapped,
    )
