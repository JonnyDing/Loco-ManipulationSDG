"""Occupancy maps projected from authored USD collision geometry."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import isaacsim.core.experimental.utils.bounds as bounds_utils
import numpy as np
from isaacsim.replicator.experimental.mobility_gen import OccupancyMap
from pxr import Usd, UsdGeom, UsdPhysics

from .navigation import RobotFootprint, Vec3
from .navigation_config import NavigationSettings


@dataclass(frozen=True)
class OccupancyMapResult:
    occupancy_map: OccupancyMap
    min_bound_world_xyz: Vec3
    max_bound_world_xyz: Vec3
    dimensions_xy: tuple[int, int]


def compute_robot_footprint(stage: Usd.Stage, robot_root: str) -> RobotFootprint:
    """Compute a conservative circular footprint from enabled robot colliders."""

    root = stage.GetPrimAtPath(robot_root)
    if not root or not root.IsValid():
        raise ValueError(f"robot prim does not exist: {robot_root}")
    colliders = [
        prim
        for prim in Usd.PrimRange(root)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
        and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
    ]
    if not colliders:
        raise RuntimeError(f"robot has no enabled colliders: {robot_root}")
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide],
        useExtentsHint=True,
    )
    bounds = np.asarray(
        bounds_utils.compute_combined_aabb(colliders, bbox_cache=cache), dtype=float
    )
    if bounds.shape != (6,) or not np.all(np.isfinite(bounds)):
        raise RuntimeError(f"robot collider bounds are invalid: {robot_root}: {bounds}")
    size = bounds[3:] - bounds[:3]
    if np.any(size <= 0):
        raise RuntimeError(f"robot collider bounds are empty: {robot_root}: {bounds}")
    half_x, half_y = size[:2] / 2
    return RobotFootprint(tuple(map(float, size)), float(math.hypot(half_x, half_y)))


def generate_stage_occupancy_map(
    stage: Usd.Stage,
    settings: NavigationSettings,
    *,
    excluded_prim_roots: Sequence[str] = (),
) -> OccupancyMapResult:
    """Rasterize enabled collider triangles crossing the configured Z slice."""

    min_x, min_y = settings.map_min_world_xy
    max_x, max_y = settings.map_max_world_xy
    width = math.ceil((max_x - min_x) / settings.resolution)
    height = math.ceil((max_y - min_y) / settings.resolution)
    occupied = np.zeros((height, width), dtype=np.uint8)
    xforms = UsdGeom.XformCache(Usd.TimeCode.Default())
    excluded = tuple(path.rstrip("/") for path in excluded_prim_roots)

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if (
            not prim.IsA(UsdGeom.Mesh)
            or not prim.HasAPI(UsdPhysics.CollisionAPI)
            or any(path == root or path.startswith(root + "/") for root in excluded)
        ):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        mesh = UsdGeom.Mesh(prim)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        if not len(counts):
            continue
        if np.any(counts != 3):
            raise RuntimeError(f"collision mesh must be triangulated: {path}")
        local_points = np.asarray(mesh.GetPointsAttr().Get())
        points = np.c_[local_points, np.ones(len(local_points))]
        points = (points @ np.asarray(xforms.GetLocalToWorldTransform(prim)))[:, :3]
        triangles = points[
            np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        ]
        z = settings.map_slice_world_z
        triangles = triangles[
            (triangles[:, :, 2].min(1) <= z) & (triangles[:, :, 2].max(1) >= z)
        ]
        pixels = np.rint(
            (triangles[:, :, :2] - (min_x, min_y)) / settings.resolution * (1, -1)
            + (0, height)
        ).astype(np.int32)
        if len(pixels):
            cv2.fillPoly(occupied, list(pixels), 1)

    if not occupied.any():
        raise RuntimeError(
            "no enabled collider triangles cross the configured map Z slice"
        )
    min_bound = (min_x, min_y, settings.map_slice_world_z)
    max_bound = (
        min_x + width * settings.resolution,
        min_y + height * settings.resolution,
        settings.map_slice_world_z,
    )
    occupancy = OccupancyMap.from_masks(
        ~occupied.astype(bool),
        occupied.astype(bool),
        settings.resolution,
        (min_x, min_y, 0.0),
    )
    return OccupancyMapResult(occupancy, min_bound, max_bound, (width, height))
