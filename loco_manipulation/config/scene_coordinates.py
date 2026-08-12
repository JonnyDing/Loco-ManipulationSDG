"""Canonical Isaac World poses for the captured Lab scene.

The values mirror the audited scene definition in the sibling HSDE project:
``configs/scenes/tabletop_pick_place.yaml``. HSDE stores quaternions as WXYZ;
Isaac Lab 3.0 configuration objects use XYZW, so rotations are converted here.
"""

from __future__ import annotations

from typing import TypeAlias

Vec3: TypeAlias = tuple[float, float, float]
QuatXyzw: TypeAlias = tuple[float, float, float, float]

# The collision mesh floor is authored 0.20 m below its local origin. Raising
# the USD root puts that floor at Isaac World Z=0, matching the robot and ground.
LAB_WORLD_POS: Vec3 = (0.0, 0.0, 0.20)
LAB_WORLD_ROT_XYZW: QuatXyzw = (0.0, 0.0, 0.0, 1.0)

# These are absolute Isaac World poses, not Lab-local coordinates. HSDE's
# ``at_position`` placement relation intentionally resolves them as world poses.
CUP_WORLD_POS: Vec3 = (-0.27, -3.54, 0.64)
CUP_WORLD_ROT_XYZW: QuatXyzw = (1.0, 0.0, 0.0, 0.0)
BAMBOO_BASKET_WORLD_POS: Vec3 = (0.0, -3.61, 0.73)
BAMBOO_BASKET_WORLD_ROT_XYZW: QuatXyzw = (
    0.0,
    0.0,
    -0.7071068,
    0.7071068,
)
