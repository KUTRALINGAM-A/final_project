"""Shared coordinate-conversion utilities for EUN (Y-up) Webots worlds.

The Webots world uses coordinateSystem "EUN":
    X = East, Y = Up, Z = North.
Ground plane: X-Z
Vertical axis: Y

All helpers operate on 3-element sequences [x, y, z].
"""
from __future__ import annotations
import math
from typing import Sequence, Tuple

# ── Axis indices ──────────────────────────────────────────────────────
IDX_EAST  = 0  # X
IDX_UP    = 1  # Y
IDX_NORTH = 2  # Z


def world_position(gps_values: Sequence[float]) -> Tuple[float, float, float]:
    """Return (x, y, z) from a GPS reading (already in world frame for EUN)."""
    return (gps_values[0], gps_values[1], gps_values[2])


def horizontal_position(pos: Sequence[float]) -> Tuple[float, float]:
    """Return the (x, z) ground-plane position (east, north)."""
    return (pos[IDX_EAST], pos[IDX_NORTH])


def altitude(pos: Sequence[float]) -> float:
    """Return the vertical component Y."""
    return pos[IDX_UP]


def horizontal_distance(pos_a: Sequence[float], pos_b: Sequence[float]) -> float:
    """Euclidean distance in the X-Z ground plane."""
    dx = pos_a[IDX_EAST]  - pos_b[IDX_EAST]
    dz = pos_a[IDX_NORTH] - pos_b[IDX_NORTH]
    return math.sqrt(dx * dx + dz * dz)


def vertical_separation(pos_a: Sequence[float], pos_b: Sequence[float]) -> float:
    """Signed vertical separation (a.y - b.y)."""
    return pos_a[IDX_UP] - pos_b[IDX_UP]


def distance_3d(pos_a: Sequence[float], pos_b: Sequence[float]) -> float:
    """Full 3-D Euclidean distance."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos_a, pos_b)))


def ground_projection(pos: Sequence[float], ground_y: float = 0.0) -> Tuple[float, float, float]:
    """Project a 3-D position onto the ground plane at *ground_y*."""
    return (pos[IDX_EAST], ground_y, pos[IDX_NORTH])


def rendezvous_point_above(
    ugv_pos: Sequence[float],
    vertical_separation_m: float,
) -> Tuple[float, float, float]:
    """UAV rendezvous target: directly above the UGV at a safe height."""
    return (
        ugv_pos[IDX_EAST],
        ugv_pos[IDX_UP] + vertical_separation_m,
        ugv_pos[IDX_NORTH],
    )


def rendezvous_point_below(
    uav_pos: Sequence[float],
    ground_y: float = 0.0,
) -> Tuple[float, float, float]:
    """UGV rendezvous target: ground projection of the UAV."""
    return ground_projection(uav_pos, ground_y)


def heading_to_target(
    current_pos: Sequence[float],
    target_pos: Sequence[float],
) -> float:
    """Heading angle in radians from current to target on the X-Z plane.

    Returns angle in [-pi, pi], measured from +Z (north) axis, positive
    towards +X (east) — i.e. conventional compass bearing in EUN.
    """
    dx = target_pos[IDX_EAST]  - current_pos[IDX_EAST]
    dz = target_pos[IDX_NORTH] - current_pos[IDX_NORTH]
    return math.atan2(dx, dz)


def yaw_from_imu(roll: float, pitch: float, yaw: float) -> float:
    """Extract yaw from InertialUnit (getRollPitchYaw) in EUN.

    In Webots EUN the InertialUnit returns [roll, pitch, yaw] where yaw
    is the rotation around the Y (up) axis.
    """
    return yaw


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* between *lo* and *hi*."""
    return max(lo, min(hi, value))


def angle_diff(a: float, b: float) -> float:
    """Shortest signed difference between two angles in radians."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d
