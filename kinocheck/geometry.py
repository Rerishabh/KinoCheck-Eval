"""Geometry primitives for grid obstacles and polyline clearance checks."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from numbers import Real
from typing import Any

from shapely.geometry import LineString, Point, Polygon

from .tolerances import FLOAT_ABS_TOL


def _validate_finite_number(value: Any, name: str) -> float:
    if not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _validate_point(point: Sequence[Real], name: str) -> tuple[float, float]:
    if isinstance(point, (str, bytes)) or len(point) != 2:
        raise ValueError(f"{name} must contain exactly two coordinates")
    x = _validate_finite_number(point[0], f"{name}[0]")
    y = _validate_finite_number(point[1], f"{name}[1]")
    return x, y


def create_obstacle_boxes(
    grid: Sequence[Sequence[int]], cell_size: Real
) -> list[Polygon]:
    """Create one axis-aligned box for every occupied grid cell.

    ``grid[i][j]`` occupies ``[i * cell_size, (i + 1) * cell_size]`` on
    the x-axis and ``[j * cell_size, (j + 1) * cell_size]`` on the y-axis.
    """
    size = _validate_finite_number(cell_size, "cell_size")
    if size <= 0:
        raise ValueError("cell_size must be positive")
    if not grid or not all(grid):
        raise ValueError("grid must be non-empty")

    row_length = len(grid[0])
    if row_length == 0 or any(len(column) != row_length for column in grid):
        raise ValueError("grid must be rectangular and non-empty")

    obstacles: list[Polygon] = []
    for i, column in enumerate(grid):
        for j, value in enumerate(column):
            if value not in (0, 1):
                raise ValueError("grid values must be 0 or 1")
            if value == 1:
                obstacles.append(
                    Polygon(
                        [
                            (i * size, j * size),
                            ((i + 1) * size, j * size),
                            ((i + 1) * size, (j + 1) * size),
                            (i * size, (j + 1) * size),
                        ]
                    )
                )
    return obstacles


def segment_min_distance(
    p1: Sequence[Real],
    p2: Sequence[Real],
    obstacles: Iterable[Any],
) -> float:
    """Return the minimum distance from a segment (or point) to obstacles."""
    start = _validate_point(p1, "p1")
    end = _validate_point(p2, "p2")
    geometries = list(obstacles)
    if not geometries:
        return math.inf

    segment = Point(start) if start == end else LineString([start, end])
    return min(segment.distance(obstacle) for obstacle in geometries)


def verify_polyline_clearance(
    waypoints: Sequence[Sequence[Real]],
    obstacles: Iterable[Any],
    robot_radius: Real,
    margin: Real,
) -> tuple[bool, str]:
    """Check that every polyline segment has the requested obstacle clearance."""
    if (
        not isinstance(robot_radius, (int, float))
        or not math.isfinite(robot_radius)
        or robot_radius <= 0
    ):
        raise ValueError("robot_radius must be a positive finite number")
    if (
        not isinstance(margin, (int, float))
        or not math.isfinite(margin)
        or margin < 0
    ):
        raise ValueError("margin must be a finite non-negative number")

    if len(waypoints) < 2:
        return False, "PATH_TOO_SHORT: Must have at least 2 waypoints."

    points = [_validate_point(point, f"waypoints[{index}]") for index, point in enumerate(waypoints)]
    geometries = list(obstacles)
    required_clearance = robot_radius + margin
    minimum_distance = math.inf
    for start, end in zip(points, points[1:]):
        minimum_distance = min(
            minimum_distance, segment_min_distance(start, end, geometries)
        )

    if not minimum_distance < required_clearance - FLOAT_ABS_TOL:
        return True, "CLEAR"
    return (
        False,
        f"INSUFFICIENT_CLEARANCE: {minimum_distance:.12g}m < "
        f"{required_clearance:.12g}m",
    )
