"""Geometry primitives for grid obstacles and polyline clearance checks."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from numbers import Real
from typing import Any

from shapely.geometry import LineString, Point, Polygon

from .tolerances import CURVATURE_ABS_TOL, FLOAT_ABS_TOL


def _validate_finite_number(value: Any, name: str) -> float:
    if not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _validate_point(point: Sequence[Real], name: str) -> tuple[float, float]:
    if (
        isinstance(point, (str, bytes))
        or not isinstance(point, Sequence)
        or len(point) != 2
    ):
        raise ValueError(f"{name} must contain exactly two coordinates")
    x = _validate_finite_number(point[0], f"{name}[0]")
    y = _validate_finite_number(point[1], f"{name}[1]")
    return x, y


def compute_menger_curvature(points: object) -> list[float]:
    """Return Menger curvature values for a sequence of planar points."""
    if isinstance(points, (str, bytes)) or not isinstance(points, Sequence):
        raise ValueError("points must be a sequence of two-dimensional points")

    validated = [
        _validate_point(point, f"points[{index}]")
        for index, point in enumerate(points)
    ]
    count = len(validated)
    if count < 3:
        return [0.0] * count

    curvature = [0.0] * count
    for index in range(1, count - 1):
        previous = validated[index - 1]
        current = validated[index]
        following = validated[index + 1]
        d1 = math.hypot(current[0] - previous[0], current[1] - previous[1])
        d2 = math.hypot(following[0] - current[0], following[1] - current[1])
        d3 = math.hypot(following[0] - previous[0], following[1] - previous[1])
        denominator = d1 * d2 * d3
        if denominator <= FLOAT_ABS_TOL:
            value = 0.0
        else:
            semiperimeter = 0.5 * (d1 + d2 + d3)
            radicand = (
                semiperimeter
                * (semiperimeter - d1)
                * (semiperimeter - d2)
                * (semiperimeter - d3)
            )
            area = math.sqrt(max(0.0, radicand))
            value = (4.0 * area) / denominator
        curvature[index] = 0.0 if value <= CURVATURE_ABS_TOL else value

    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]
    return curvature


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


def compute_polyline_risk(
    path: Sequence[Sequence[Real]],
    risk_grid: Sequence[Sequence[Real]],
    cell_size: Real,
) -> float:
    """Return the piecewise-constant risk line integral along a polyline."""
    size = _validate_finite_number(cell_size, "cell_size")
    if size <= 0:
        raise ValueError("cell_size must be positive")
    if not risk_grid or not all(risk_grid):
        raise ValueError("risk_grid must be non-empty")

    row_length = len(risk_grid[0])
    if row_length == 0 or any(len(column) != row_length for column in risk_grid):
        raise ValueError("risk_grid must be rectangular and non-empty")

    grid: list[list[float]] = []
    for column in risk_grid:
        values: list[float] = []
        for value in column:
            risk = _validate_finite_number(value, "risk_grid value")
            if risk < 0:
                raise ValueError("risk_grid values must be non-negative")
            values.append(risk)
        grid.append(values)

    points = [
        _validate_point(point, f"path[{index}]")
        for index, point in enumerate(path)
    ]
    max_x = len(grid) * size
    max_y = row_length * size
    for index, (x, y) in enumerate(points):
        if (
            x < 0.0
            or x > max_x
            or y < 0.0
            or y > max_y
        ):
            raise ValueError(f"path[{index}] is outside the arena bounds")
        points[index] = (
            min(max(x, 0.0), max_x),
            min(max(y, 0.0), max_y),
        )

    total_risk = 0.0
    width = len(grid)
    height = row_length
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        dx = x1 - x0
        dy = y1 - y0
        segment_length = math.hypot(dx, dy)
        if segment_length == 0.0:
            continue

        parameters = [0.0, 1.0]
        if dx != 0.0:
            first = max(1, math.ceil(min(x0, x1) / size))
            last = min(width - 1, math.floor(max(x0, x1) / size))
            for index in range(first, last + 1):
                parameter = (index * size - x0) / dx
                if 0.0 < parameter < 1.0:
                    parameters.append(parameter)
        if dy != 0.0:
            first = max(1, math.ceil(min(y0, y1) / size))
            last = min(height - 1, math.floor(max(y0, y1) / size))
            for index in range(first, last + 1):
                parameter = (index * size - y0) / dy
                if 0.0 < parameter < 1.0:
                    parameters.append(parameter)

        parameters = sorted(parameters)
        unique_parameters: list[float] = []
        for parameter in parameters:
            if (
                not unique_parameters
                or parameter - unique_parameters[-1] > FLOAT_ABS_TOL
            ):
                unique_parameters.append(parameter)

        for start, end in zip(unique_parameters, unique_parameters[1:]):
            interval = end - start
            if interval <= 0.0:
                continue
            midpoint_x = x0 + (start + end) * dx / 2.0
            midpoint_y = y0 + (start + end) * dy / 2.0
            vertical_line = next(
                (
                    index
                    for index in range(1, width)
                    if abs(dx) <= FLOAT_ABS_TOL
                    and abs(x0 - index * size) <= FLOAT_ABS_TOL
                ),
                None,
            )
            horizontal_line = next(
                (
                    index
                    for index in range(1, height)
                    if abs(dy) <= FLOAT_ABS_TOL
                    and abs(y0 - index * size) <= FLOAT_ABS_TOL
                ),
                None,
            )

            if vertical_line is not None:
                y_cell = min(height - 1, max(0, math.floor(midpoint_y / size)))
                risk = (grid[vertical_line - 1][y_cell] + grid[vertical_line][y_cell]) / 2.0
            elif horizontal_line is not None:
                x_cell = min(width - 1, max(0, math.floor(midpoint_x / size)))
                risk = (grid[x_cell][horizontal_line - 1] + grid[x_cell][horizontal_line]) / 2.0
            else:
                x_cell = min(width - 1, max(0, math.floor(midpoint_x / size)))
                y_cell = min(height - 1, max(0, math.floor(midpoint_y / size)))
                risk = grid[x_cell][y_cell]

            total_risk += interval * segment_length * risk

    return total_risk
