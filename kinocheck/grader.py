"""Reusable KC-02 scenario result validation."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Real
from typing import Literal

from .geometry import create_obstacle_boxes, verify_polyline_clearance
from .tolerances import CLEARANCE_MARGIN_METERS, FLOAT_ABS_TOL


@dataclass(frozen=True)
class ScenarioVerdict:
    status: Literal["PASS", "FAIL"]
    failure_category: str | None
    reason: str | None
    result: object


def _failure(category: str, result: object, reason: str | None = None) -> ScenarioVerdict:
    return ScenarioVerdict("FAIL", category, reason, result)


def _format_points(result: object) -> tuple[list[tuple[float, float]] | None, str | None]:
    if isinstance(result, (str, bytes)) or not isinstance(result, Sequence):
        return None, "Result must be a sequence of coordinates."
    if len(result) < 2:
        return None, "Result must contain at least two coordinates."

    points: list[tuple[float, float]] = []
    for point in result:
        if (
            isinstance(point, (str, bytes))
            or not isinstance(point, Sequence)
            or len(point) != 2
            or not all(isinstance(value, Real) for value in point)
        ):
            return None, "Each result coordinate must contain two numeric values."
        points.append((float(point[0]), float(point[1])))
    return points, None


def evaluate_scenario(result: object, scenario: dict) -> ScenarioVerdict:
    """Validate one candidate result against one scenario contract."""
    if not scenario["feasible"]:
        if result is None:
            return ScenarioVerdict("PASS", None, None, result)
        return _failure("FALSE_FEASIBLE", result)

    if result is None:
        return _failure("FALSE_INFEASIBLE", result)

    points, format_reason = _format_points(result)
    if points is None:
        return _failure("INVALID_OUTPUT", result, format_reason)

    start = scenario["start"]
    goal = scenario["goal"]
    if (
        abs(points[0][0] - start[0]) > 1e-9
        or abs(points[0][1] - start[1]) > 1e-9
        or abs(points[-1][0] - goal[0]) > 1e-9
        or abs(points[-1][1] - goal[1]) > 1e-9
    ):
        return _failure("ENDPOINT_ERROR", result)

    required_clearance = (
        scenario["radius"] + CLEARANCE_MARGIN_METERS - FLOAT_ABS_TOL
    )
    width = len(scenario["grid"]) * scenario["cell_size"]
    height = len(scenario["grid"][0]) * scenario["cell_size"]
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        if (
            min(x1, x2) < required_clearance
            or max(x1, x2) > width - required_clearance
            or min(y1, y2) < required_clearance
            or max(y1, y2) > height - required_clearance
        ):
            return _failure("OUT_OF_BOUNDS", result)

    obstacles = create_obstacle_boxes(scenario["grid"], scenario["cell_size"])
    clear, reason = verify_polyline_clearance(
        points,
        obstacles,
        scenario["radius"],
        CLEARANCE_MARGIN_METERS,
    )
    if not clear:
        category = (
            "COLLISION_CORNER"
            if scenario.get("id") == "s1_diagonal_touch_pinch"
            else "COLLISION_SWEPT"
        )
        return _failure(category, result, reason)

    return ScenarioVerdict("PASS", None, None, result)
