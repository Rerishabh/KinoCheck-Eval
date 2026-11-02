"""Reusable KC-02 and KC-03 scenario result validation."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Real
from typing import Literal

from .geometry import (
    compute_menger_curvature,
    compute_polyline_risk,
    create_obstacle_boxes,
    verify_polyline_clearance,
)
from .tolerances import (
    ACCEL_ABS_TOL,
    CLEARANCE_MARGIN_METERS,
    FLOAT_ABS_TOL,
    RISK_ABS_TOL,
    TIME_ABS_TOL,
    VEL_ABS_TOL,
)


@dataclass(frozen=True)
class ScenarioVerdict:
    status: Literal["PASS", "FAIL"]
    failure_category: str | None
    reason: str | None
    result: object
    metrics: dict[str, float] | None = None
    warnings: list[str] | None = None


def _failure(
    category: str, result: object, reason: str | None = None
) -> ScenarioVerdict:
    return ScenarioVerdict("FAIL", category, reason, result)


def _format_points(
    result: object,
) -> tuple[list[tuple[float, float]] | None, str | None]:
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


def _evaluate_kc02_scenario(result: object, scenario: dict) -> ScenarioVerdict:
    """Validate one candidate result against the KC-02 scenario contract."""
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


def _validate_finite_real(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} cannot be a boolean")
    if not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _validate_kc03_scenario(scenario: dict) -> None:
    required = {
        "grid",
        "risk_grid",
        "cell_size",
        "risk_budget",
        "optimal_length",
        "feasible",
        "start",
        "goal",
        "radius",
    }
    missing = required.difference(scenario)
    if missing:
        raise ValueError(f"KC-03 scenario missing keys: {sorted(missing)}")

    grid = scenario["grid"]
    risk_grid = scenario["risk_grid"]
    for name, value in (("grid", grid), ("risk_grid", risk_grid)):
        if (
            isinstance(value, (str, bytes))
            or not isinstance(value, Sequence)
            or not value
            or any(
                isinstance(column, (str, bytes))
                or not isinstance(column, Sequence)
                or not column
                for column in value
            )
        ):
            raise ValueError(f"{name} must be non-empty and rectangular")

    grid_height = len(grid[0])
    risk_height = len(risk_grid[0])
    if (
        len(grid) != len(risk_grid)
        or any(len(column) != grid_height for column in grid)
        or any(len(column) != risk_height for column in risk_grid)
        or grid_height != risk_height
    ):
        raise ValueError(
            "grid and risk_grid must have identical rectangular dimensions"
        )

    for column in grid:
        if any(type(value) is not int or value not in (0, 1) for value in column):
            raise ValueError("grid values must be 0 or 1")
    for column in risk_grid:
        for value in column:
            risk = _validate_finite_real(value, "risk_grid value")
            if risk < 0:
                raise ValueError("risk_grid values must be non-negative")

    cell_size = _validate_finite_real(scenario["cell_size"], "cell_size")
    if cell_size <= 0:
        raise ValueError("cell_size must be positive")
    risk_budget = _validate_finite_real(scenario["risk_budget"], "risk_budget")
    if risk_budget < 0:
        raise ValueError("risk_budget must be non-negative")
    optimal_length = _validate_finite_real(
        scenario["optimal_length"], "optimal_length"
    )
    if optimal_length < 0:
        raise ValueError("optimal_length must be non-negative")
    if not isinstance(scenario["feasible"], bool):
        raise ValueError("feasible must be a boolean")
    if scenario["feasible"] and optimal_length <= 0.0:
        raise ValueError("optimal_length must be positive for feasible scenarios")

    for name in ("start", "goal"):
        point = scenario[name]
        if (
            isinstance(point, (str, bytes))
            or not isinstance(point, Sequence)
            or len(point) != 2
        ):
            raise ValueError(f"{name} must contain exactly two coordinates")
        for coordinate in point:
            _validate_finite_real(coordinate, f"{name} coordinate")
        width = len(grid) * cell_size
        height = len(grid[0]) * cell_size
        if not (0.0 <= point[0] <= width and 0.0 <= point[1] <= height):
            raise ValueError(f"{name} must lie within the arena bounds")

    radius = _validate_finite_real(scenario["radius"], "radius")
    if radius <= 0:
        raise ValueError("radius must be positive")


def evaluate_kc03_scenario(result: object, scenario: dict) -> ScenarioVerdict:
    """Validate one candidate result against the KC-03 scenario contract."""
    _validate_kc03_scenario(scenario)

    if result is not None:
        points, format_reason = _format_points(result)
        if points is None or not all(
            math.isfinite(coordinate)
            for point in points
            for coordinate in point
        ):
            return _failure(
                "INVALID_OUTPUT",
                result,
                format_reason or "Result coordinates must be finite.",
            )
    else:
        points = None

    if result is None:
        if scenario["feasible"]:
            return _failure("FALSE_INFEASIBLE", result)
        return ScenarioVerdict("PASS", None, None, result)
    if not scenario["feasible"]:
        return _failure("FALSE_FEASIBLE", result)

    start = scenario["start"]
    goal = scenario["goal"]
    if (
        abs(points[0][0] - start[0]) > FLOAT_ABS_TOL
        or abs(points[0][1] - start[1]) > FLOAT_ABS_TOL
        or abs(points[-1][0] - goal[0]) > FLOAT_ABS_TOL
        or abs(points[-1][1] - goal[1]) > FLOAT_ABS_TOL
    ):
        return _failure("INVALID_OUTPUT", result, "Endpoints do not match start/goal")

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
        return _failure("COLLISION_SWEPT", result, reason)

    accumulated_risk = compute_polyline_risk(
        points, scenario["risk_grid"], scenario["cell_size"]
    )
    if accumulated_risk > scenario["risk_budget"] + RISK_ABS_TOL:
        return _failure(
            "BUDGET_EXCEEDED",
            result,
            f"Accumulated risk {accumulated_risk:.12g} exceeds "
            f"budget {scenario['risk_budget']:.12g}.",
        )

    path_length = sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(points, points[1:])
    )
    accumulated_risk = float(accumulated_risk)
    budget_slack = scenario["risk_budget"] - accumulated_risk
    length_regret = (
        (path_length - scenario["optimal_length"])
        / max(scenario["optimal_length"], FLOAT_ABS_TOL)
    )
    warnings: list[str] = []
    if length_regret < -FLOAT_ABS_TOL:
        warnings.append("REGRET_BELOW_ZERO_WARNING")
    elif length_regret < 0.0:
        length_regret = 0.0

    return ScenarioVerdict(
        "PASS",
        None,
        None,
        points,
        metrics={
            "path_length": path_length,
            "accumulated_risk": accumulated_risk,
            "budget_slack": budget_slack,
            "length_regret": length_regret,
        },
        warnings=warnings,
    )


def _validate_kc04_scenario(scenario: dict) -> None:
    if not isinstance(scenario, dict):
        raise ValueError("Scenario must be a dictionary")
    required = {
        "task_id",
        "path",
        "v_max",
        "a_accel_max",
        "a_brake_max",
        "a_lat_max",
        "v_start",
        "v_goal",
        "feasible",
        "optimal_time",
    }
    missing = required.difference(scenario)
    if missing:
        raise ValueError(f"KC-04 scenario missing keys: {sorted(missing)}")
    if scenario["task_id"] != "KC-04":
        raise ValueError("task_id must be KC-04")

    path = scenario["path"]
    if (
        isinstance(path, (str, bytes))
        or not isinstance(path, Sequence)
        or len(path) < 2
    ):
        raise ValueError("path must contain at least two points")
    for index, point in enumerate(path):
        if (
            isinstance(point, (str, bytes))
            or not isinstance(point, Sequence)
            or len(point) != 2
        ):
            raise ValueError(f"path[{index}] must contain exactly two coordinates")
        for coordinate in point:
            _validate_finite_real(coordinate, f"path[{index}] coordinate")

    for name in ("v_max", "a_accel_max", "a_brake_max", "a_lat_max"):
        value = _validate_finite_real(scenario[name], name)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
    for name in ("v_start", "v_goal"):
        value = _validate_finite_real(scenario[name], name)
        if value < 0:
            raise ValueError(f"{name} must be non-negative")
    if not isinstance(scenario["feasible"], bool):
        raise ValueError("feasible must be a boolean")
    optimal_time = _validate_finite_real(
        scenario["optimal_time"], "optimal_time"
    )
    if optimal_time < 0 or (scenario["feasible"] and optimal_time <= 0):
        requirement = (
            "positive for feasible scenarios"
            if scenario["feasible"]
            else "non-negative"
        )
        raise ValueError(f"optimal_time must be {requirement}")


def _format_trajectory(
    result: object, path: Sequence
) -> tuple[list[tuple[float, float, float, float]] | None, str | None]:
    if isinstance(result, (str, bytes)) or not isinstance(result, Sequence):
        return None, "Result must be a sequence of trajectory tuples."
    if len(result) != len(path):
        return None, "Trajectory length must match path length."

    trajectory: list[tuple[float, float, float, float]] = []
    for index, item in enumerate(result):
        if (
            isinstance(item, (str, bytes))
            or not isinstance(item, Sequence)
            or len(item) != 4
        ):
            return None, "Each trajectory item must contain four values."
        if any(isinstance(value, bool) or not isinstance(value, Real) for value in item):
            return None, "Trajectory values must be numeric."
        values = tuple(float(value) for value in item)
        if not all(math.isfinite(value) for value in values):
            return None, "Trajectory values must be finite."
        x, y, velocity, timestamp = values
        if (
            abs(x - path[index][0]) > FLOAT_ABS_TOL
            or abs(y - path[index][1]) > FLOAT_ABS_TOL
        ):
            return None, "Trajectory coordinates do not match path."
        if velocity < -VEL_ABS_TOL:
            return None, "Trajectory velocities must be non-negative."
        if velocity < 0.0:
            velocity = 0.0
        if index == 0:
            if abs(timestamp) > TIME_ABS_TOL:
                return None, "Trajectory must start at time zero."
        elif timestamp - trajectory[-1][3] < TIME_ABS_TOL:
            return None, "Trajectory timestamps must be strictly increasing."
        trajectory.append((x, y, velocity, timestamp))
    return trajectory, None


def _evaluate_kc04_scenario(result: object, scenario: dict) -> ScenarioVerdict:
    _validate_kc04_scenario(scenario)
    path = scenario["path"]
    if result is not None:
        trajectory, format_reason = _format_trajectory(result, path)
        if trajectory is None:
            return _failure("INVALID_OUTPUT", result, format_reason)
    else:
        trajectory = None

    if trajectory is None:
        if scenario["feasible"]:
            return _failure("FALSE_INFEASIBLE", result)
        return ScenarioVerdict("PASS", None, None, result)
    if not scenario["feasible"]:
        return _failure("FALSE_FEASIBLE", result)

    if (
        abs(trajectory[0][2] - scenario["v_start"]) > VEL_ABS_TOL
        or abs(trajectory[-1][2] - scenario["v_goal"]) > VEL_ABS_TOL
    ):
        return _failure("ENDPOINT_SPEED_ERROR", result)

    max_velocity = max(item[2] for item in trajectory)
    if max_velocity > scenario["v_max"] + VEL_ABS_TOL:
        return _failure("SPEED_LIMIT_EXCEEDED", result)

    points = [item[:2] for item in trajectory]
    curvatures = compute_menger_curvature(points)
    lateral_accelerations = [
        velocity * velocity * curvature
        for (_, _, velocity, _), curvature in zip(trajectory, curvatures)
    ]
    max_lateral_accel = max(lateral_accelerations)
    if max_lateral_accel > scenario["a_lat_max"] + ACCEL_ABS_TOL:
        return _failure("LATERAL_ACCEL_EXCEEDED", result)

    longitudinal_accelerations: list[float] = []
    for first, second in zip(trajectory, trajectory[1:]):
        ds = math.hypot(second[0] - first[0], second[1] - first[1])
        dt = second[3] - first[3]
        if ds > FLOAT_ABS_TOL:
            acceleration = (second[2] ** 2 - first[2] ** 2) / (2 * ds)
        else:
            acceleration = (second[2] - first[2]) / dt
        longitudinal_accelerations.append(acceleration)
        if (
            acceleration < -scenario["a_brake_max"] - ACCEL_ABS_TOL
            or acceleration > scenario["a_accel_max"] + ACCEL_ABS_TOL
        ):
            return _failure("LONGITUDINAL_ACCEL_EXCEEDED", result)

    max_longitudinal_accel = max(
        (abs(acceleration) for acceleration in longitudinal_accelerations),
        default=0.0,
    )
    total_time = trajectory[-1][3]
    time_regret = (
        (total_time - scenario["optimal_time"])
        / max(scenario["optimal_time"], FLOAT_ABS_TOL)
    )
    warnings: list[str] = []
    if time_regret < -FLOAT_ABS_TOL:
        warnings.append("REGRET_BELOW_ZERO_WARNING")
    elif time_regret < 0.0:
        time_regret = 0.0
    return ScenarioVerdict(
        "PASS",
        None,
        None,
        trajectory,
        metrics={
            "total_time": total_time,
            "max_velocity": max_velocity,
            "max_lateral_accel": max_lateral_accel,
            "max_longitudinal_accel": max_longitudinal_accel,
            "time_regret": time_regret,
        },
        warnings=warnings,
    )


def evaluate_scenario(result: object, scenario: dict) -> ScenarioVerdict:
    """Validate one candidate result against its scenario contract."""
    if not isinstance(scenario, dict):
        raise ValueError("Scenario must be a dictionary")
    if scenario.get("task_id") == "KC-04":
        return _evaluate_kc04_scenario(result, scenario)
    if "risk_grid" in scenario or "risk_budget" in scenario:
        return evaluate_kc03_scenario(result, scenario)
    return _evaluate_kc02_scenario(result, scenario)
