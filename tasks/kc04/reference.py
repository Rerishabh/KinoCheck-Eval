"""Reference curvature-aware speed profiler for KC-04."""

from __future__ import annotations

import math

from kinocheck.geometry import compute_menger_curvature
from kinocheck.tolerances import FLOAT_ABS_TOL, TIME_ABS_TOL, VEL_ABS_TOL


def _profile(
    problem: dict,
    *,
    use_curvature: bool = True,
    apply_braking: bool = True,
) -> list[tuple[float, float, float, float]] | None:
    if not problem.get("feasible", True):
        return None

    points = [
        (float(point[0]), float(point[1])) for point in problem.get("path", [])
    ]
    if len(points) < 2:
        return None

    curvature = compute_menger_curvature(points)
    caps = [
        min(
            problem["v_max"],
            math.sqrt(
                problem["a_lat_max"] / max(kappa, 1e-6)
            )
            if use_curvature
            else problem["v_max"],
        )
        for kappa in curvature
    ]
    if (
        problem["v_start"] > caps[0] + VEL_ABS_TOL
        or problem["v_goal"] > caps[-1] + VEL_ABS_TOL
    ):
        return None

    speeds = list(caps)
    speeds[0] = min(speeds[0], problem["v_start"])
    if apply_braking:
        speeds[-1] = min(speeds[-1], problem["v_goal"])

    for index in range(len(points) - 1):
        ds = math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        speeds[index + 1] = min(
            speeds[index + 1],
            math.sqrt(speeds[index] ** 2 + 2 * problem["a_accel_max"] * ds),
        )

    if apply_braking:
        for index in range(len(points) - 2, -1, -1):
            ds = math.hypot(
                points[index + 1][0] - points[index][0],
                points[index + 1][1] - points[index][1],
            )
            speeds[index] = min(
                speeds[index],
                math.sqrt(
                    speeds[index + 1] ** 2
                    + 2 * problem["a_brake_max"] * ds
                ),
            )

    if abs(speeds[0] - problem["v_start"]) > VEL_ABS_TOL:
        return None
    if apply_braking and abs(speeds[-1] - problem["v_goal"]) > VEL_ABS_TOL:
        return None

    timestamps = [0.0] * len(points)
    for index in range(len(points) - 1):
        ds = math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        if ds <= FLOAT_ABS_TOL:
            dt = TIME_ABS_TOL
        elif speeds[index] + speeds[index + 1] > VEL_ABS_TOL:
            dt = 2 * ds / (speeds[index] + speeds[index + 1])
        else:
            dt = math.sqrt(
                2 * ds / max(problem["a_accel_max"], 0.1)
            )
        timestamps[index + 1] = timestamps[index] + max(dt, TIME_ABS_TOL)

    return [
        (point[0], point[1], speeds[index], timestamps[index])
        for index, point in enumerate(points)
    ]


def plan(problem: dict) -> list[tuple[float, float, float, float]] | None:
    """Return a curvature- and acceleration-compliant speed profile."""
    return _profile(problem)
