"""Targeted KC-04 speed-profile mutants."""

from __future__ import annotations

from .reference import _profile


def plan_m1_curvature_blind(problem: dict):
    return _profile(problem, use_curvature=False)


def plan_m2_infinite_braking(problem: dict):
    profile = _profile(problem, apply_braking=False)
    if profile is None:
        return None
    result = list(profile)
    x, y, _velocity, timestamp = result[-1]
    result[-1] = (x, y, problem["v_goal"], timestamp)
    return result


def plan_m3_endpoint_speed_error(problem: dict):
    profile = _profile(problem)
    if profile is None:
        return None
    result = list(profile)
    x, y, velocity, timestamp = result[-1]
    result[-1] = (x, y, velocity + 1.0, timestamp)
    return result


def plan_m4_speed_limit_exceeded(problem: dict):
    profile = _profile(problem)
    if profile is None:
        return None
    result = list(profile)
    for index in range(1, len(result) - 1):
        x, y, _velocity, timestamp = result[index]
        result[index] = (x, y, 1.5 * problem["v_max"], timestamp)
    return result


def plan_m5_false_infeasible(problem: dict):
    if problem.get("feasible", True):
        return None
    return _profile(problem)


MUTANTS = {
    "M1_CURVATURE_BLIND": plan_m1_curvature_blind,
    "M2_INFINITE_BRAKING": plan_m2_infinite_braking,
    "M3_ENDPOINT_SPEED_ERROR": plan_m3_endpoint_speed_error,
    "M4_SPEED_LIMIT_EXCEEDED": plan_m4_speed_limit_exceeded,
    "M5_FALSE_INFEASIBLE": plan_m5_false_infeasible,
}
