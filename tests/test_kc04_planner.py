import pytest

from kinocheck.grader import evaluate_scenario
from tasks.kc04 import mutants, reference


def scenario(**overrides):
    current = {
        "task_id": "KC-04",
        "path": [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        "v_max": 4.0,
        "a_accel_max": 2.0,
        "a_brake_max": 2.0,
        "a_lat_max": 4.0,
        "v_start": 1.0,
        "v_goal": 1.0,
        "feasible": True,
        "optimal_time": 1.5,
    }
    current.update(overrides)
    return current


def test_reference_profiles_straight_track():
    current = scenario(
        path=[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        v_start=1.0,
        v_goal=1.0,
        optimal_time=1.2360679774997896,
    )
    result = reference.plan(current)
    verdict = evaluate_scenario(result, current)

    assert verdict.status == "PASS"
    assert result[0][2] == pytest.approx(1.0)
    assert result[-1][2] == pytest.approx(1.0)
    assert verdict.metrics["time_regret"] == pytest.approx(0.0)


def test_reference_throttles_for_sharp_bend():
    current = scenario(
        path=[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
        v_max=4.0,
        a_lat_max=1.0,
        v_start=0.5,
        v_goal=0.5,
        optimal_time=2.0,
    )
    result = reference.plan(current)
    verdict = evaluate_scenario(result, current)

    assert verdict.status == "PASS"
    assert result[1][2] < 1.0
    assert verdict.metrics["max_lateral_accel"] <= current["a_lat_max"]


def test_reference_returns_none_when_braking_distance_is_insufficient():
    current = scenario(
        path=[[0.0, 0.0], [0.1, 0.0]],
        v_max=4.0,
        a_accel_max=2.0,
        a_brake_max=0.1,
        v_start=4.0,
        v_goal=0.0,
        optimal_time=1.0,
    )

    assert reference.plan(current) is None


def test_reference_returns_none_for_infeasible_scenario():
    assert reference.plan(scenario(feasible=False)) is None


@pytest.mark.parametrize(
    ("name", "category"),
    [
        ("M1_CURVATURE_BLIND", "LATERAL_ACCEL_EXCEEDED"),
        ("M2_INFINITE_BRAKING", "LONGITUDINAL_ACCEL_EXCEEDED"),
        ("M3_ENDPOINT_SPEED_ERROR", "ENDPOINT_SPEED_ERROR"),
        ("M4_SPEED_LIMIT_EXCEEDED", "SPEED_LIMIT_EXCEEDED"),
        ("M5_FALSE_INFEASIBLE", "FALSE_INFEASIBLE"),
    ],
)
def test_kc04_mutants_target_expected_categories(name, category):
    if name == "M1_CURVATURE_BLIND":
        current = scenario(
            path=[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            v_max=4.0,
            a_lat_max=1.0,
            v_start=1.0,
            v_goal=1.0,
            optimal_time=2.0,
        )
    elif name == "M2_INFINITE_BRAKING":
        current = scenario(
            path=[[0.0, 0.0], [1.0, 0.0]],
            v_max=4.0,
            a_accel_max=2.0,
            a_brake_max=1.0,
            v_start=4.0,
            v_goal=0.0,
            optimal_time=1.0,
        )
    elif name == "M3_ENDPOINT_SPEED_ERROR":
        current = scenario()
    elif name == "M4_SPEED_LIMIT_EXCEEDED":
        current = scenario()
    else:
        current = scenario()

    result = mutants.MUTANTS[name](current)
    verdict = evaluate_scenario(result, current)

    assert verdict.status == "FAIL"
    assert verdict.failure_category == category
