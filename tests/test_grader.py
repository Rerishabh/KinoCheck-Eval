import pytest

from kinocheck.grader import evaluate_scenario
from kinocheck.tolerances import FLOAT_ABS_TOL, VEL_ABS_TOL


def scenario(
    *,
    scenario_id="grader",
    grid=None,
    start=(0.5, 0.5),
    goal=(2.5, 0.5),
    radius=0.3,
):
    return {
        "id": scenario_id,
        "grid": grid or [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
        "cell_size": 1.0,
        "radius": radius,
        "start": list(start),
        "goal": list(goal),
        "feasible": True,
    }


def test_clean_path_passes():
    verdict = evaluate_scenario([(0.5, 0.5), (2.5, 0.5)], scenario())

    assert verdict.status == "PASS"
    assert verdict.failure_category is None


def test_boundary_violation_is_classified():
    current = scenario(start=(0.5, 0.25), goal=(2.5, 0.25))

    verdict = evaluate_scenario(
        [(0.5, 0.25), (2.5, 0.25)],
        current,
    )

    assert verdict.status == "FAIL"
    assert verdict.failure_category == "OUT_OF_BOUNDS"


def test_corner_collision_is_classified():
    current = scenario(
        scenario_id="s1_diagonal_touch_pinch",
        grid=[[0, 1], [1, 0]],
        start=(0.5, 0.5),
        goal=(1.5, 1.5),
    )

    verdict = evaluate_scenario([(0.5, 0.5), (1.5, 1.5)], current)

    assert verdict.failure_category == "COLLISION_CORNER"


def test_swept_collision_is_classified():
    current = scenario(
        grid=[[0, 0, 0], [0, 1, 0], [0, 0, 0]],
        start=(0.5, 1.5),
        goal=(2.5, 1.5),
    )

    verdict = evaluate_scenario([(0.5, 1.5), (2.5, 1.5)], current)

    assert verdict.failure_category == "COLLISION_SWEPT"


def test_format_error_is_classified():
    verdict = evaluate_scenario([], scenario())

    assert verdict.failure_category == "INVALID_OUTPUT"


def test_endpoint_mismatch_is_classified():
    verdict = evaluate_scenario([(0.5, 0.5), (2.4, 0.5)], scenario())

    assert verdict.failure_category == "ENDPOINT_ERROR"


def kc03_scenario(**overrides):
    current = {
        "id": "kc03-grader",
        "grid": [[0], [0], [0]],
        "risk_grid": [[1.0], [1.0], [1.0]],
        "cell_size": 1.0,
        "radius": 0.1,
        "start": [0.5, 0.5],
        "goal": [2.5, 0.5],
        "feasible": True,
        "risk_budget": 10.0,
        "optimal_length": 2.0,
    }
    current.update(overrides)
    return current


@pytest.mark.parametrize(
    "changes",
    [
        {"risk_grid": [[1.0], [1.0]]},
        {"risk_grid": [[1.0, 1.0], [1.0], [1.0]]},
        {"cell_size": 0.0},
        {"risk_budget": -1.0},
        {"optimal_length": float("inf")},
        {"feasible": 1},
        {"risk_grid": [[-1.0], [1.0], [1.0]]},
    ],
)
def test_kc03_malformed_scenario_raises_value_error(changes):
    current = kc03_scenario()
    current.update(changes)

    with pytest.raises(ValueError):
        evaluate_scenario(None, current)


def test_kc03_missing_required_schema_key_raises_value_error():
    current = kc03_scenario()
    del current["optimal_length"]

    with pytest.raises(ValueError):
        evaluate_scenario(None, current)


def test_kc03_structural_format_error_is_invalid_output():
    verdict = evaluate_scenario([("bad", 0.5)], kc03_scenario())

    assert verdict.failure_category == "INVALID_OUTPUT"


def test_kc03_none_on_feasible_is_false_infeasible():
    verdict = evaluate_scenario(None, kc03_scenario())

    assert verdict.failure_category == "FALSE_INFEASIBLE"


def test_kc03_path_on_infeasible_is_false_feasible():
    verdict = evaluate_scenario(
        [(0.5, 0.5), (2.5, 0.5)],
        kc03_scenario(feasible=False),
    )

    assert verdict.failure_category == "FALSE_FEASIBLE"


def test_kc03_none_on_infeasible_passes():
    verdict = evaluate_scenario(None, kc03_scenario(feasible=False))

    assert verdict.status == "PASS"
    assert verdict.result is None


def test_kc03_endpoint_mismatch_is_invalid_output():
    verdict = evaluate_scenario(
        [(0.5, 0.5), (2.4, 0.5)],
        kc03_scenario(),
    )

    assert verdict.failure_category == "INVALID_OUTPUT"
    assert verdict.reason == "Endpoints do not match start/goal"


def test_kc03_boundary_violation_is_out_of_bounds():
    current = kc03_scenario(start=[0.5, 0.1], goal=[2.5, 0.1])

    verdict = evaluate_scenario(
        [(0.5, 0.1), (2.5, 0.1)],
        current,
    )

    assert verdict.failure_category == "OUT_OF_BOUNDS"


def test_kc03_obstacle_violation_is_collision_swept():
    current = kc03_scenario(
        grid=[[0], [1], [0]],
        risk_grid=[[1.0], [1.0], [1.0]],
    )

    verdict = evaluate_scenario(
        [(0.5, 0.5), (2.5, 0.5)],
        current,
    )

    assert verdict.failure_category == "COLLISION_SWEPT"


def test_kc03_risk_budget_exceeded():
    current = kc03_scenario(
        risk_grid=[[10.0], [10.0], [10.0]],
        risk_budget=1.0,
    )

    verdict = evaluate_scenario(
        [(0.5, 0.5), (2.5, 0.5)],
        current,
    )

    assert verdict.failure_category == "BUDGET_EXCEEDED"


def test_kc03_pass_reports_metrics():
    verdict = evaluate_scenario(
        [(0.5, 0.5), (2.5, 0.5)],
        kc03_scenario(),
    )

    assert verdict.status == "PASS"
    assert verdict.result == [(0.5, 0.5), (2.5, 0.5)]
    assert verdict.metrics == {
        "path_length": pytest.approx(2.0),
        "accumulated_risk": pytest.approx(2.0),
        "budget_slack": pytest.approx(8.0),
        "length_regret": pytest.approx(0.0),
    }
    assert verdict.warnings == []


def test_kc03_negative_regret_adds_warning():
    verdict = evaluate_scenario(
        [(0.5, 0.5), (2.5, 0.5)],
        kc03_scenario(optimal_length=3.0),
    )

    assert verdict.status == "PASS"
    assert verdict.metrics["length_regret"] == pytest.approx(-1 / 3)
    assert verdict.warnings == ["REGRET_BELOW_ZERO_WARNING"]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("cell_size", True),
        ("radius", False),
        ("risk_budget", True),
        ("optimal_length", False),
        ("start", [True, 0.5]),
        ("goal", [2.5, False]),
    ],
)
def test_kc03_scenario_rejects_booleans_in_numeric_fields(field, value):
    with pytest.raises(ValueError):
        evaluate_scenario(None, kc03_scenario(**{field: value}))


@pytest.mark.parametrize(
    "grid",
    [
        [[True], [0], [0]],
        [[1.0], [0], [0]],
        [["1"], [0], [0]],
    ],
)
def test_kc03_scenario_rejects_non_int_or_bool_in_grid(grid):
    with pytest.raises(ValueError):
        evaluate_scenario(None, kc03_scenario(grid=grid))


def test_kc03_feasible_scenario_requires_positive_optimal_length():
    with pytest.raises(ValueError, match="optimal_length must be positive"):
        evaluate_scenario(None, kc03_scenario(optimal_length=0.0))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("start", [-0.1, 0.5]),
        ("goal", [3.5, 0.5]),
    ],
)
def test_kc03_scenario_rejects_start_or_goal_outside_arena(field, value):
    with pytest.raises(ValueError, match="arena bounds"):
        evaluate_scenario(None, kc03_scenario(**{field: value}))


def test_kc03_precedence_out_of_bounds_before_collision():
    current = kc03_scenario(
        start=[0.5, 0.1],
        goal=[2.5, 0.1],
        grid=[[0], [1], [0]],
        risk_grid=[[1.0], [1.0], [1.0]],
    )

    verdict = evaluate_scenario(
        [(0.5, 0.1), (1.5, 0.5), (2.5, 0.1)],
        current,
    )

    assert verdict.failure_category == "OUT_OF_BOUNDS"


def test_kc03_boundary_clearance_threshold():
    required_clearance = 0.1 + 0.01 - FLOAT_ABS_TOL
    passing = kc03_scenario(
        start=[0.5, required_clearance],
        goal=[2.5, required_clearance],
    )
    failing = kc03_scenario(
        start=[0.5, required_clearance - 2 * FLOAT_ABS_TOL],
        goal=[2.5, required_clearance - 2 * FLOAT_ABS_TOL],
    )

    assert evaluate_scenario(
        [(0.5, required_clearance), (2.5, required_clearance)],
        passing,
    ).status == "PASS"
    assert evaluate_scenario(
        [
            (0.5, required_clearance - 2 * FLOAT_ABS_TOL),
            (2.5, required_clearance - 2 * FLOAT_ABS_TOL),
        ],
        failing,
    ).failure_category == "OUT_OF_BOUNDS"


def kc04_scenario(**overrides):
    current = {
        "task_id": "KC-04",
        "path": [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        "v_max": 2.0,
        "a_accel_max": 1.0,
        "a_brake_max": 1.0,
        "a_lat_max": 1.0,
        "v_start": 1.0,
        "v_goal": 1.0,
        "feasible": True,
        "optimal_time": 2.0,
    }
    current.update(overrides)
    return current


def kc04_trajectory(**overrides):
    current = [
        (0.0, 0.0, 1.0, 0.0),
        (1.0, 0.0, 1.0, 1.0),
        (2.0, 0.0, 1.0, 2.0),
    ]
    current[0] = (*current[0][:2], overrides.get("start_velocity", current[0][2]), current[0][3])
    current[-1] = (*current[-1][:2], overrides.get("goal_velocity", current[-1][2]), current[-1][3])
    return current


@pytest.mark.parametrize(
    "changes",
    [
        {"path": [[0.0, 0.0]]},
        {"v_max": True},
        {"a_accel_max": 0.0},
        {"a_brake_max": float("inf")},
        {"a_lat_max": -1.0},
        {"v_start": False},
        {"v_goal": -1.0},
        {"feasible": 1},
        {"optimal_time": -1.0},
    ],
)
def test_kc04_malformed_scenario_raises_value_error(changes):
    with pytest.raises(ValueError):
        evaluate_scenario(None, kc04_scenario(**changes))


@pytest.mark.parametrize(
    "trajectory",
    [
        [(0.0, 0.0, 1.0, 0.0)],
        [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)],
        [(0.0, 0.0, 1.0, 0.0), (1.0, 0.0, 1.0, -0.1), (2.0, 0.0, 1.0, 2.0)],
        [(0.0, 0.0, 1.0, 0.0), (1.1, 0.0, 1.0, 1.0), (2.0, 0.0, 1.0, 2.0)],
    ],
)
def test_kc04_invalid_trajectory_format_is_invalid_output(trajectory):
    verdict = evaluate_scenario(trajectory, kc04_scenario())

    assert verdict.status == "FAIL"
    assert verdict.failure_category == "INVALID_OUTPUT"


def test_kc04_feasibility_verdicts():
    assert evaluate_scenario(None, kc04_scenario()).failure_category == "FALSE_INFEASIBLE"
    assert (
        evaluate_scenario(
            kc04_trajectory(), kc04_scenario(feasible=False, optimal_time=0.0)
        ).failure_category
        == "FALSE_FEASIBLE"
    )
    verdict = evaluate_scenario(None, kc04_scenario(feasible=False, optimal_time=0.0))
    assert verdict.status == "PASS"


@pytest.mark.parametrize(
    ("start_velocity", "goal_velocity"),
    [(0.0, 1.0), (1.0, 0.0)],
)
def test_kc04_endpoint_speed_error(start_velocity, goal_velocity):
    trajectory = kc04_trajectory(
        start_velocity=start_velocity, goal_velocity=goal_velocity
    )
    verdict = evaluate_scenario(
        trajectory,
        kc04_scenario(v_start=1.0, v_goal=1.0),
    )

    assert verdict.failure_category == "ENDPOINT_SPEED_ERROR"


def test_kc04_speed_limit_exceeded():
    trajectory = kc04_trajectory()
    trajectory[1] = (1.0, 0.0, 2.1, 1.0)
    verdict = evaluate_scenario(trajectory, kc04_scenario())

    assert verdict.failure_category == "SPEED_LIMIT_EXCEEDED"


def test_kc04_lateral_acceleration_exceeded():
    path = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]
    trajectory = [(0.0, 0.0, 2.0, 0.0), (1.0, 0.0, 2.0, 1.0), (1.0, 1.0, 2.0, 2.0)]
    verdict = evaluate_scenario(
        trajectory,
        kc04_scenario(path=path, v_start=2.0, v_goal=2.0, a_lat_max=1.0),
    )

    assert verdict.failure_category == "LATERAL_ACCEL_EXCEEDED"


@pytest.mark.parametrize(
    ("speeds", "v_start", "v_goal"),
    [((0.0, 2.0), 0.0, 2.0), ((2.0, 0.0), 2.0, 0.0)],
)
def test_kc04_longitudinal_acceleration_exceeded(speeds, v_start, v_goal):
    trajectory = [
        (0.0, 0.0, speeds[0], 0.0),
        (1.0, 0.0, speeds[1], 1.0),
        (2.0, 0.0, speeds[1], 2.0),
    ]
    verdict = evaluate_scenario(
        trajectory,
        kc04_scenario(
            v_start=v_start,
            v_goal=v_goal,
            a_accel_max=1.0,
            a_brake_max=1.0,
        ),
    )

    assert verdict.failure_category == "LONGITUDINAL_ACCEL_EXCEEDED"


def test_kc04_pass_reports_kinematic_metrics():
    verdict = evaluate_scenario(kc04_trajectory(), kc04_scenario())

    assert verdict.status == "PASS"
    assert verdict.metrics == {
        "total_time": pytest.approx(2.0),
        "max_velocity": pytest.approx(1.0),
        "max_lateral_accel": pytest.approx(0.0),
        "max_longitudinal_accel": pytest.approx(0.0),
        "time_regret": pytest.approx(0.0),
    }
    assert verdict.warnings == []


def test_kc04_negative_time_regret_adds_warning():
    verdict = evaluate_scenario(
        kc04_trajectory(),
        kc04_scenario(optimal_time=3.0),
    )

    assert verdict.status == "PASS"
    assert verdict.metrics["time_regret"] == pytest.approx(-1 / 3)
    assert verdict.warnings == ["REGRET_BELOW_ZERO_WARNING"]


def test_kc04_scenario_rejects_non_dict():
    with pytest.raises(ValueError, match="Scenario must be a dictionary"):
        evaluate_scenario(None, "not_a_dict")


def test_kc04_velocity_near_zero_is_clamped():
    trajectory = kc04_trajectory()
    trajectory[1] = (1.0, 0.0, -0.5 * VEL_ABS_TOL, 1.0)
    verdict = evaluate_scenario(trajectory, kc04_scenario())

    assert verdict.status == "PASS"
    assert verdict.result[1][2] == 0.0


def test_kc04_zero_distance_acceleration_fallback():
    path = [[0.0, 0.0], [0.0, 0.0], [1.0, 0.0]]
    trajectory = [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 2.0, 1.0),
        (1.0, 0.0, 2.0, 2.0),
    ]
    verdict = evaluate_scenario(
        trajectory,
        kc04_scenario(
            path=path,
            v_start=0.0,
            v_goal=2.0,
            a_accel_max=1.0,
            a_lat_max=10.0,
        ),
    )

    assert verdict.failure_category == "LONGITUDINAL_ACCEL_EXCEEDED"


def test_kc04_near_zero_negative_regret_is_clamped():
    optimal_time = 2.0
    trajectory = kc04_trajectory()
    trajectory[-1] = (2.0, 0.0, 1.0, optimal_time - 0.5 * FLOAT_ABS_TOL)
    verdict = evaluate_scenario(
        trajectory,
        kc04_scenario(optimal_time=optimal_time),
    )

    assert verdict.status == "PASS"
    assert verdict.metrics["time_regret"] == 0.0
    assert "REGRET_BELOW_ZERO_WARNING" not in verdict.warnings
