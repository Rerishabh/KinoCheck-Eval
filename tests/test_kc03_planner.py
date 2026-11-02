import pytest

from kinocheck.grader import evaluate_scenario
from kinocheck.tolerances import FLOAT_ABS_TOL
from tasks.kc03 import mutants, reference


def scenario(**overrides):
    current = {
        "id": "kc03-test",
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


def test_reference_unobstructed_corridor_has_zero_regret():
    current = scenario()
    verdict = evaluate_scenario(reference.plan(current), current)

    assert verdict.status == "PASS"
    assert verdict.metrics["length_regret"] == pytest.approx(0.0)


def test_reference_chooses_longer_low_risk_route():
    current = scenario(
        grid=[[0, 0, 0], [0, 1, 0], [0, 0, 0]],
        risk_grid=[[1.0, 1.0, 1.0], [1.0, 20.0, 1.0], [1.0, 1.0, 1.0]],
        start=[0.5, 1.5],
        goal=[2.5, 1.5],
        risk_budget=5.0,
        optimal_length=4.0,
    )
    verdict = evaluate_scenario(reference.plan(current), current)

    assert verdict.status == "PASS"
    assert verdict.metrics["accumulated_risk"] <= current["risk_budget"]
    assert verdict.metrics["path_length"] > 2.0
    assert verdict.metrics["length_regret"] == pytest.approx(
        0.0, abs=FLOAT_ABS_TOL
    )


def test_reference_returns_none_when_risk_budget_is_impossible():
    current = scenario(risk_grid=[[10.0], [10.0], [10.0]], risk_budget=0.1)

    assert reference.plan(current) is None


def test_reference_returns_none_when_geometry_is_blocked():
    current = scenario(
        grid=[[0], [1], [0]],
        risk_grid=[[1.0], [1.0], [1.0]],
        start=[0.5, 0.5],
        goal=[2.5, 0.5],
    )

    assert reference.plan(current) is None


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("M1_RISK_BLIND", "BUDGET_EXCEEDED"),
        ("M2_RISK_OVER_CONSERVATIVE", "FALSE_INFEASIBLE"),
        ("M3A_OUT_OF_BOUNDS", "OUT_OF_BOUNDS"),
        ("M3B_COLLISION_SWEPT", "COLLISION_SWEPT"),
        ("M4_INVALID_OUTPUT", "INVALID_OUTPUT"),
    ],
)
def test_kc03_mutants_target_expected_categories(name, expected):
    if name == "M1_RISK_BLIND":
        current = scenario(
            grid=[[0, 0, 0], [0, 1, 0], [0, 0, 0]],
            risk_grid=[[1.0, 1.0, 1.0], [20.0, 20.0, 20.0], [1.0, 1.0, 1.0]],
            start=[0.5, 1.5],
            goal=[2.5, 1.5],
            risk_budget=5.0,
            optimal_length=2.0,
        )
    elif name == "M3A_OUT_OF_BOUNDS":
        current = scenario(
            grid=[[0], [0], [0]],
            start=[0.5, 0.05],
            goal=[2.5, 0.05],
            radius=0.1,
            risk_budget=10.0,
        )
    elif name == "M3B_COLLISION_SWEPT":
        current = scenario(
            grid=[[0, 0, 0], [0, 1, 0], [0, 0, 0]],
            risk_grid=[[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
            start=[0.5, 1.5],
            goal=[2.5, 1.5],
            risk_budget=10.0,
        )
    else:
        current = scenario()
    verdict = evaluate_scenario(mutants.MUTANTS[name](current), current)

    assert verdict.status == "FAIL"
    assert verdict.failure_category == expected


def test_reference_handles_coincident_lattice_endpoint_without_cycles():
    current = scenario(
        grid=[[0], [0]],
        risk_grid=[[1.0], [1.0]],
        start=[0.5, 0.5],
        goal=[1.5, 0.5],
        optimal_length=1.0,
    )

    path = reference.plan(current)
    verdict = evaluate_scenario(path, current)

    assert path == [(0.5, 0.5), (1.5, 0.5)]
    assert verdict.status == "PASS"
    assert verdict.metrics["length_regret"] == pytest.approx(0.0)
