import json
import math
from pathlib import Path

import pytest

from kinocheck.grader import _validate_kc03_scenario, evaluate_scenario
from kinocheck.tolerances import FLOAT_ABS_TOL
from tasks.kc03 import mutants, reference


ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = json.loads(
    (ROOT / "tasks" / "kc03" / "scenarios.json").read_text(encoding="utf-8")
)


def test_kc03_scenario_suite_schema_and_validation():
    assert [scenario["id"] for scenario in SCENARIOS] == [
        "s1_unobstructed_direct",
        "s2_risk_detour",
        "s3_tight_budget_corridor",
        "s4_budget_infeasible",
        "s5_geometrically_blocked",
        "s6_perimeter_sensitive",
        "s7_obstacle_chokepoint",
    ]
    for scenario in SCENARIOS:
        _validate_kc03_scenario(scenario)


def test_reference_passes_all_kc03_scenarios():
    for scenario in SCENARIOS:
        path = reference.plan(scenario)
        verdict = evaluate_scenario(path, scenario)
        assert verdict.status == "PASS", (scenario["id"], verdict)
        if scenario["feasible"]:
            assert verdict.metrics["length_regret"] == pytest.approx(
                0.0, abs=FLOAT_ABS_TOL
            )
        else:
            assert path is None


@pytest.mark.parametrize(
    ("name", "category"),
    [
        ("M1_RISK_BLIND", "BUDGET_EXCEEDED"),
        ("M2_RISK_OVER_CONSERVATIVE", "FALSE_INFEASIBLE"),
        ("M3A_OUT_OF_BOUNDS", "OUT_OF_BOUNDS"),
        ("M3B_COLLISION_SWEPT", "COLLISION_SWEPT"),
        ("M4_INVALID_OUTPUT", "INVALID_OUTPUT"),
    ],
)
def test_each_kc03_mutant_is_killed(name, category):
    observed = []
    for scenario in SCENARIOS:
        result = mutants.MUTANTS[name](scenario)
        observed.append(evaluate_scenario(result, scenario))
    assert any(verdict.failure_category == category for verdict in observed)
