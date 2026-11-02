import json
from pathlib import Path

import pytest

from kinocheck.grader import _validate_kc04_scenario, evaluate_scenario
from kinocheck.tolerances import FLOAT_ABS_TOL
from tasks.kc04.mutants import MUTANTS
from tasks.kc04.reference import plan


SCENARIOS_PATH = Path(__file__).parents[1] / "tasks" / "kc04" / "scenarios.json"
SCENARIOS = json.loads(SCENARIOS_PATH.read_text(encoding="utf-8"))


def test_kc04_scenarios_validate_and_reference_passes():
    assert len(SCENARIOS) == 6
    for scenario in SCENARIOS:
        _validate_kc04_scenario(scenario)
        result = plan(scenario)
        verdict = evaluate_scenario(result, scenario)
        assert verdict.status == "PASS", (scenario["id"], verdict)
        if scenario["feasible"]:
            assert verdict.metrics["time_regret"] == pytest.approx(
                0.0, abs=FLOAT_ABS_TOL
            )
        else:
            assert result is None


@pytest.mark.parametrize(
    ("mutant_name", "failure_category"),
    [
        ("M1_CURVATURE_BLIND", "LATERAL_ACCEL_EXCEEDED"),
        ("M2_INFINITE_BRAKING", "LONGITUDINAL_ACCEL_EXCEEDED"),
        ("M3_ENDPOINT_SPEED_ERROR", "ENDPOINT_SPEED_ERROR"),
        ("M4_SPEED_LIMIT_EXCEEDED", "SPEED_LIMIT_EXCEEDED"),
        ("M5_FALSE_INFEASIBLE", "FALSE_INFEASIBLE"),
    ],
)
def test_kc04_mutants_are_killed(mutant_name, failure_category):
    mutant = MUTANTS[mutant_name]
    verdicts = [
        evaluate_scenario(mutant(scenario), scenario)
        for scenario in SCENARIOS
    ]
    assert any(
        verdict.status == "FAIL"
        and verdict.failure_category == failure_category
        for verdict in verdicts
    )
