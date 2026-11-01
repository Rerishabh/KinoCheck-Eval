import json
from pathlib import Path

from kinocheck.grader import evaluate_scenario
from tasks.kc02 import reference
from tasks.kc02.mutants import plan_m3_out_of_bounds
from scripts.evaluate_kc02_mutants import evaluate


SCENARIOS = json.loads(
    (
        Path(__file__).parent.parent / "tasks" / "kc02" / "scenarios.json"
    ).read_text(encoding="utf-8")
)


def make_scenario(
    *,
    scenario_id="evaluator",
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


def test_valid_path_passes_boundary_clearance():
    scenario = make_scenario()

    verdict, _result = evaluate(
        lambda _scenario: [(0.5, 0.5), (2.5, 0.5)],
        scenario,
    )

    assert verdict == "PASS"


def test_grazing_boundary_fails_out_of_bounds():
    scenario = make_scenario(
        start=(0.5, 0.25),
        goal=(2.5, 0.25),
    )

    verdict, _result = evaluate(
        lambda _scenario: [(0.5, 0.25), (2.5, 0.25)],
        scenario,
    )

    assert verdict == "OUT_OF_BOUNDS"


def test_out_of_arena_segment_fails_out_of_bounds():
    scenario = make_scenario(goal=(3.1, 0.5))

    verdict, _result = evaluate(
        lambda _scenario: [(0.5, 0.5), (3.1, 0.5)],
        scenario,
    )

    assert verdict == "OUT_OF_BOUNDS"


def test_diagonal_boundary_grazing_fails_out_of_bounds():
    scenario = make_scenario(
        start=(0.2, 1.5),
        goal=(1.5, 0.2),
    )

    verdict, _result = evaluate(
        lambda _scenario: [(0.2, 1.5), (1.5, 0.2)],
        scenario,
    )

    assert verdict == "OUT_OF_BOUNDS"


def test_swept_obstacle_intersection_is_classified():
    scenario = make_scenario(
        grid=[[0, 0, 0], [0, 1, 0], [0, 0, 0]],
        start=(0.5, 1.5),
        goal=(2.5, 1.5),
    )

    verdict, _result = evaluate(
        lambda _scenario: [(0.5, 1.5), (2.5, 1.5)],
        scenario,
    )

    assert verdict == "COLLISION_SWEPT"


def test_corner_grazing_clearance_violation_is_caught():
    scenario = make_scenario(
        scenario_id="s1_diagonal_touch_pinch",
        grid=[[0, 1], [1, 0]],
        start=(0.5, 0.5),
        goal=(1.5, 1.5),
    )

    verdict, _result = evaluate(
        lambda _scenario: [(0.5, 0.5), (1.5, 1.5)],
        scenario,
    )

    assert verdict == "COLLISION_CORNER"


def test_endpoint_mismatch_fails_endpoint_validation():
    scenario = make_scenario()

    verdict, _result = evaluate(
        lambda _scenario: [(0.5, 0.5), (2.4, 0.5)],
        scenario,
    )

    assert verdict == "ENDPOINT_ERROR"


def test_empty_path_fails_format_validation():
    scenario = make_scenario()

    verdict, _result = evaluate(lambda _scenario: [], scenario)

    assert verdict == "INVALID_OUTPUT"


def test_single_waypoint_fails_format_validation():
    scenario = make_scenario()

    verdict, _result = evaluate(lambda _scenario: [(0.5, 0.5)], scenario)

    assert verdict == "INVALID_OUTPUT"


def test_s7_reference_plan_passes():
    scenario = next(item for item in SCENARIOS if item["id"] == "s7_perimeter_graze")

    verdict = evaluate_scenario(reference.plan(scenario), scenario)

    assert verdict.status == "PASS"


def test_s7_m3_plan_fails_out_of_bounds():
    scenario = next(item for item in SCENARIOS if item["id"] == "s7_perimeter_graze")

    verdict = evaluate_scenario(plan_m3_out_of_bounds(scenario), scenario)

    assert verdict.status == "FAIL"
    assert verdict.failure_category == "OUT_OF_BOUNDS"
