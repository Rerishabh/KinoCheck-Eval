from kinocheck.grader import evaluate_scenario


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
