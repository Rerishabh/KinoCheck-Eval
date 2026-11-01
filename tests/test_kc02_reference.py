import json
from pathlib import Path

import pytest

from kinocheck.geometry import create_obstacle_boxes, verify_polyline_clearance
from kinocheck.tolerances import CLEARANCE_MARGIN_METERS, FLOAT_ABS_TOL
from tasks.kc02 import reference


SCENARIOS_PATH = (
    Path(__file__).parent.parent / "tasks" / "kc02" / "scenarios.json"
)
SCENARIOS = json.loads(SCENARIOS_PATH.read_text(encoding="utf-8"))


@pytest.mark.parametrize("scenario", SCENARIOS, ids=[scenario["id"] for scenario in SCENARIOS])
def test_reference_plan_matches_frozen_scenario(scenario):
    result = reference.plan(scenario)

    if not scenario["feasible"]:
        assert result is None
        return

    assert result is not None
    assert len(result) >= 2
    assert abs(result[0][0] - scenario["start"][0]) <= 1e-9
    assert abs(result[0][1] - scenario["start"][1]) <= 1e-9
    assert abs(result[-1][0] - scenario["goal"][0]) <= 1e-9
    assert abs(result[-1][1] - scenario["goal"][1]) <= 1e-9

    obstacles = create_obstacle_boxes(scenario["grid"], scenario["cell_size"])
    valid, reason = verify_polyline_clearance(
        result,
        obstacles,
        scenario["radius"],
        CLEARANCE_MARGIN_METERS,
    )
    assert valid is True, reason

    req_clearance = scenario["radius"] + CLEARANCE_MARGIN_METERS
    width = len(scenario["grid"]) * scenario["cell_size"]
    height = len(scenario["grid"][0]) * scenario["cell_size"]
    boundary_limit = req_clearance - FLOAT_ABS_TOL
    for (x1, y1), (x2, y2) in zip(result, result[1:]):
        assert min(x1, x2) >= boundary_limit
        assert max(x1, x2) <= width - boundary_limit
        assert min(y1, y2) >= boundary_limit
        assert max(y1, y2) <= height - boundary_limit
