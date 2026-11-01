"""Calibrate the KC-02 reference planner and deliberate mutants."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kinocheck.geometry import create_obstacle_boxes, verify_polyline_clearance
from kinocheck.tolerances import CLEARANCE_MARGIN_METERS, FLOAT_ABS_TOL
from tasks.kc02.mutants import MUTANTS
from tasks.kc02.reference import plan


SCENARIOS_PATH = ROOT / "tasks" / "kc02" / "scenarios.json"
SCENARIOS = json.loads(SCENARIOS_PATH.read_text(encoding="utf-8"))
CANDIDATES: dict[str, Callable[[dict], Any]] = {"REFERENCE": plan, **MUTANTS}


def _format_result(result: Any) -> tuple[bool, list[tuple[float, float]], str]:
    if not isinstance(result, (list, tuple)) or len(result) < 2:
        return False, [], "INVALID_OUTPUT"

    points: list[tuple[float, float]] = []
    for point in result:
        if (
            not isinstance(point, (list, tuple))
            or len(point) != 2
            or not all(isinstance(value, (int, float)) for value in point)
        ):
            return False, [], "INVALID_OUTPUT"
        points.append((float(point[0]), float(point[1])))
    return True, points, ""


def evaluate(candidate: Callable[[dict], Any], scenario: dict) -> tuple[str, Any]:
    try:
        result = candidate(scenario)
    except Exception as error:
        return "INVALID_OUTPUT", f"{type(error).__name__}: {error}"

    if not scenario["feasible"]:
        if result is None:
            return "PASS", result
        return "FALSE_FEASIBLE", result

    if result is None:
        return "FALSE_INFEASIBLE", result

    valid, points, category = _format_result(result)
    if not valid:
        return category, result

    start = scenario["start"]
    goal = scenario["goal"]
    if (
        abs(points[0][0] - start[0]) > 1e-9
        or abs(points[0][1] - start[1]) > 1e-9
        or abs(points[-1][0] - goal[0]) > 1e-9
        or abs(points[-1][1] - goal[1]) > 1e-9
    ):
        return "ENDPOINT_ERROR", result

    required_clearance = (
        scenario["radius"] + CLEARANCE_MARGIN_METERS - FLOAT_ABS_TOL
    )
    width = len(scenario["grid"]) * scenario["cell_size"]
    height = len(scenario["grid"][0]) * scenario["cell_size"]
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        if (
            min(x1, x2) < required_clearance
            or max(x1, x2) > width - required_clearance
            or min(y1, y2) < required_clearance
            or max(y1, y2) > height - required_clearance
        ):
            return "OUT_OF_BOUNDS", result

    obstacles = create_obstacle_boxes(scenario["grid"], scenario["cell_size"])
    clear, _reason = verify_polyline_clearance(
        points,
        obstacles,
        scenario["radius"],
        CLEARANCE_MARGIN_METERS,
    )
    if not clear:
        category = (
            "COLLISION_CORNER"
            if scenario["id"] == "s1_diagonal_touch_pinch"
            else "COLLISION_SWEPT"
        )
        return category, result
    return "PASS", result


def main() -> None:
    results: dict[tuple[str, str], tuple[str, Any]] = {}
    for candidate_name, candidate in CANDIDATES.items():
        for scenario in SCENARIOS:
            results[(candidate_name, scenario["id"])] = evaluate(
                candidate, scenario
            )

    scenario_ids = [scenario["id"] for scenario in SCENARIOS]
    print("| Candidate | " + " | ".join(scenario_ids) + " |")
    print("|---|" + "|".join("---" for _ in scenario_ids) + "|")
    for candidate_name in CANDIDATES:
        cells = []
        for scenario_id in scenario_ids:
            category, _result = results[(candidate_name, scenario_id)]
            cells.append("PASS" if category == "PASS" else f"FAIL ({category})")
        print(f"| {candidate_name} | " + " | ".join(cells) + " |")

    print("\n### Kill rates")
    scenario_count = len(SCENARIOS)
    for candidate_name in CANDIDATES:
        killed = sum(
            results[(candidate_name, scenario_id)][0] != "PASS"
            for scenario_id in scenario_ids
        )
        print(
            f"- {candidate_name}: {killed}/{scenario_count} "
            f"({killed / scenario_count:.1%})"
        )


if __name__ == "__main__":
    main()
