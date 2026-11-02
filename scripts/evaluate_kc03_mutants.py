"""Calibrate the KC-03 reference planner and deliberate mutants."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kinocheck.grader import evaluate_scenario
from kinocheck.runner import CandidateSpec, run_benchmark
from kinocheck.tolerances import FLOAT_ABS_TOL
from tasks.kc03.mutants import MUTANTS


SCENARIOS_PATH = ROOT / "tasks" / "kc03" / "scenarios.json"
SCENARIOS = json.loads(SCENARIOS_PATH.read_text(encoding="utf-8"))
CANDIDATES = [
    CandidateSpec(name="REFERENCE", module_path="tasks.kc03.reference"),
    *(
        CandidateSpec(
            name=name,
            module_path="tasks.kc03.mutants",
            entrypoint=planner.__name__,
        )
        for name, planner in MUTANTS.items()
    ),
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    report = run_benchmark(CANDIDATES, SCENARIOS, timeout_seconds=10.0)
    by_name = {item["candidate_name"]: item for item in report["candidate_reports"]}
    scenario_ids = [item["id"] for item in SCENARIOS]

    print("| Candidate | " + " | ".join(scenario_ids) + " |")
    print("|---|" + "|".join("---" for _ in scenario_ids) + "|")
    for candidate in CANDIDATES:
        item = by_name[candidate.name]
        cells = []
        for scenario in item["scenarios"]:
            cells.append(
                "PASS"
                if scenario["status"] == "PASS"
                else f"FAIL ({scenario['failure_category'] or scenario['execution_status']})"
            )
        print(f"| {candidate.name} | " + " | ".join(cells) + " |")

    print("\n### Kill rates")
    for candidate in CANDIDATES:
        item = by_name[candidate.name]
        print(f"- {candidate.name}: {item['failed']}/{len(SCENARIOS)} ({item['failed'] / len(SCENARIOS):.1%})")

    reference = by_name["REFERENCE"]
    reference_ok = reference["failed"] == 0
    print(f"\nReference calibration: {'PASS' if reference_ok else 'FAIL'}")
    expected_kills = {
        "M1_RISK_BLIND": "BUDGET_EXCEEDED",
        "M2_RISK_OVER_CONSERVATIVE": "FALSE_INFEASIBLE",
        "M3A_OUT_OF_BOUNDS": "OUT_OF_BOUNDS",
        "M3B_COLLISION_SWEPT": "COLLISION_SWEPT",
        "M4_INVALID_OUTPUT": "INVALID_OUTPUT",
    }
    for name, category in expected_kills.items():
        killed = any(
            scenario["failure_category"] == category
            for scenario in by_name[name]["scenarios"]
        )
        print(f"{name} targeted kill ({category}): {'PASS' if killed else 'FAIL'}")

    if args.json_out is not None:
        args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if not reference_ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
