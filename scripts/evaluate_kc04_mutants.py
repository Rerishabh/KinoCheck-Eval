"""Calibrate the KC-04 reference speed profiler and deliberate mutants."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kinocheck.grader import evaluate_scenario
from kinocheck.runner import CandidateSpec, run_benchmark
from tasks.kc04.mutants import MUTANTS


SCENARIOS_PATH = ROOT / "tasks" / "kc04" / "scenarios.json"
SCENARIOS = json.loads(SCENARIOS_PATH.read_text(encoding="utf-8"))
CANDIDATES = [
    CandidateSpec(name="REFERENCE", module_path="tasks.kc04.reference"),
    *(
        CandidateSpec(
            name=name,
            module_path="tasks.kc04.mutants",
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

    reference = by_name["REFERENCE"]
    reference_ok = reference["failed"] == 0
    print(f"\nReference calibration: {'PASS' if reference_ok else 'FAIL'}")

    expected_kills = {
        "M1_CURVATURE_BLIND": "LATERAL_ACCEL_EXCEEDED",
        "M2_INFINITE_BRAKING": "LONGITUDINAL_ACCEL_EXCEEDED",
        "M3_ENDPOINT_SPEED_ERROR": "ENDPOINT_SPEED_ERROR",
        "M4_SPEED_LIMIT_EXCEEDED": "SPEED_LIMIT_EXCEEDED",
        "M5_FALSE_INFEASIBLE": "FALSE_INFEASIBLE",
    }
    kill_results = {}
    for name, category in expected_kills.items():
        killed = any(
            scenario["failure_category"] == category
            for scenario in by_name[name]["scenarios"]
        )
        kill_results[name] = killed
        print(f"{name} targeted kill ({category}): {'PASS' if killed else 'FAIL'}")

    kill_count = sum(kill_results.values())
    print(f"\nMutant kill rate: {kill_count}/5 ({kill_count / 5:.1%})")

    if args.json_out is not None:
        output = dict(report)
        output["calibration"] = {
            "reference_passed": reference_ok,
            "mutant_kills": kill_results,
            "kill_count": kill_count,
            "kill_rate": kill_count / 5,
        }
        args.json_out.write_text(json.dumps(output, indent=2), encoding="utf-8")

    if not reference_ok or kill_count != 5:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
