"""Calibrate the KC-02 reference planner and deliberate mutants."""

from __future__ import annotations

import json
import sys
import argparse
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kinocheck.grader import evaluate_scenario
from kinocheck.runner import CandidateSpec, run_benchmark
from tasks.kc02.mutants import MUTANTS


SCENARIOS_PATH = ROOT / "tasks" / "kc02" / "scenarios.json"
SCENARIOS = json.loads(SCENARIOS_PATH.read_text(encoding="utf-8"))
CANDIDATES = [
    CandidateSpec(name="REFERENCE", module_path="tasks.kc02.reference"),
    *(
        CandidateSpec(
            name=name,
            module_path="tasks.kc02.mutants",
            entrypoint=getattr(planner, "__name__"),
        )
        for name, planner in MUTANTS.items()
    ),
]


def evaluate(candidate: Callable[[dict], Any], scenario: dict) -> tuple[str, Any]:
    verdict = evaluate_scenario(candidate(scenario), scenario)
    return verdict.failure_category or "PASS", verdict.result


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate the KC-02 reference planner and deliberate mutants."
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        help="Write the structured benchmark report to this path.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    report = run_benchmark(CANDIDATES, SCENARIOS, timeout_seconds=5.0)
    if args.json_out is not None:
        args.json_out.write_text(
            json.dumps(report, indent=2),
            encoding="utf-8",
        )

    scenario_ids = [scenario["id"] for scenario in SCENARIOS]
    print("| Candidate | " + " | ".join(scenario_ids) + " |")
    print("|---|" + "|".join("---" for _ in scenario_ids) + "|")
    reports_by_name = {
        candidate_report["candidate_name"]: candidate_report
        for candidate_report in report["candidate_reports"]
    }
    for candidate in CANDIDATES:
        candidate_name = candidate.name
        candidate_report = reports_by_name[candidate_name]
        scenarios_by_id = {
            scenario["scenario_id"]: scenario
            for scenario in candidate_report["scenarios"]
        }
        cells = []
        for scenario_id in scenario_ids:
            scenario_report = scenarios_by_id[scenario_id]
            if scenario_report["status"] == "PASS":
                cells.append("PASS")
            else:
                failure = (
                    scenario_report["failure_category"]
                    or scenario_report["execution_status"]
                )
                cells.append(f"FAIL ({failure})")
        print(f"| {candidate_name} | " + " | ".join(cells) + " |")

    print("\n### Kill rates")
    scenario_count = len(SCENARIOS)
    for candidate in CANDIDATES:
        candidate_report = reports_by_name[candidate.name]
        killed = candidate_report["failed"]
        print(
            f"- {candidate.name}: {killed}/{scenario_count} "
            f"({killed / scenario_count:.1%})"
        )


if __name__ == "__main__":
    main()
