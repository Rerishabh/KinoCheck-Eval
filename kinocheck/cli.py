"""Command-line interface for running KinoCheck benchmarks."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .runner import CandidateSpec, run_benchmark


ROOT = Path(__file__).resolve().parents[1]
TASKS = {
    "KC-02": (
        ROOT / "tasks" / "kc02" / "task.json",
        ROOT / "tasks" / "kc02" / "scenarios.json",
    )
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m kinocheck.cli")
    commands = parser.add_subparsers(dest="command", required=True)
    benchmark = commands.add_parser("benchmark")
    benchmark.add_argument("--task", required=True)
    benchmark.add_argument("--submission", required=True)
    benchmark.add_argument("--entrypoint", default="plan")
    benchmark.add_argument("--timeout", type=float, default=5.0)
    benchmark.add_argument("--json-out", type=Path)
    return parser


def _load_task(task_id: str) -> tuple[dict, list[dict]]:
    try:
        task_path, scenarios_path = TASKS[task_id]
    except KeyError as error:
        raise ValueError(f"Unknown task: {task_id}") from error
    try:
        task = json.loads(task_path.read_text(encoding="utf-8"))
        scenarios = json.loads(scenarios_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Unable to load task {task_id}: {error}") from error
    if not isinstance(task, dict):
        raise ValueError("task metadata must be a JSON object")
    if not isinstance(scenarios, list):
        raise ValueError("scenarios must be a JSON array")
    if task.get("task_id") != task_id:
        raise ValueError(f"Malformed task data for {task_id}.")
    return task, scenarios


def _candidate(submission: str, entrypoint: str) -> CandidateSpec:
    path = Path(submission).expanduser()
    if path.exists():
        if not path.is_file():
            raise ValueError(f"Submission is not a file: {submission}")
        return CandidateSpec(
            name=str(path),
            file_path=str(path),
            entrypoint=entrypoint,
        )
    if path.suffix == ".py" or ("\\" in submission or "/" in submission):
        raise ValueError(f"Submission file does not exist: {submission}")
    return CandidateSpec(
        name=submission,
        module_path=submission,
        entrypoint=entrypoint,
    )


def _print_report(report: dict[str, object]) -> None:
    candidate = report["candidate_reports"][0]
    print(f"Task: {report['task_id']}")
    print(f"Candidate: {candidate['candidate_name']}")
    print(f"Scenarios: {len(candidate['scenarios'])}")
    print(f"Passed: {candidate['passed']}")
    print(f"Failed: {candidate['failed']}")
    print(f"Score: {candidate['overall_score']:.1%}")
    print(f"Verdict: {candidate['overall_verdict']}")
    print("\n| Scenario | Status | Failure |")
    print("|---|---|---|")
    for scenario in candidate["scenarios"]:
        failure = scenario["failure_category"] or scenario["execution_status"]
        print(
            f"| {scenario['scenario_id']} | {scenario['status']} | "
            f"{failure if scenario['status'] == 'FAIL' else '-'} |"
        )


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command != "benchmark":
        return 2
    try:
        if args.timeout <= 0:
            raise ValueError("timeout must be positive")
        task, scenarios = _load_task(args.task)
        candidate = _candidate(args.submission, args.entrypoint)
        report = run_benchmark([candidate], scenarios, args.timeout)
        if args.json_out is not None:
            args.json_out.write_text(
                json.dumps(report, indent=2),
                encoding="utf-8",
            )
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    _print_report(report)
    return 0 if report["candidate_reports"][0]["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
