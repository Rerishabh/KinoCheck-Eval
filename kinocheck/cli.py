"""Command-line interface for running KinoCheck benchmarks."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .runner import CandidateSpec, run_benchmark


ROOT = Path(__file__).resolve().parents[1]
TASKS_DIR = ROOT / "tasks"
TASKS = {
    "KC-02": (
        TASKS_DIR / "kc02" / "task.json",
        TASKS_DIR / "kc02" / "scenarios.json",
    ),
    "KC-03": (
        TASKS_DIR / "kc03" / "task.json",
        TASKS_DIR / "kc03" / "scenarios.json",
    ),
    "KC-04": (
        TASKS_DIR / "kc04" / "task.json",
        TASKS_DIR / "kc04" / "scenarios.json",
    )
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m kinocheck.cli")
    commands = parser.add_subparsers(dest="command", required=True)
    benchmark = commands.add_parser("benchmark")
    benchmark.add_argument(
        "--task",
        required=True,
        help=(
            "Task ID (KC-02, KC-03, KC-04), comma-separated task IDs, "
            "or 'all'."
        ),
    )
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


def _resolve_tasks(task_arg: str) -> list[str]:
    if task_arg.lower() == "all":
        return list(TASKS)

    task_ids = [task_id.strip() for task_id in task_arg.split(",")]
    if not task_ids or any(not task_id for task_id in task_ids):
        raise ValueError(f"Unknown task: {task_arg}")
    for task_id in task_ids:
        if task_id not in TASKS:
            raise ValueError(f"Unknown task: {task_id}")
    return task_ids


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


def _print_multi_report(reports: list[dict[str, object]]) -> None:
    for index, report in enumerate(reports):
        if index:
            print()
        print(f"=== Task {report['task_id']} ===")
        _print_report(report)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command != "benchmark":
        return 2
    try:
        if args.timeout <= 0:
            raise ValueError("timeout must be positive")
        task_ids = _resolve_tasks(args.task)
        candidate = _candidate(args.submission, args.entrypoint)
        reports = []
        for task_id in task_ids:
            task, scenarios = _load_task(task_id)
            reports.append(
                run_benchmark(
                    [candidate],
                    scenarios,
                    args.timeout,
                    task_id=task_id,
                )
            )
        if len(reports) == 1:
            report = reports[0]
            if args.json_out is not None:
                args.json_out.write_text(
                    json.dumps(report, indent=2),
                    encoding="utf-8",
                )
        else:
            report = {
                "tasks": reports,
                "overall_status": (
                    "PASS"
                    if all(
                        task_report["candidate_reports"][0]["overall_verdict"]
                        == "PASS"
                        for task_report in reports
                    )
                    else "FAIL"
                ),
            }
            if args.json_out is not None:
                args.json_out.write_text(
                    json.dumps(report, indent=2),
                    encoding="utf-8",
                )
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if len(reports) == 1:
        _print_report(reports[0])
        passed = reports[0]["candidate_reports"][0]["failed"] == 0
    else:
        _print_multi_report(reports)
        passed = report["overall_status"] == "PASS"
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
