import json
import subprocess
import sys
from pathlib import Path

import kinocheck.cli as cli
from kinocheck.cli import main


ROOT = Path(__file__).parent.parent


def test_argument_parser_rejects_non_positive_timeout(capsys):
    assert main(
        [
            "benchmark",
            "--task",
            "KC-02",
            "--submission",
            "tasks.kc02.reference",
            "--timeout",
            "0",
        ]
    ) == 2
    assert "timeout must be positive" in capsys.readouterr().err


def test_missing_submission_file_exits_two(capsys):
    assert main(
        [
            "benchmark",
            "--task",
            "KC-02",
            "--submission",
            "missing_submission.py",
        ]
    ) == 2
    assert "does not exist" in capsys.readouterr().err


def test_unknown_task_exits_two(capsys):
    assert main(
        [
            "benchmark",
            "--task",
            "KC-99",
            "--submission",
            "tasks.kc02.reference",
        ]
    ) == 2
    assert "Unknown task" in capsys.readouterr().err


def test_non_dict_task_metadata_exits_two(monkeypatch, tmp_path, capsys):
    task_path = tmp_path / "task.json"
    scenarios_path = tmp_path / "scenarios.json"
    task_path.write_text("[]", encoding="utf-8")
    scenarios_path.write_text("[]", encoding="utf-8")
    monkeypatch.setitem(
        cli.TASKS,
        "KC-02",
        (task_path, scenarios_path),
    )

    assert main(
        [
            "benchmark",
            "--task",
            "KC-02",
            "--submission",
            "tasks.kc02.reference",
        ]
    ) == 2
    assert "task metadata must be a JSON object" in capsys.readouterr().err


def test_file_candidate_with_spaces_runs_in_spawned_worker(tmp_path, capsys):
    submission = tmp_path / "planner with spaces.py"
    submission.write_text(
        "from tasks.kc02.reference import plan as reference_plan\n"
        "\n"
        "def plan(scenario):\n"
        "    return reference_plan(scenario)\n",
        encoding="utf-8",
    )
    result = main(
        [
            "benchmark",
            "--task",
            "KC-02",
            "--submission",
            str(submission),
            "--timeout",
            "5",
        ]
    )
    output = capsys.readouterr().out
    assert result == 0
    assert "Task: KC-02" in output
    assert "Verdict: PASS" in output


def test_missing_entrypoint_is_benchmark_failure(tmp_path, capsys):
    submission = tmp_path / "planner.py"
    submission.write_text("def plan(scenario): return None\n", encoding="utf-8")
    result = main(
        [
            "benchmark",
            "--task",
            "KC-02",
            "--submission",
            str(submission),
            "--entrypoint",
            "missing",
            "--timeout",
            "5",
        ]
    )
    output = capsys.readouterr().out
    assert result == 1
    assert "LOAD_ERROR" in output


def test_module_candidate_cli_invocation():
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "kinocheck.cli",
            "benchmark",
            "--task",
            "KC-02",
            "--submission",
            "tasks.kc02.reference",
            "--timeout",
            "5",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0
    assert "Verdict: PASS" in completed.stdout
