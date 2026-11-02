import json
import math
import time
from pathlib import Path

import pytest

from kinocheck.runner import (
    BenchmarkReport,
    CandidateSpec,
    ScenarioReport,
    _report_dict,
    _json_safe,
    run_benchmark,
    run_candidate_scenario,
)
from tasks.kc04.reference import plan as kc04_plan
from tasks.kc02.reference import plan


ROOT = Path(__file__).parent.parent
SCENARIOS = json.loads(
    (ROOT / "tasks" / "kc02" / "scenarios.json").read_text(encoding="utf-8")
)
KC04_SCENARIO = json.loads(
    (ROOT / "tasks" / "kc04" / "scenarios.json").read_text(encoding="utf-8")
)[0]


def direct_reference(problem):
    return plan(problem)


def runtime_error(_problem):
    raise RuntimeError("candidate failed")


def slow_candidate(_problem):
    time.sleep(1.0)
    return None


def crash_candidate(_problem):
    raise SystemExit(3)


class Unserializable:
    def __repr__(self):
        raise RuntimeError("repr failed")


def test_reference_module_candidate_passes_all_scenarios():
    candidate = CandidateSpec(
        name="REFERENCE",
        module_path="tasks.kc02.reference",
    )

    reports = [
        run_candidate_scenario(candidate, scenario, 5.0)
        for scenario in SCENARIOS
    ]

    assert [report.status for report in reports] == ["PASS"] * len(SCENARIOS)


def test_direct_callable_candidate_is_spawned_and_evaluated():
    candidate = CandidateSpec(name="DIRECT", func=direct_reference)

    report = run_candidate_scenario(candidate, SCENARIOS[1], 5.0)

    assert report.status == "PASS"
    assert report.execution_status == "COMPLETED"


def test_scenario_report_carries_metrics_and_warnings():
    report = run_candidate_scenario(
        CandidateSpec(name="KC04", func=kc04_plan),
        KC04_SCENARIO,
        5.0,
    )

    assert report.status == "PASS"
    assert report.metrics is not None
    assert report.metrics["time_regret"] == 0.0
    assert report.warnings == []


def test_report_dict_serializes_metrics_and_warnings():
    report = BenchmarkReport(
        candidate_name="TEST",
        total_runtime_ms=1.0,
        scenarios=(
            ScenarioReport(
                scenario_id="s1",
                status="PASS",
                execution_time_ms=1.0,
                metrics={"total_time": 2.5},
                warnings=["REGRET_BELOW_ZERO_WARNING"],
            ),
        ),
        passed=1,
        failed=0,
        overall_score=1.0,
        overall_verdict="PASS",
        task_id="KC-04",
    )

    scenario = _report_dict(report)["scenarios"][0]
    assert scenario["metrics"] == {"total_time": 2.5}
    assert scenario["warnings"] == ["REGRET_BELOW_ZERO_WARNING"]


@pytest.mark.parametrize(
    ("scenarios", "task_id", "expected"),
    [
        (KC04_SCENARIO and [KC04_SCENARIO], None, "KC-04"),
        (SCENARIOS[:1], "KC-03", "KC-03"),
        ([{"id": "untyped"}], None, "KC-02"),
        ([], None, "KC-02"),
    ],
)
def test_run_benchmark_resolves_task_id(scenarios, task_id, expected):
    report = run_benchmark(
        [CandidateSpec(name="DIRECT", func=lambda _scenario: None)],
        scenarios,
        5.0,
        task_id=task_id,
    )

    assert report["task_id"] == expected


@pytest.mark.parametrize(
    "kwargs",
    [
        {},
        {"module_path": "tasks.kc02.reference", "func": direct_reference},
    ],
)
def test_candidate_spec_requires_exactly_one_execution_source(kwargs):
    with pytest.raises(ValueError):
        CandidateSpec(name="INVALID", **kwargs)


def test_missing_module_is_load_error():
    candidate = CandidateSpec(name="MISSING", module_path="does.not.exist")

    report = run_candidate_scenario(candidate, SCENARIOS[1], 5.0)

    assert report.execution_status == "LOAD_ERROR"
    assert report.status == "FAIL"
    assert report.failure_category is None


def test_missing_entrypoint_is_load_error():
    candidate = CandidateSpec(
        name="MISSING_ENTRYPOINT",
        module_path="tasks.kc02.reference",
        entrypoint="missing",
    )

    report = run_candidate_scenario(candidate, SCENARIOS[1], 5.0)

    assert report.execution_status == "LOAD_ERROR"


def test_candidate_runtime_exception_is_reported():
    report = run_candidate_scenario(
        CandidateSpec(name="ERROR", func=runtime_error),
        SCENARIOS[1],
        5.0,
    )

    assert report.execution_status == "RUNTIME_ERROR"
    assert "candidate failed" in (report.reason or "")


def test_timeout_is_reported_and_worker_is_terminated():
    report = run_candidate_scenario(
        CandidateSpec(name="SLOW", func=slow_candidate),
        SCENARIOS[1],
        0.05,
    )

    assert report.execution_status == "TIMEOUT"
    assert report.status == "FAIL"


def test_worker_crash_is_reported():
    report = run_candidate_scenario(
        CandidateSpec(name="CRASH", func=crash_candidate),
        SCENARIOS[1],
        5.0,
    )

    assert report.execution_status == "CRASH"


def test_benchmark_report_is_json_serializable_and_has_schema():
    report = run_benchmark(
        [CandidateSpec(name="REFERENCE", module_path="tasks.kc02.reference")],
        SCENARIOS,
        5.0,
    )

    encoded = json.dumps(report)
    decoded = json.loads(encoded)
    candidate_report = decoded["candidate_reports"][0]
    assert decoded["schema_version"] == "1.0"
    assert decoded["task_id"] == "KC-02"
    assert candidate_report["passed"] == len(SCENARIOS)
    assert candidate_report["failed"] == 0
    assert candidate_report["overall_verdict"] == "PASS"


def test_json_safe_handles_cycles_non_finite_values_and_bad_repr():
    cyclic = []
    cyclic.append(cyclic)

    sanitized = _json_safe(
        {
            "cycle": cyclic,
            "nan": math.nan,
            "positive": math.inf,
            "negative": -math.inf,
            "bad": Unserializable(),
        }
    )

    assert sanitized == {
        "cycle": ["<cycle>"],
        "nan": "nan",
        "positive": "inf",
        "negative": "-inf",
        "bad": "<Unserializable (unserializable)>",
    }


def test_non_picklable_candidate_is_reported_as_crash():
    report = run_candidate_scenario(
        CandidateSpec(name="NON_PICKLABLE", func=lambda _problem: None),
        SCENARIOS[1],
        5.0,
    )

    assert report.execution_status == "CRASH"
    assert report.status == "FAIL"
