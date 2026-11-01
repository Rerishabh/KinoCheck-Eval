"""Isolated benchmark execution for planner candidates."""

from __future__ import annotations

import copy
import importlib
import importlib.util
import json
import math
import multiprocessing
from pathlib import Path
import pickle
import queue as queue_module
import time
from dataclasses import dataclass
from multiprocessing.queues import Queue
from numbers import Real
from typing import Any, Callable, Literal

from .grader import evaluate_scenario


ExecutionStatus = Literal[
    "COMPLETED",
    "TIMEOUT",
    "LOAD_ERROR",
    "RUNTIME_ERROR",
    "CRASH",
]


@dataclass(frozen=True)
class CandidateSpec:
    name: str
    module_path: str | None = None
    entrypoint: str = "plan"
    func: Callable[[dict], object] | None = None
    file_path: str | None = None

    def __post_init__(self) -> None:
        module_candidate = self.module_path is not None
        file_candidate = self.file_path is not None
        callable_candidate = self.func is not None
        if sum((module_candidate, file_candidate, callable_candidate)) != 1:
            raise ValueError(
                "CandidateSpec requires exactly one of module_path, file_path, or func."
            )
        if (module_candidate or file_candidate) and not self.entrypoint:
            raise ValueError("A module candidate requires a non-empty entrypoint.")
        if file_candidate:
            object.__setattr__(
                self,
                "file_path",
                str(Path(self.file_path).expanduser().resolve()),
            )


@dataclass(frozen=True)
class ExecutionResult:
    execution_status: ExecutionStatus
    result: object = None
    error: str | None = None
    execution_time_ms: float = 0.0


@dataclass(frozen=True)
class ScenarioReport:
    scenario_id: str
    status: Literal["PASS", "FAIL"]
    execution_time_ms: float
    failure_category: str | None = None
    reason: str | None = None
    result: object = None
    execution_status: ExecutionStatus = "COMPLETED"


@dataclass(frozen=True)
class BenchmarkReport:
    candidate_name: str
    total_runtime_ms: float
    scenarios: tuple[ScenarioReport, ...]
    passed: int
    failed: int
    overall_score: float
    overall_verdict: Literal["PASS", "FAIL"]
    schema_version: str = "1.0"
    task_id: str = "KC-02"


def _worker_run(
    module_path: str,
    file_path: str | None,
    entrypoint: str,
    scenario: dict,
    result_queue: Queue,
) -> None:
    """Load and execute a module candidate inside a spawned worker."""
    started = time.perf_counter()
    try:
        if file_path is not None:
            module_name = f"kinocheck_submission_{abs(hash(file_path))}"
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"Unable to load submission file: {file_path}")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        else:
            module = importlib.import_module(module_path)
        candidate = getattr(module, entrypoint)
        if not callable(candidate):
            raise TypeError(f"{entrypoint!r} is not callable")
    except Exception as error:
        _put_message(
            result_queue,
            {
                "execution_status": "LOAD_ERROR",
                "error": f"{type(error).__name__}: {error}",
                "execution_time_ms": _elapsed_ms(started),
            },
        )
        return

    try:
        result = candidate(copy.deepcopy(scenario))
    except Exception as error:
        _put_message(
            result_queue,
            {
                "execution_status": "RUNTIME_ERROR",
                "error": f"{type(error).__name__}: {error}",
                "execution_time_ms": _elapsed_ms(started),
            },
        )
        return

    _put_message(
        result_queue,
        {
            "execution_status": "COMPLETED",
            "result": result,
            "execution_time_ms": _elapsed_ms(started),
        },
    )


def _worker_run_callable(
    func: Callable[[dict], object],
    scenario: dict,
    result_queue: Queue,
) -> None:
    """Execute a picklable test callable inside a spawned worker."""
    started = time.perf_counter()
    try:
        result = func(copy.deepcopy(scenario))
    except Exception as error:
        _put_message(
            result_queue,
            {
                "execution_status": "RUNTIME_ERROR",
                "error": f"{type(error).__name__}: {error}",
                "execution_time_ms": _elapsed_ms(started),
            },
        )
        return

    _put_message(
        result_queue,
        {
            "execution_status": "COMPLETED",
            "result": result,
            "execution_time_ms": _elapsed_ms(started),
        },
    )


def _put_message(result_queue: Queue, message: dict[str, object]) -> None:
    try:
        payload = pickle.dumps(message)
    except Exception as error:
        payload = pickle.dumps(
            {
                "execution_status": "RUNTIME_ERROR",
                "error": (
                    "Result serialization failed: "
                    f"{type(error).__name__}: {error}"
                ),
                "execution_time_ms": message.get("execution_time_ms", 0.0),
            }
        )
    result_queue.put(payload)


def _elapsed_ms(started: float) -> float:
    return (time.perf_counter() - started) * 1000.0


def _execution_from_message(
    payload: bytes,
    elapsed_ms: float,
) -> ExecutionResult:
    try:
        message = pickle.loads(payload)
        status = message["execution_status"]
        if status not in {
            "COMPLETED",
            "LOAD_ERROR",
            "RUNTIME_ERROR",
        }:
            raise ValueError(f"Unknown worker status: {status!r}")
        return ExecutionResult(
            execution_status=status,
            result=message.get("result"),
            error=message.get("error"),
            execution_time_ms=elapsed_ms,
        )
    except Exception as error:
        return ExecutionResult(
            execution_status="CRASH",
            error=f"Invalid worker response: {type(error).__name__}: {error}",
            execution_time_ms=elapsed_ms,
        )


def run_candidate_scenario(
    candidate: CandidateSpec,
    scenario: dict,
    timeout_seconds: float,
) -> ScenarioReport:
    if timeout_seconds <= 0:
        raise ValueError("timeout_seconds must be positive.")

    started = time.perf_counter()
    context = multiprocessing.get_context("spawn")
    result_queue = context.Queue(maxsize=1)
    module_candidate = (
        candidate.module_path is not None or candidate.file_path is not None
    )
    target = _worker_run if module_candidate else _worker_run_callable
    args = (
        (
            candidate.module_path,
            candidate.file_path,
            candidate.entrypoint,
            scenario,
            result_queue,
        )
        if module_candidate
        else (candidate.func, scenario, result_queue)
    )
    process = context.Process(target=target, args=args)
    execution: ExecutionResult | None = None

    try:
        try:
            process.start()
        except Exception as error:
            execution = ExecutionResult(
                execution_status="CRASH",
                error=f"Worker start failed: {type(error).__name__}: {error}",
                execution_time_ms=_elapsed_ms(started),
            )
        else:
            process.join(timeout_seconds)
            if process.is_alive():
                process.terminate()
                process.join()
                execution = ExecutionResult(
                    execution_status="TIMEOUT",
                    error=f"Exceeded {timeout_seconds}s timeout.",
                    execution_time_ms=_elapsed_ms(started),
                )
            else:
                try:
                    payload = result_queue.get(timeout=0.2)
                except queue_module.Empty:
                    execution = ExecutionResult(
                        execution_status=(
                            "CRASH" if process.exitcode else "RUNTIME_ERROR"
                        ),
                        error=(
                            f"Worker exited with code {process.exitcode}."
                            if process.exitcode
                            else "Worker produced no result."
                        ),
                        execution_time_ms=_elapsed_ms(started),
                    )
                else:
                    execution = _execution_from_message(
                        payload, _elapsed_ms(started)
                    )

        if execution is None:
            execution = ExecutionResult(
                execution_status="CRASH",
                error="Worker produced no execution result.",
                execution_time_ms=_elapsed_ms(started),
            )

        if execution.execution_status != "COMPLETED":
            return ScenarioReport(
                scenario_id=scenario["id"],
                status="FAIL",
                execution_time_ms=execution.execution_time_ms,
                reason=execution.error,
                execution_status=execution.execution_status,
            )

        verdict = evaluate_scenario(execution.result, scenario)
        return ScenarioReport(
            scenario_id=scenario["id"],
            status=verdict.status,
            execution_time_ms=execution.execution_time_ms,
            failure_category=verdict.failure_category,
            reason=verdict.reason,
            result=verdict.result,
            execution_status="COMPLETED",
        )
    finally:
        try:
            alive = process.is_alive()
        except Exception:
            alive = False
        if alive:
            try:
                process.terminate()
            except Exception:
                pass
        try:
            process.join()
        except Exception:
            pass
        if execution is not None and execution.execution_status != "COMPLETED":
            try:
                result_queue.cancel_join_thread()
            except Exception:
                pass
        try:
            result_queue.close()
        except Exception:
            pass
        try:
            result_queue.join_thread()
        except Exception:
            pass


def _json_safe(value: object, seen: set[int] | None = None) -> object:
    if seen is None:
        seen = set()
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    value_id = id(value)
    if value_id in seen:
        return "<cycle>"
    if isinstance(value, dict):
        seen.add(value_id)
        sanitized = {
            str(key): _json_safe(item, seen) for key, item in value.items()
        }
        seen.remove(value_id)
        return sanitized
    if isinstance(value, (list, tuple)):
        seen.add(value_id)
        sanitized = [_json_safe(item, seen) for item in value]
        seen.remove(value_id)
        return sanitized
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError, OverflowError):
        try:
            representation = repr(value)
            return (
                representation
                if isinstance(representation, str)
                else f"<{type(value).__name__} (unserializable)>"
            )
        except Exception:
            return f"<{type(value).__name__} (unserializable)>"
    return value


def _report_dict(report: BenchmarkReport) -> dict[str, object]:
    return {
        "candidate_name": report.candidate_name,
        "total_runtime_ms": report.total_runtime_ms,
        "scenarios": [
            {
                "scenario_id": scenario.scenario_id,
                "status": scenario.status,
                "execution_time_ms": scenario.execution_time_ms,
                "failure_category": scenario.failure_category,
                "reason": scenario.reason,
                "result": _json_safe(scenario.result),
                "execution_status": scenario.execution_status,
            }
            for scenario in report.scenarios
        ],
        "passed": report.passed,
        "failed": report.failed,
        "overall_score": report.overall_score,
        "overall_verdict": report.overall_verdict,
        "schema_version": report.schema_version,
        "task_id": report.task_id,
    }


def run_benchmark(
    candidates: list[CandidateSpec],
    scenarios: list[dict],
    timeout_seconds: float,
) -> dict[str, object]:
    reports: list[dict[str, object]] = []
    for candidate in candidates:
        started = time.perf_counter()
        scenario_reports = tuple(
            run_candidate_scenario(candidate, scenario, timeout_seconds)
            for scenario in scenarios
        )
        passed = sum(report.status == "PASS" for report in scenario_reports)
        failed = len(scenario_reports) - passed
        reports.append(
            _report_dict(
                BenchmarkReport(
                    candidate_name=candidate.name,
                    total_runtime_ms=_elapsed_ms(started),
                    scenarios=scenario_reports,
                    passed=passed,
                    failed=failed,
                    overall_score=(
                        passed / len(scenario_reports)
                        if scenario_reports
                        else 0.0
                    ),
                    overall_verdict="PASS" if failed == 0 else "FAIL",
                )
            )
        )
    return {
        "schema_version": "1.0",
        "task_id": "KC-02",
        "candidate_reports": reports,
    }
