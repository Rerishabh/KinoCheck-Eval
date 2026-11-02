"""Targeted KC-03 planner mutants."""

from __future__ import annotations

import math
import heapq

from kinocheck.geometry import (
    compute_polyline_risk,
    create_obstacle_boxes,
    verify_polyline_clearance,
)
from kinocheck.tolerances import CLEARANCE_MARGIN_METERS, FLOAT_ABS_TOL

from .reference import _build_nodes, _shortest_path


def plan_m1_risk_blind(problem: dict) -> list[tuple[float, float]] | None:
    """Return the shortest collision-free path without enforcing risk."""
    return _shortest_path(problem, enforce_budget=False)


def plan_m2_risk_over_conservative(problem: dict) -> list[tuple[float, float]] | None:
    """Incorrectly reject every feasible positive-budget problem."""
    if problem["feasible"] and problem["risk_budget"] > 0:
        return None
    return _shortest_path(problem, enforce_budget=True)


def _mutant_edges(problem: dict, *, perimeter: bool, obstacles: bool):
    grid = problem["grid"]
    size = float(problem["cell_size"])
    radius = float(problem["radius"])
    risk_grid = problem["risk_grid"]
    width = len(grid) * size
    height = len(grid[0]) * size
    points = _build_nodes(
        problem, perimeter=perimeter, obstacles=obstacles
    )
    start_id = len(points) - 2
    goal_id = len(points) - 1
    result = [[] for _ in points]
    lattice_nodes = {}
    for node_id, point in enumerate(points[:-2]):
        x = int(math.floor(point[0] / size))
        y = int(math.floor(point[1] / size))
        lattice_nodes[(x, y)] = node_id
    candidates: set[tuple[int, int]] = set()
    for (x, y), source in lattice_nodes.items():
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            target = lattice_nodes.get((x + dx, y + dy))
            if target is not None:
                candidates.add((source, target))
    for node_id in range(len(points)):
        if node_id != start_id:
            candidates.add((start_id, node_id))
        if node_id != goal_id:
            candidates.add((node_id, goal_id))
    candidates.add((start_id, goal_id))

    obstacle_boxes = create_obstacle_boxes(grid, size)
    for source, target in sorted(candidates):
        if source == target or math.hypot(
            points[target][0] - points[source][0],
            points[target][1] - points[source][1],
        ) <= FLOAT_ABS_TOL:
            continue
        start = points[source]
        goal = points[target]
        if perimeter and not (
            min(start[0], goal[0]) >= radius + CLEARANCE_MARGIN_METERS - FLOAT_ABS_TOL
            and max(start[0], goal[0]) <= width - radius - CLEARANCE_MARGIN_METERS + FLOAT_ABS_TOL
            and min(start[1], goal[1]) >= radius + CLEARANCE_MARGIN_METERS - FLOAT_ABS_TOL
            and max(start[1], goal[1]) <= height - radius - CLEARANCE_MARGIN_METERS + FLOAT_ABS_TOL
        ):
            continue
        if obstacles and not verify_polyline_clearance(
            [start, goal], obstacle_boxes, radius, CLEARANCE_MARGIN_METERS
        )[0]:
            continue
        result[source].append(
            (
                target,
                math.hypot(goal[0] - start[0], goal[1] - start[1]),
                compute_polyline_risk(
                    [start, goal], risk_grid, size
                ),
            )
        )
    return points, result, start_id, goal_id


def _shortest_mutant_path(problem: dict, *, perimeter: bool, obstacles: bool):
    points, edges, start_id, goal_id = _mutant_edges(
        problem, perimeter=perimeter, obstacles=obstacles
    )
    distances = {start_id: 0.0}
    previous = {}
    pending = [(0.0, start_id)]
    while pending:
        distance, source = heapq.heappop(pending)
        if source == goal_id:
            break
        for target, length, _risk in edges[source]:
            candidate = distance + length
            if candidate < distances.get(target, float("inf")):
                distances[target] = candidate
                previous[target] = source
                heapq.heappush(pending, (candidate, target))
    if goal_id not in distances:
        return None
    path_ids = [goal_id]
    while path_ids[-1] != start_id:
        path_ids.append(previous[path_ids[-1]])
    return [points[index] for index in reversed(path_ids)]


def plan_m3a_out_of_bounds(problem: dict) -> list[tuple[float, float]] | None:
    """Ignore perimeter clearance while retaining obstacle checks."""
    return _shortest_mutant_path(problem, perimeter=False, obstacles=True)


def plan_m3b_collision_swept(problem: dict) -> list[tuple[float, float]] | None:
    """Ignore swept obstacle checks while retaining perimeter checks."""
    return _shortest_mutant_path(problem, perimeter=True, obstacles=False)


def plan_m4_invalid_output(problem: dict) -> list[tuple[float, float]] | None:
    """Return a valid path with its start coordinate perturbed."""
    path = _shortest_path(problem, enforce_budget=True)
    if path is None:
        return None
    start_x, start_y = path[0]
    return [(start_x + 0.1, start_y)] + path[1:]


MUTANTS = {
    "M1_RISK_BLIND": plan_m1_risk_blind,
    "M2_RISK_OVER_CONSERVATIVE": plan_m2_risk_over_conservative,
    "M3A_OUT_OF_BOUNDS": plan_m3a_out_of_bounds,
    "M3B_COLLISION_SWEPT": plan_m3b_collision_swept,
    "M4_INVALID_OUTPUT": plan_m4_invalid_output,
}
