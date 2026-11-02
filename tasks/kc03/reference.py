"""Canonical reference planner for KC-03."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from kinocheck.geometry import (
    compute_polyline_risk,
    create_obstacle_boxes,
    verify_polyline_clearance,
)
from kinocheck.tolerances import (
    CLEARANCE_MARGIN_METERS,
    FLOAT_ABS_TOL,
    RISK_ABS_TOL,
)


Point = tuple[float, float]


@dataclass(frozen=True)
class _Edge:
    target: int
    length: float
    risk: float


@dataclass(frozen=True)
class _Label:
    length: float
    risk: float
    node_id: int
    path: tuple[Point, ...]
    visited: frozenset[int]


def _boundary_clear(start: Point, goal: Point, width: float, height: float, clearance: float) -> bool:
    return (
        min(start[0], goal[0]) >= clearance
        and max(start[0], goal[0]) <= width - clearance
        and min(start[1], goal[1]) >= clearance
        and max(start[1], goal[1]) <= height - clearance
    )


def _edge_data(
    start: Point,
    goal: Point,
    obstacles: list,
    radius: float,
    cell_size: float,
    risk_grid: list[list[float]],
    width: float,
    height: float,
    perimeter_clearance: float,
) -> tuple[float, float] | None:
    if not _boundary_clear(start, goal, width, height, perimeter_clearance):
        return None
    clear, _reason = verify_polyline_clearance(
        [start, goal], obstacles, radius, CLEARANCE_MARGIN_METERS
    )
    if not clear:
        return None
    length = math.hypot(goal[0] - start[0], goal[1] - start[1])
    risk = compute_polyline_risk([start, goal], risk_grid, cell_size)
    return length, risk


def _build_nodes(
    problem: dict, *, perimeter: bool = True, obstacles: bool = True
) -> list[Point]:
    grid = problem["grid"]
    size = float(problem["cell_size"])
    radius = float(problem["radius"])
    width = len(grid) * size
    height = len(grid[0]) * size
    perimeter_clearance = radius + CLEARANCE_MARGIN_METERS - FLOAT_ABS_TOL
    obstacle_boxes = create_obstacle_boxes(grid, size)
    nodes = []
    for x, column in enumerate(grid):
        for y, value in enumerate(column):
            if value != 0:
                continue
            point = ((x + 0.5) * size, (y + 0.5) * size)
            if perimeter and not (
                perimeter_clearance <= point[0] <= width - perimeter_clearance
                and perimeter_clearance <= point[1] <= height - perimeter_clearance
            ):
                continue
            if obstacles and not verify_polyline_clearance(
                [point, point], obstacle_boxes, radius, CLEARANCE_MARGIN_METERS
            )[0]:
                continue
            nodes.append(point)
    start = (float(problem["start"][0]), float(problem["start"][1]))
    goal = (float(problem["goal"][0]), float(problem["goal"][1]))
    nodes.extend((start, goal))
    return nodes


def _build_graph(problem: dict, *, include_risk: bool = True) -> tuple[list[Point], list[list[_Edge]]]:
    grid = problem["grid"]
    size = float(problem["cell_size"])
    radius = float(problem["radius"])
    risk_grid = problem["risk_grid"]
    width = len(grid) * size
    height = len(grid[0]) * size
    perimeter_clearance = radius + CLEARANCE_MARGIN_METERS - FLOAT_ABS_TOL
    obstacles = create_obstacle_boxes(grid, size)
    nodes = _build_nodes(problem)
    node_ids = {point: index for index, point in enumerate(nodes)}
    edges: list[list[_Edge]] = [[] for _ in nodes]

    lattice_nodes = {}
    for node_id, point in enumerate(nodes[:-2]):
        x = int(math.floor(point[0] / size))
        y = int(math.floor(point[1] / size))
        lattice_nodes[(x, y)] = node_id
    candidates: set[tuple[int, int]] = set()
    for (x, y), source in lattice_nodes.items():
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            target = lattice_nodes.get((x + dx, y + dy))
            if target is not None:
                candidates.add((source, target))

    start_id = len(nodes) - 2
    goal_id = len(nodes) - 1
    for node_id in range(len(nodes)):
        if node_id != start_id:
            candidates.add((start_id, node_id))
        if node_id != goal_id:
            candidates.add((node_id, goal_id))
    candidates.add((start_id, goal_id))

    for source, target in sorted(candidates):
        if source == target or math.hypot(
            nodes[target][0] - nodes[source][0],
            nodes[target][1] - nodes[source][1],
        ) <= FLOAT_ABS_TOL:
            continue
        data = _edge_data(
            nodes[source],
            nodes[target],
            obstacles,
            radius,
            size,
            risk_grid,
            width,
            height,
            perimeter_clearance,
        )
        if data is None:
            continue
        length, risk = data
        if not include_risk:
            risk = 0.0
        edges[source].append(_Edge(target, length, risk))
    return nodes, edges


def _shortest_path(problem: dict, *, enforce_budget: bool) -> list[Point] | None:
    nodes, edges = _build_graph(problem, include_risk=enforce_budget)
    start_id = len(nodes) - 2
    goal_id = len(nodes) - 1
    budget = float(problem["risk_budget"])

    queue: list[tuple[float, float, int, int, _Label]] = []
    labels: list[list[_Label]] = [[] for _ in nodes]
    sequence = 0
    initial = _Label(0.0, 0.0, start_id, (nodes[start_id],), frozenset({start_id}))
    labels[start_id].append(initial)
    heapq.heappush(queue, (0.0, 0.0, start_id, sequence, initial))

    while queue:
        _priority_length, _priority_risk, _node_id, _sequence, label = heapq.heappop(queue)
        if label not in labels[label.node_id]:
            continue
        if label.node_id == goal_id:
            continue
        for edge in edges[label.node_id]:
            if edge.target in label.visited:
                continue
            new_risk = label.risk + edge.risk
            if enforce_budget and new_risk > budget + RISK_ABS_TOL:
                continue
            new_label = _Label(
                label.length + edge.length,
                new_risk,
                edge.target,
                label.path + (nodes[edge.target],),
                label.visited | {edge.target},
            )
            dominated = any(
                existing.length <= new_label.length
                and existing.risk <= new_label.risk
                and (
                    existing.length < new_label.length
                    or existing.risk < new_label.risk
                )
                for existing in labels[edge.target]
            )
            if dominated:
                continue
            labels[edge.target] = [
                existing
                for existing in labels[edge.target]
                if not (
                    new_label.length <= existing.length
                    and new_label.risk <= existing.risk
                    and (
                        new_label.length < existing.length
                        or new_label.risk < existing.risk
                    )
                )
            ]
            labels[edge.target].append(new_label)
            sequence += 1
            heapq.heappush(
                queue,
                (new_label.length, new_label.risk, new_label.node_id, sequence, new_label),
            )

    goal_labels = sorted(
        labels[goal_id], key=lambda label: (label.length, label.risk, label.path)
    )
    for label in goal_labels:
        candidate_path = list(label.path)
        if (
            not enforce_budget
            or compute_polyline_risk(
                candidate_path, problem["risk_grid"], problem["cell_size"]
            )
            <= budget + RISK_ABS_TOL
        ):
            return candidate_path
    return None


def plan(problem: dict) -> list[tuple[float, float]] | None:
    """Return the shortest collision-free path within the risk budget."""
    return _shortest_path(problem, enforce_budget=True)
