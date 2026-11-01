"""Canonical reference planner for KC-02."""

from __future__ import annotations

from collections import deque

from kinocheck.geometry import (
    create_obstacle_boxes,
    segment_min_distance,
    verify_polyline_clearance,
)
from kinocheck.tolerances import CLEARANCE_MARGIN_METERS, FLOAT_ABS_TOL


def plan(problem: dict) -> list[tuple[float, float]] | None:
    grid = problem["grid"]
    cell_size = problem["cell_size"]
    radius = problem["radius"]
    start = (float(problem["start"][0]), float(problem["start"][1]))
    goal = (float(problem["goal"][0]), float(problem["goal"][1]))

    req_clearance = radius + CLEARANCE_MARGIN_METERS
    width = len(grid) * cell_size
    height = len(grid[0]) * cell_size
    obstacles = create_obstacle_boxes(grid, cell_size)

    def boundary_clear(segment_start: tuple[float, float], segment_end: tuple[float, float]) -> bool:
        x1, y1 = segment_start
        x2, y2 = segment_end
        return (
            min(x1, x2) >= req_clearance - FLOAT_ABS_TOL
            and max(x1, x2) <= width - req_clearance + FLOAT_ABS_TOL
            and min(y1, y2) >= req_clearance - FLOAT_ABS_TOL
            and max(y1, y2) <= height - req_clearance + FLOAT_ABS_TOL
        )

    def obstacle_clear(
        segment_start: tuple[float, float], segment_end: tuple[float, float]
    ) -> bool:
        return verify_polyline_clearance(
            [segment_start, segment_end],
            obstacles,
            radius,
            CLEARANCE_MARGIN_METERS,
        )[0]

    def segment_clear(
        segment_start: tuple[float, float], segment_end: tuple[float, float]
    ) -> bool:
        return boundary_clear(segment_start, segment_end) and obstacle_clear(
            segment_start, segment_end
        )

    def point_clear(point: tuple[float, float]) -> bool:
        if not boundary_clear(point, point):
            return False
        return (
            segment_min_distance(point, point, obstacles)
            >= req_clearance - FLOAT_ABS_TOL
        )

    if not point_clear(start) or not point_clear(goal):
        return None

    if (
        abs(start[0] - goal[0]) <= FLOAT_ABS_TOL
        and abs(start[1] - goal[1]) <= FLOAT_ABS_TOL
    ):
        return [start]

    start_cell = (
        min(len(grid) - 1, max(0, int(start[0] // cell_size))),
        min(len(grid[0]) - 1, max(0, int(start[1] // cell_size))),
    )
    goal_cell = (
        min(len(grid) - 1, max(0, int(goal[0] // cell_size))),
        min(len(grid[0]) - 1, max(0, int(goal[1] // cell_size))),
    )

    free_cells = {
        (i, j)
        for i, column in enumerate(grid)
        for j, value in enumerate(column)
        if value == 0
    }
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None

    def cell_center(cell: tuple[int, int]) -> tuple[float, float]:
        i, j = cell
        return ((i + 0.5) * cell_size, (j + 0.5) * cell_size)

    queue: deque[tuple[int, int]] = deque([start_cell])
    previous: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
    for_cell_directions = ((1, 0), (-1, 0), (0, 1), (0, -1))

    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break
        for di, dj in for_cell_directions:
            neighbor = (current[0] + di, current[1] + dj)
            if neighbor not in free_cells or neighbor in previous:
                continue
            if segment_clear(cell_center(current), cell_center(neighbor)):
                previous[neighbor] = current
                queue.append(neighbor)

    if goal_cell not in previous:
        return None

    cell_path: list[tuple[int, int]] = []
    current: tuple[int, int] | None = goal_cell
    while current is not None:
        cell_path.append(current)
        current = previous[current]
    cell_path.reverse()
    centers = [cell_center(cell) for cell in cell_path]

    raw_path: list[tuple[float, float]] = [start]
    if start != centers[0]:
        if not segment_clear(start, centers[0]):
            return None
        raw_path.append(centers[0])
    for center in centers[1:]:
        raw_path.append(center)
    if goal != centers[-1]:
        if not segment_clear(centers[-1], goal):
            return None
        raw_path.append(goal)

    deduplicated: list[tuple[float, float]] = []
    for point in raw_path:
        if not deduplicated or point != deduplicated[-1]:
            deduplicated.append(point)

    smoothed: list[tuple[float, float]] = [deduplicated[0]]
    current_index = 0
    while current_index < len(deduplicated) - 1:
        furthest_index = current_index + 1
        for candidate_index in range(
            len(deduplicated) - 1, current_index, -1
        ):
            if segment_clear(
                deduplicated[current_index], deduplicated[candidate_index]
            ):
                furthest_index = candidate_index
                break
        smoothed.append(deduplicated[furthest_index])
        current_index = furthest_index

    smoothed[0] = start
    smoothed[-1] = goal
    return smoothed
