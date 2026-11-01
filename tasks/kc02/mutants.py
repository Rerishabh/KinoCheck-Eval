"""Deliberately mutated KC-02 planners for evaluator tests."""

from __future__ import annotations

from collections import deque

from kinocheck.geometry import (
    create_obstacle_boxes,
    segment_min_distance,
    verify_polyline_clearance,
)
from kinocheck.tolerances import CLEARANCE_MARGIN_METERS, FLOAT_ABS_TOL


def _problem_values(problem: dict):
    grid = problem["grid"]
    cell_size = problem["cell_size"]
    radius = problem["radius"]
    start = (float(problem["start"][0]), float(problem["start"][1]))
    goal = (float(problem["goal"][0]), float(problem["goal"][1]))
    width = len(grid) * cell_size
    height = len(grid[0]) * cell_size
    obstacles = create_obstacle_boxes(grid, cell_size)
    return grid, cell_size, radius, start, goal, width, height, obstacles


def _cells(grid, cell_size, start, goal):
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
    return start_cell, goal_cell, free_cells


def _cell_center(cell, cell_size):
    i, j = cell
    return ((i + 0.5) * cell_size, (j + 0.5) * cell_size)


def plan_m1_collision_corner(problem: dict) -> list[tuple[float, float]] | None:
    """Allow 8-connected diagonal transitions without checking them."""
    (
        grid,
        cell_size,
        radius,
        start,
        goal,
        width,
        height,
        obstacles,
    ) = _problem_values(problem)
    req_clearance = radius + CLEARANCE_MARGIN_METERS

    def boundary_clear(point_a, point_b):
        return (
            min(point_a[0], point_b[0]) >= req_clearance - FLOAT_ABS_TOL
            and max(point_a[0], point_b[0]) <= width - req_clearance + FLOAT_ABS_TOL
            and min(point_a[1], point_b[1]) >= req_clearance - FLOAT_ABS_TOL
            and max(point_a[1], point_b[1]) <= height - req_clearance + FLOAT_ABS_TOL
        )

    def point_clear(point):
        return boundary_clear(point, point) and (
            segment_min_distance(point, point, obstacles)
            >= req_clearance - FLOAT_ABS_TOL
        )

    def segment_clear(point_a, point_b):
        return boundary_clear(point_a, point_b) and verify_polyline_clearance(
            [point_a, point_b], obstacles, radius, CLEARANCE_MARGIN_METERS
        )[0]

    if not point_clear(start) or not point_clear(goal):
        return None

    start_cell, goal_cell, free_cells = _cells(
        grid, cell_size, start, goal
    )
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None

    queue = deque([start_cell])
    previous = {start_cell: None}
    directions = (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    )
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break
        for di, dj in directions:
            neighbor = (current[0] + di, current[1] + dj)
            if neighbor not in free_cells or neighbor in previous:
                continue
            current_center = _cell_center(current, cell_size)
            neighbor_center = _cell_center(neighbor, cell_size)
            if abs(di) == 1 and abs(dj) == 1:
                previous[neighbor] = current
                queue.append(neighbor)
            elif segment_clear(current_center, neighbor_center):
                previous[neighbor] = current
                queue.append(neighbor)

    return _path_from_cells(
        previous, start_cell, goal_cell, start, goal, cell_size, segment_clear
    )


def _path_from_cells(
    previous,
    start_cell,
    goal_cell,
    start,
    goal,
    cell_size,
    segment_clear,
):
    if goal_cell not in previous:
        return None
    cell_path = []
    current = goal_cell
    while current is not None:
        cell_path.append(current)
        current = previous[current]
    cell_path.reverse()
    centers = [_cell_center(cell, cell_size) for cell in cell_path]
    raw_path = [start]
    if start != centers[0]:
        if not segment_clear(start, centers[0]):
            return None
        raw_path.append(centers[0])
    raw_path.extend(centers[1:])
    if goal != centers[-1]:
        if not segment_clear(centers[-1], goal):
            return None
        raw_path.append(goal)

    deduplicated = []
    for point in raw_path:
        if not deduplicated or point != deduplicated[-1]:
            deduplicated.append(point)

    smoothed = [deduplicated[0]]
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


def plan_m2_collision_swept(problem: dict) -> list[tuple[float, float]] | None:
    """Check only waypoint clearance, not swept segment obstacle clearance."""
    (
        grid,
        cell_size,
        radius,
        start,
        goal,
        width,
        height,
        obstacles,
    ) = _problem_values(problem)
    req_clearance = radius + CLEARANCE_MARGIN_METERS

    def boundary_clear(point_a, point_b):
        return (
            min(point_a[0], point_b[0]) >= req_clearance - FLOAT_ABS_TOL
            and max(point_a[0], point_b[0]) <= width - req_clearance + FLOAT_ABS_TOL
            and min(point_a[1], point_b[1]) >= req_clearance - FLOAT_ABS_TOL
            and max(point_a[1], point_b[1]) <= height - req_clearance + FLOAT_ABS_TOL
        )

    def waypoint_clear(point):
        return segment_min_distance(point, point, obstacles) >= (
            req_clearance - FLOAT_ABS_TOL
        )

    def segment_clear(point_a, point_b):
        return boundary_clear(point_a, point_b) and waypoint_clear(point_a) and waypoint_clear(
            point_b
        )

    if not boundary_clear(start, start) or not boundary_clear(goal, goal):
        return None
    if not waypoint_clear(start) or not waypoint_clear(goal):
        return None

    start_cell, goal_cell, free_cells = _cells(
        grid, cell_size, start, goal
    )
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None

    queue = deque([start_cell])
    previous = {start_cell: None}
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break
        for di, dj in directions:
            neighbor = (current[0] + di, current[1] + dj)
            if neighbor not in free_cells or neighbor in previous:
                continue
            if segment_clear(
                _cell_center(current, cell_size),
                _cell_center(neighbor, cell_size),
            ):
                previous[neighbor] = current
                queue.append(neighbor)

    return _path_from_cells(
        previous, start_cell, goal_cell, start, goal, cell_size, segment_clear
    )


def plan_m3_out_of_bounds(problem: dict) -> list[tuple[float, float]] | None:
    """Remove boundary clearance checks; frozen scenarios may not kill this mutant."""
    (
        grid,
        cell_size,
        radius,
        start,
        goal,
        _width,
        _height,
        obstacles,
    ) = _problem_values(problem)
    req_clearance = radius + CLEARANCE_MARGIN_METERS

    def segment_clear(point_a, point_b):
        return verify_polyline_clearance(
            [point_a, point_b], obstacles, radius, CLEARANCE_MARGIN_METERS
        )[0]

    if (
        segment_min_distance(start, start, obstacles) < req_clearance - FLOAT_ABS_TOL
        or segment_min_distance(goal, goal, obstacles)
        < req_clearance - FLOAT_ABS_TOL
    ):
        return None

    start_cell, goal_cell, free_cells = _cells(
        grid, cell_size, start, goal
    )
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None

    queue = deque([start_cell])
    previous = {start_cell: None}
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break
        for di, dj in directions:
            neighbor = (current[0] + di, current[1] + dj)
            if neighbor not in free_cells or neighbor in previous:
                continue
            if segment_clear(
                _cell_center(current, cell_size),
                _cell_center(neighbor, cell_size),
            ):
                previous[neighbor] = current
                queue.append(neighbor)

    return _path_from_cells(
        previous, start_cell, goal_cell, start, goal, cell_size, segment_clear
    )


def plan_m4_false_infeasible(problem: dict) -> list[tuple[float, float]] | None:
    """Use an inflated clearance margin and reject valid narrow corridors."""
    (
        grid,
        cell_size,
        radius,
        start,
        goal,
        width,
        height,
        obstacles,
    ) = _problem_values(problem)
    margin = CLEARANCE_MARGIN_METERS + 0.05
    req_clearance = radius + margin

    def boundary_clear(point_a, point_b):
        return (
            min(point_a[0], point_b[0]) >= req_clearance - FLOAT_ABS_TOL
            and max(point_a[0], point_b[0]) <= width - req_clearance + FLOAT_ABS_TOL
            and min(point_a[1], point_b[1]) >= req_clearance - FLOAT_ABS_TOL
            and max(point_a[1], point_b[1]) <= height - req_clearance + FLOAT_ABS_TOL
        )

    def segment_clear(point_a, point_b):
        return boundary_clear(point_a, point_b) and verify_polyline_clearance(
            [point_a, point_b], obstacles, radius, margin
        )[0]

    if not boundary_clear(start, start) or not boundary_clear(goal, goal):
        return None
    if (
        segment_min_distance(start, start, obstacles) < req_clearance - FLOAT_ABS_TOL
        or segment_min_distance(goal, goal, obstacles)
        < req_clearance - FLOAT_ABS_TOL
    ):
        return None

    start_cell, goal_cell, free_cells = _cells(
        grid, cell_size, start, goal
    )
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None

    queue = deque([start_cell])
    previous = {start_cell: None}
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break
        for di, dj in directions:
            neighbor = (current[0] + di, current[1] + dj)
            if neighbor not in free_cells or neighbor in previous:
                continue
            if segment_clear(
                _cell_center(current, cell_size),
                _cell_center(neighbor, cell_size),
            ):
                previous[neighbor] = current
                queue.append(neighbor)

    return _path_from_cells(
        previous, start_cell, goal_cell, start, goal, cell_size, segment_clear
    )


def plan_m5_false_feasible(problem: dict) -> list[tuple[float, float]] | None:
    """Return a direct path whenever the endpoint cells are free."""
    grid = problem["grid"]
    cell_size = problem["cell_size"]
    start = (float(problem["start"][0]), float(problem["start"][1]))
    goal = (float(problem["goal"][0]), float(problem["goal"][1]))
    start_cell, goal_cell, free_cells = _cells(
        grid, cell_size, start, goal
    )
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None
    return [start, goal]


MUTANTS = {
    "M1_COLLISION_CORNER": plan_m1_collision_corner,
    "M2_COLLISION_SWEPT": plan_m2_collision_swept,
    "M3_OUT_OF_BOUNDS": plan_m3_out_of_bounds,
    "M4_FALSE_INFEASIBLE": plan_m4_false_infeasible,
    "M5_FALSE_FEASIBLE": plan_m5_false_feasible,
}
