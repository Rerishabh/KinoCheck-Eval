def plan(problem: dict) -> list[tuple[float, float]] | None:
    from collections import deque
    import math

    try:
        grid = problem["grid"]
        cell_size = float(problem["cell_size"])
        radius = float(problem["radius"])
        start = (float(problem["start"][0]), float(problem["start"][1]))
        goal = (float(problem["goal"][0]), float(problem["goal"][1]))
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None

    if (
        not isinstance(grid, list)
        or not grid
        or not isinstance(grid[0], list)
        or not grid[0]
        or any(not isinstance(column, list) or len(column) != len(grid[0])
               for column in grid)
        or any(value not in (0, 1) for column in grid for value in column)
        or not math.isfinite(cell_size)
        or not math.isfinite(radius)
        or not all(math.isfinite(value) for value in (*start, *goal))
        or cell_size <= 0.0
        or radius < 0.0
    ):
        return None

    required = radius + 0.01
    tolerance = 1e-9
    width = len(grid) * cell_size
    height = len(grid[0]) * cell_size

    obstacles = [
        (x * cell_size, y * cell_size,
         (x + 1) * cell_size, (y + 1) * cell_size)
        for x, column in enumerate(grid)
        for y, value in enumerate(column)
        if value == 1
    ]

    def point_segment_distance(
        point: tuple[float, float],
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        px, py = point
        ax, ay = first
        bx, by = second
        dx = bx - ax
        dy = by - ay
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            return math.hypot(px - ax, py - ay)
        parameter = ((px - ax) * dx + (py - ay) * dy) / length_squared
        parameter = max(0.0, min(1.0, parameter))
        return math.hypot(
            px - (ax + parameter * dx),
            py - (ay + parameter * dy),
        )

    def orientation(
        first: tuple[float, float],
        second: tuple[float, float],
        third: tuple[float, float],
    ) -> float:
        return (
            (second[0] - first[0]) * (third[1] - first[1])
            - (second[1] - first[1]) * (third[0] - first[0])
        )

    def on_segment(
        first: tuple[float, float],
        second: tuple[float, float],
        point: tuple[float, float],
    ) -> bool:
        return (
            min(first[0], second[0]) - tolerance <= point[0]
            <= max(first[0], second[0]) + tolerance
            and min(first[1], second[1]) - tolerance <= point[1]
            <= max(first[1], second[1]) + tolerance
        )

    def segments_intersect(
        first: tuple[float, float],
        second: tuple[float, float],
        third: tuple[float, float],
        fourth: tuple[float, float],
    ) -> bool:
        first_orientation = orientation(first, second, third)
        second_orientation = orientation(first, second, fourth)
        third_orientation = orientation(third, fourth, first)
        fourth_orientation = orientation(third, fourth, second)
        proper = (
            ((first_orientation > tolerance and second_orientation < -tolerance)
             or (first_orientation < -tolerance and second_orientation > tolerance))
            and
            ((third_orientation > tolerance and fourth_orientation < -tolerance)
             or (third_orientation < -tolerance and fourth_orientation > tolerance))
        )
        return proper or (
            abs(first_orientation) <= tolerance
            and on_segment(first, second, third)
            or abs(second_orientation) <= tolerance
            and on_segment(first, second, fourth)
            or abs(third_orientation) <= tolerance
            and on_segment(third, fourth, first)
            or abs(fourth_orientation) <= tolerance
            and on_segment(third, fourth, second)
        )

    def rectangle_distance(
        first: tuple[float, float],
        second: tuple[float, float],
        rectangle: tuple[float, float, float, float],
    ) -> float:
        left, bottom, right, top = rectangle
        corners = (
            (left, bottom),
            (right, bottom),
            (right, top),
            (left, top),
        )
        edges = tuple(zip(corners, corners[1:] + corners[:1]))
        if (
            (left - tolerance <= first[0] <= right + tolerance
             and bottom - tolerance <= first[1] <= top + tolerance)
            or
            (left - tolerance <= second[0] <= right + tolerance
             and bottom - tolerance <= second[1] <= top + tolerance)
        ):
            return 0.0
        if any(segments_intersect(first, second, edge[0], edge[1])
               for edge in edges):
            return 0.0
        if first == second:
            return min(point_segment_distance(first, edge[0], edge[1])
                       for edge in edges)
        return min(
            min(point_segment_distance(first, edge[0], edge[1])
                for edge in edges),
            min(point_segment_distance(second, edge[0], edge[1])
                for edge in edges),
            min(point_segment_distance(corner, first, second)
                for corner in corners),
        )

    def boundary_clear(
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> bool:
        return (
            min(first[0], second[0]) >= required - tolerance
            and max(first[0], second[0]) <= width - required + tolerance
            and min(first[1], second[1]) >= required - tolerance
            and max(first[1], second[1]) <= height - required + tolerance
        )

    def segment_clear(
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> bool:
        return boundary_clear(first, second) and all(
            rectangle_distance(first, second, obstacle)
            >= required - tolerance
            for obstacle in obstacles
        )

    if not segment_clear(start, start) or not segment_clear(goal, goal):
        return None
    if math.hypot(start[0] - goal[0], start[1] - goal[1]) <= tolerance:
        return [start]

    columns = len(grid)
    rows = len(grid[0])

    def cell_for(point: tuple[float, float]) -> tuple[int, int]:
        return (
            min(columns - 1, max(0, int(point[0] // cell_size))),
            min(rows - 1, max(0, int(point[1] // cell_size))),
        )

    start_cell = cell_for(start)
    goal_cell = cell_for(goal)
    free_cells = {
        (x, y)
        for x, column in enumerate(grid)
        for y, value in enumerate(column)
        if value == 0
    }
    if start_cell not in free_cells or goal_cell not in free_cells:
        return None

    def center(cell: tuple[int, int]) -> tuple[float, float]:
        return ((cell[0] + 0.5) * cell_size, (cell[1] + 0.5) * cell_size)

    previous: dict[tuple[int, int], tuple[int, int] | None] = {
        start_cell: None
    }
    queue: deque[tuple[int, int]] = deque([start_cell])
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in previous or neighbor not in free_cells:
                continue
            if segment_clear(center(current), center(neighbor)):
                previous[neighbor] = current
                queue.append(neighbor)

    if goal_cell not in previous:
        return None

    cells: list[tuple[int, int]] = []
    current: tuple[int, int] | None = goal_cell
    while current is not None:
        cells.append(current)
        current = previous[current]
    cells.reverse()
    centers = [center(cell) for cell in cells]

    raw: list[tuple[float, float]] = [start]
    if start != centers[0]:
        if not segment_clear(start, centers[0]):
            return None
        raw.append(centers[0])
    raw.extend(centers[1:])
    if goal != centers[-1]:
        if not segment_clear(centers[-1], goal):
            return None
        raw.append(goal)

    path: list[tuple[float, float]] = []
    for point in raw:
        if not path or point != path[-1]:
            path.append(point)

    smoothed = [path[0]]
    index = 0
    while index < len(path) - 1:
        next_index = index + 1
        for candidate in range(len(path) - 1, index, -1):
            if segment_clear(path[index], path[candidate]):
                next_index = candidate
                break
        smoothed.append(path[next_index])
        index = next_index
    smoothed[0] = start
    smoothed[-1] = goal
    return smoothed
