def plan(problem: dict) -> list[tuple[float, float]] | None:
    from collections import deque
    import math

    grid = problem["grid"]
    cell_size = float(problem["cell_size"])
    radius = float(problem["radius"])
    start = (float(problem["start"][0]), float(problem["start"][1]))
    goal = (float(problem["goal"][0]), float(problem["goal"][1]))

    if (
        not grid
        or not grid[0]
        or any(len(column) != len(grid[0]) for column in grid)
        or cell_size <= 0
        or radius <= 0
    ):
        return None

    margin = 0.01
    tolerance = 1e-9
    required = radius + margin
    width = len(grid) * cell_size
    height = len(grid[0]) * cell_size

    obstacles = [
        (
            i * cell_size,
            j * cell_size,
            (i + 1) * cell_size,
            (j + 1) * cell_size,
        )
        for i, column in enumerate(grid)
        for j, value in enumerate(column)
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
        if length_squared == 0:
            return math.hypot(px - ax, py - ay)
        parameter = ((px - ax) * dx + (py - ay) * dy) / length_squared
        parameter = max(0.0, min(1.0, parameter))
        return math.hypot(px - (ax + parameter * dx), py - (ay + parameter * dy))

    def orientation(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
    ) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_segment(
        a: tuple[float, float],
        b: tuple[float, float],
        point: tuple[float, float],
    ) -> bool:
        return (
            min(a[0], b[0]) - tolerance <= point[0] <= max(a[0], b[0]) + tolerance
            and min(a[1], b[1]) - tolerance <= point[1] <= max(a[1], b[1]) + tolerance
        )

    def segments_intersect(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
        d: tuple[float, float],
    ) -> bool:
        o1 = orientation(a, b, c)
        o2 = orientation(a, b, d)
        o3 = orientation(c, d, a)
        o4 = orientation(c, d, b)

        if (
            ((o1 > tolerance and o2 < -tolerance) or (o1 < -tolerance and o2 > tolerance))
            and ((o3 > tolerance and o4 < -tolerance) or (o3 < -tolerance and o4 > tolerance))
        ):
            return True

        return (
            abs(o1) <= tolerance and on_segment(a, b, c)
            or abs(o2) <= tolerance and on_segment(a, b, d)
            or abs(o3) <= tolerance and on_segment(c, d, a)
            or abs(o4) <= tolerance and on_segment(c, d, b)
        )

    def segment_rectangle_distance(
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
            left - tolerance <= first[0] <= right + tolerance
            and bottom - tolerance <= first[1] <= top + tolerance
            or left - tolerance <= second[0] <= right + tolerance
            and bottom - tolerance <= second[1] <= top + tolerance
        ):
            return 0.0

        if any(segments_intersect(first, second, edge[0], edge[1]) for edge in edges):
            return 0.0

        return min(
            point_segment_distance(first, edge[0], edge[1])
            for edge in edges
        ) if first == second else min(
            point_segment_distance(first, edge[0], edge[1]),
            point_segment_distance(second, edge[0], edge[1]),
            point_segment_distance(edge[0], first, second),
            point_segment_distance(edge[1], first, second),
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
        if not boundary_clear(first, second):
            return False
        return all(
            segment_rectangle_distance(first, second, obstacle)
            >= required - tolerance
            for obstacle in obstacles
        )

    if not segment_clear(start, start) or not segment_clear(goal, goal):
        return None

    if (
        abs(start[0] - goal[0]) <= tolerance
        and abs(start[1] - goal[1]) <= tolerance
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

    def center(cell: tuple[int, int]) -> tuple[float, float]:
        i, j = cell
        return ((i + 0.5) * cell_size, (j + 0.5) * cell_size)

    queue = deque([start_cell])
    previous: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}

    while queue:
        current = queue.popleft()
        if current == goal_cell:
            break

        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (current[0] + di, current[1] + dj)
            if neighbor not in free_cells or neighbor in previous:
                continue
            if segment_clear(center(current), center(neighbor)):
                previous[neighbor] = current
                queue.append(neighbor)

    if goal_cell not in previous:
        return None

    cells = []
    current = goal_cell
    while current is not None:
        cells.append(current)
        current = previous[current]
    cells.reverse()

    centers = [center(cell) for cell in cells]
    path = [start]
    if start != centers[0]:
        if not segment_clear(start, centers[0]):
            return None
        path.append(centers[0])

    path.extend(centers[1:])

    if goal != centers[-1]:
        if not segment_clear(centers[-1], goal):
            return None
        path.append(goal)

    deduplicated = []
    for point in path:
        if not deduplicated or point != deduplicated[-1]:
            deduplicated.append(point)

    smoothed = [deduplicated[0]]
    index = 0
    while index < len(deduplicated) - 1:
        next_index = index + 1
        for candidate in range(len(deduplicated) - 1, index, -1):
            if segment_clear(deduplicated[index], deduplicated[candidate]):
                next_index = candidate
                break
        smoothed.append(deduplicated[next_index])
        index = next_index

    smoothed[0] = start
    smoothed[-1] = goal
    return smoothed
