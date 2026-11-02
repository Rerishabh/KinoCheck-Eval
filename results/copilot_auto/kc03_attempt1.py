def plan(problem: dict) -> list[list[int]] | None:
    grid = problem["grid"]
    risk_grid = problem["risk_grid"]
    start = problem["start"]
    goal = problem["goal"]
    budget = problem["risk_budget"]

    if not isinstance(grid, list) or not grid or not isinstance(grid[0], list):
        return None
    width = len(grid)
    height = len(grid[0])
    if height == 0 or any(not isinstance(column, list) or len(column) != height for column in grid):
        return None
    if (
        not isinstance(risk_grid, list)
        or len(risk_grid) != width
        or any(not isinstance(column, list) or len(column) != height for column in risk_grid)
    ):
        return None
    if not isinstance(budget, (int, float)) or budget < 0:
        return None
    if any(
        not isinstance(risk_grid[x][y], (int, float)) or risk_grid[x][y] < 0
        for x in range(width)
        for y in range(height)
    ):
        return None

    try:
        if len(start) != 2 or len(goal) != 2:
            return None
        start_point = (int(start[0]), int(start[1]))
        goal_point = (int(goal[0]), int(goal[1]))
    except (TypeError, ValueError, IndexError):
        return None
    if start_point != tuple(start) or goal_point != tuple(goal):
        return None

    def in_bounds(point: tuple[int, int]) -> bool:
        return 0 <= point[0] < width and 0 <= point[1] < height

    if not in_bounds(start_point) or not in_bounds(goal_point):
        return None
    if grid[start_point[0]][start_point[1]] != 0 or grid[goal_point[0]][goal_point[1]] != 0:
        return None
    if start_point == goal_point:
        return [[start_point[0], start_point[1]]]
    if risk_grid[start_point[0]][start_point[1]] > budget:
        return None

    current = {start_point: 0.0}
    parents: dict[tuple[int, tuple[int, int]], tuple[int, tuple[int, int]] | None] = {
        (0, start_point): None
    }
    depth = 0
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))

    while current:
        next_depth = depth + 1
        next_layer: dict[tuple[int, int], float] = {}
        next_parents: dict[tuple[int, int], tuple[int, int]] = {}

        for point, accumulated in current.items():
            for dx, dy in directions:
                neighbor = (point[0] + dx, point[1] + dy)
                if not in_bounds(neighbor) or grid[neighbor[0]][neighbor[1]] != 0:
                    continue
                new_risk = accumulated + float(risk_grid[neighbor[0]][neighbor[1]])
                if new_risk > float(budget) + 1e-9:
                    continue
                if neighbor not in next_layer or new_risk < next_layer[neighbor]:
                    next_layer[neighbor] = new_risk
                    next_parents[neighbor] = point

        for point, parent in next_parents.items():
            parents[(next_depth, point)] = (depth, parent)

        if goal_point in next_layer:
            path = [goal_point]
            point = goal_point
            layer = next_depth
            while layer > 0:
                previous = parents[(layer, point)]
                point = previous[1]
                path.append(point)
                layer -= 1
            path.reverse()
            return [[x, y] for x, y in path]

        current = next_layer
        depth = next_depth

    return None
