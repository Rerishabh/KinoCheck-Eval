import heapq
import math


def _point_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denominator
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _segments_intersect(a, b, c, d):
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p, q, r):
        return (
            min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
            and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
        )

    eps = 1e-12
    ab_c = cross(a, b, c)
    ab_d = cross(a, b, d)
    cd_a = cross(c, d, a)
    cd_b = cross(c, d, b)
    if (
        ((ab_c > eps and ab_d < -eps) or (ab_c < -eps and ab_d > eps))
        and ((cd_a > eps and cd_b < -eps) or (cd_a < -eps and cd_b > eps))
    ):
        return True
    return (
        abs(ab_c) <= eps and on_segment(a, c, b)
        or abs(ab_d) <= eps and on_segment(a, d, b)
        or abs(cd_a) <= eps and on_segment(c, a, d)
        or abs(cd_b) <= eps and on_segment(c, b, d)
    )


def _segment_rectangle_distance(a, b, left, bottom, right, top):
    # A segment crossing the rectangle has zero distance to it.
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for origin, direction, low, high in (
        (a[0], dx, left, right),
        (a[1], dy, bottom, top),
    ):
        if abs(direction) <= 1e-15:
            if origin < low or origin > high:
                break
        else:
            near = (low - origin) / direction
            far = (high - origin) / direction
            if near > far:
                near, far = far, near
            t0 = max(t0, near)
            t1 = min(t1, far)
            if t0 > t1:
                break
    else:
        return 0.0

    corners = (
        (left, bottom),
        (right, bottom),
        (right, top),
        (left, top),
    )
    edges = zip(corners, corners[1:] + corners[:1])
    return min(
        [_point_segment_distance(c[0], c[1], a[0], a[1], b[0], b[1]) for c in corners]
        + [
            min(
                _point_segment_distance(a[0], a[1], c[0], c[1], d[0], d[1]),
                _point_segment_distance(b[0], b[1], c[0], c[1], d[0], d[1]),
            )
            for c, d in edges
        ]
    )


def _risk_segment(a, b, risk_grid, size):
    width = len(risk_grid)
    height = len(risk_grid[0])
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        return 0.0
    parameters = [0.0, 1.0]
    if dx != 0.0:
        first = max(1, math.ceil(min(a[0], b[0]) / size))
        last = min(width - 1, math.floor(max(a[0], b[0]) / size))
        for i in range(first, last + 1):
            t = (i * size - a[0]) / dx
            if 0.0 < t < 1.0:
                parameters.append(t)
    if dy != 0.0:
        first = max(1, math.ceil(min(a[1], b[1]) / size))
        last = min(height - 1, math.floor(max(a[1], b[1]) / size))
        for i in range(first, last + 1):
            t = (i * size - a[1]) / dy
            if 0.0 < t < 1.0:
                parameters.append(t)
    parameters.sort()
    unique = []
    for t in parameters:
        if not unique or t - unique[-1] > 1e-9:
            unique.append(t)
    total = 0.0
    for lo, hi in zip(unique, unique[1:]):
        if hi <= lo:
            continue
        mx = a[0] + (lo + hi) * dx / 2.0
        my = a[1] + (lo + hi) * dy / 2.0
        if abs(dx) <= 1e-9 and any(abs(a[0] - i * size) <= 1e-9 for i in range(1, width)):
            y = min(height - 1, max(0, math.floor(my / size)))
            line = round(a[0] / size)
            value = (risk_grid[line - 1][y] + risk_grid[line][y]) / 2.0
        elif abs(dy) <= 1e-9 and any(abs(a[1] - i * size) <= 1e-9 for i in range(1, height)):
            x = min(width - 1, max(0, math.floor(mx / size)))
            line = round(a[1] / size)
            value = (risk_grid[x][line - 1] + risk_grid[x][line]) / 2.0
        else:
            x = min(width - 1, max(0, math.floor(mx / size)))
            y = min(height - 1, max(0, math.floor(my / size)))
            value = risk_grid[x][y]
        total += (hi - lo) * length * value
    return total


def _path_risk(path, risk_grid, size):
    return sum(_risk_segment(a, b, risk_grid, size) for a, b in zip(path, path[1:]))


def plan(problem: dict) -> list[list[float]] | list[tuple[float, float]] | None:
    try:
        grid = problem["grid"]
        risk_grid = problem["risk_grid"]
        size = float(problem["cell_size"])
        radius = float(problem["radius"])
        budget = float(problem["risk_budget"])
        start = (float(problem["start"][0]), float(problem["start"][1]))
        goal = (float(problem["goal"][0]), float(problem["goal"][1]))
        if (
            not grid
            or not grid[0]
            or len(risk_grid) != len(grid)
            or any(len(column) != len(grid[0]) for column in grid)
            or any(len(column) != len(grid[0]) for column in risk_grid)
            or size <= 0.0
            or radius <= 0.0
            or budget < 0.0
        ):
            return None
        if not all(math.isfinite(v) for v in (*start, *goal, size, radius, budget)):
            return None
    except (KeyError, TypeError, ValueError, IndexError):
        return None

    width, height = len(grid) * size, len(grid[0]) * size
    clearance = radius + 0.01 - 1e-9
    obstacles = [
        (x * size, y * size, (x + 1) * size, (y + 1) * size)
        for x, column in enumerate(grid)
        for y, value in enumerate(column)
        if value == 1
    ]

    def clear(a, b):
        if (
            min(a[0], b[0]) < clearance
            or max(a[0], b[0]) > width - clearance
            or min(a[1], b[1]) < clearance
            or max(a[1], b[1]) > height - clearance
        ):
            return False
        return all(
            _segment_rectangle_distance(a, b, left, bottom, right, top)
            >= clearance
            for left, bottom, right, top in obstacles
        )

    nodes = []
    for x, column in enumerate(grid):
        for y, value in enumerate(column):
            point = ((x + 0.5) * size, (y + 0.5) * size)
            if value == 0 and clear(point, point):
                nodes.append(point)
    nodes.extend((start, goal))
    start_id, goal_id = len(nodes) - 2, len(nodes) - 1

    edges = [[] for _ in nodes]
    lattice = {
        (int(math.floor(point[0] / size)), int(math.floor(point[1] / size))): i
        for i, point in enumerate(nodes[:-2])
    }
    candidates = set()
    for (x, y), source in lattice.items():
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            target = lattice.get((x + dx, y + dy))
            if target is not None:
                candidates.add((source, target))
    for i in range(len(nodes)):
        if i != start_id:
            candidates.add((start_id, i))
        if i != goal_id:
            candidates.add((i, goal_id))
    candidates.add((start_id, goal_id))

    for source, target in candidates:
        if source == target:
            continue
        a, b = nodes[source], nodes[target]
        if not clear(a, b):
            continue
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        risk = _risk_segment(a, b, risk_grid, size)
        edges[source].append((target, length, risk))

    labels = [[] for _ in nodes]
    initial = (0.0, 0.0, start_id, (start_id,), (start,))
    labels[start_id].append(initial)
    queue = [(0.0, 0.0, 0, initial)]
    sequence = 1
    while queue:
        _, _, _, label = heapq.heappop(queue)
        length, risk, node, visited, path = label
        if label not in labels[node] or node == goal_id:
            continue
        for target, edge_length, edge_risk in edges[node]:
            if target in visited:
                continue
            new_risk = risk + edge_risk
            if new_risk > budget + 1e-9:
                continue
            new_label = (
                length + edge_length,
                new_risk,
                target,
                visited + (target,),
                path + (nodes[target],),
            )
            if any(
                old[0] <= new_label[0]
                and old[1] <= new_label[1]
                and (old[0] < new_label[0] or old[1] < new_label[1])
                for old in labels[target]
            ):
                continue
            labels[target] = [
                old for old in labels[target]
                if not (
                    new_label[0] <= old[0]
                    and new_label[1] <= old[1]
                    and (new_label[0] < old[0] or new_label[1] < old[1])
                )
            ]
            labels[target].append(new_label)
            heapq.heappush(queue, (new_label[0], new_label[1], sequence, new_label))
            sequence += 1

    choices = sorted(labels[goal_id], key=lambda label: (label[0], label[1], label[4]))
    for label in choices:
        if _path_risk(label[4], risk_grid, size) <= budget + 1e-9:
            return list(label[4])
    return None
