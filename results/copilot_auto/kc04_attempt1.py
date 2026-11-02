from __future__ import annotations

import math


def plan(problem: dict) -> list[tuple[float, float, float, float]] | None:
    """Build the fastest speed profile satisfying the supplied constraints."""
    try:
        raw_path = problem["path"]
        v_max = float(problem["v_max"])
        a_accel = float(problem["a_accel_max"])
        a_brake = float(problem["a_brake_max"])
        a_lat = float(problem["a_lat_max"])
        v_start = float(problem["v_start"])
        v_goal = float(problem["v_goal"])
    except (KeyError, TypeError, ValueError):
        return None

    if not isinstance(raw_path, list) or len(raw_path) < 2:
        return None
    if any(
        not isinstance(point, (list, tuple)) or len(point) < 2
        for point in raw_path
    ):
        return None
    try:
        points = [(float(point[0]), float(point[1])) for point in raw_path]
    except (TypeError, ValueError, IndexError):
        return None
    if any(not math.isfinite(value) for point in points for value in point):
        return None
    if any(
        not math.isfinite(value) or value < 0.0
        for value in (v_max, a_accel, a_brake, a_lat, v_start, v_goal)
    ):
        return None

    count = len(points)
    distances = [
        math.hypot(
            points[i + 1][0] - points[i][0],
            points[i + 1][1] - points[i][1],
        )
        for i in range(count - 1)
    ]

    curvatures = [0.0] * count
    for i in range(1, count - 1):
        ax, ay = points[i - 1]
        bx, by = points[i]
        cx, cy = points[i + 1]
        twice_area = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        denominator = (
            distances[i - 1]
            * distances[i]
            * math.hypot(cx - ax, cy - ay)
        )
        if denominator > 0.0:
            curvatures[i] = 2.0 * twice_area / denominator

    caps: list[float] = []
    for curvature in curvatures:
        if curvature > 0.0:
            if a_lat == 0.0:
                cap = 0.0
            else:
                cap = math.sqrt(a_lat / curvature)
            caps.append(min(v_max, cap))
        else:
            caps.append(v_max)

    tolerance = 1e-9
    if v_start > caps[0] + tolerance or v_goal > caps[-1] + tolerance:
        return None

    speeds = caps[:]
    speeds[0] = v_start
    speeds[-1] = v_goal

    # Alternating passes are needed because lowering a speed for a corner can
    # make a later acceleration or an earlier braking constraint tighter.
    for _ in range(2 * count + 2):
        changed = False

        for i, distance in enumerate(distances):
            if distance == 0.0:
                if abs(speeds[i + 1] - speeds[i]) > tolerance:
                    if i + 1 == count - 1:
                        return None
                    new_speed = min(speeds[i], caps[i + 1])
                    if new_speed < speeds[i + 1] - tolerance:
                        speeds[i + 1] = new_speed
                        changed = True
                continue
            reachable = math.sqrt(speeds[i] * speeds[i] + 2.0 * a_accel * distance)
            if i + 1 == count - 1:
                if speeds[i + 1] > reachable + tolerance:
                    return None
            elif speeds[i + 1] > reachable:
                speeds[i + 1] = reachable
                changed = True

        for i in range(count - 2, -1, -1):
            distance = distances[i]
            if distance == 0.0:
                if abs(speeds[i + 1] - speeds[i]) > tolerance:
                    if i == 0:
                        return None
                    new_speed = min(speeds[i + 1], caps[i])
                    if new_speed < speeds[i] - tolerance:
                        speeds[i] = new_speed
                        changed = True
                continue
            reachable = math.sqrt(speeds[i + 1] * speeds[i + 1] + 2.0 * a_brake * distance)
            if i == 0:
                if speeds[i] > reachable + tolerance:
                    return None
            elif speeds[i] > reachable:
                speeds[i] = reachable
                changed = True

        if not changed:
            break
    else:
        return None

    if (
        abs(speeds[0] - v_start) > tolerance
        or abs(speeds[-1] - v_goal) > tolerance
    ):
        return None
    for i, distance in enumerate(distances):
        if distance == 0.0:
            if abs(speeds[i + 1] - speeds[i]) > tolerance:
                return None
            continue
        acceleration = (speeds[i + 1] ** 2 - speeds[i] ** 2) / (2.0 * distance)
        if acceleration < -a_brake - tolerance or acceleration > a_accel + tolerance:
            return None

    timestamps = [0.0]
    for i, distance in enumerate(distances):
        speed_sum = speeds[i] + speeds[i + 1]
        if distance == 0.0:
            duration = 1e-9
        elif speed_sum > tolerance:
            duration = 2.0 * distance / speed_sum
        elif a_accel > 0.0:
            duration = math.sqrt(2.0 * distance / a_accel)
        else:
            return None
        timestamps.append(timestamps[-1] + duration)

    return [
        (points[i][0], points[i][1], speeds[i], timestamps[i])
        for i in range(count)
    ]
