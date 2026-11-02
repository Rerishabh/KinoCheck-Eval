# KC-04: Curvature-Aware Speed Profiling

Implement `plan(problem)` to produce a time-parameterized speed profile for
the supplied spatial polyline.

## Input

Each problem contains:

- `path`: an ordered polyline of at least two `[x, y]` points.
- `v_max`: maximum allowed speed.
- `a_accel_max`: maximum positive longitudinal acceleration.
- `a_brake_max`: maximum allowed braking magnitude.
- `a_lat_max`: maximum lateral acceleration.
- `v_start` and `v_goal`: required speeds at the first and last points.
- `feasible`: whether a valid profile is expected.
- `optimal_time`: reference total time for regret scoring.

The path is piecewise linear. At each waypoint, use the discrete Menger
curvature of the neighboring points. The lateral acceleration constraint is

`v[i] ** 2 * kappa[i] <= a_lat_max`.

For every interval between consecutive waypoints, the longitudinal
acceleration must satisfy

`-a_brake_max <= (v[i+1] ** 2 - v[i] ** 2) / (2 * ds) <= a_accel_max`,

where `ds` is the interval distance. Zero-distance intervals use the
timestamp-based velocity change.

## Output

For a feasible scenario, return one tuple per input point in the form
`(x, y, v, t)`, where `x` and `y` match the path, `v` is the point speed,
and `t` is the timestamp. Timestamps start at zero and increase strictly.
Speeds must respect `v_max`, the lateral acceleration bound, the longitudinal
acceleration and braking bounds, and the required boundary speeds.

Integrate timestamps over each interval using the endpoint speeds and
distance. The total time is the final timestamp. Minimize total time relative
to `optimal_time`; candidate time regret is measured as

`(total_time - optimal_time) / max(optimal_time, FLOAT_ABS_TOL)`.

Return `None` when `feasible` is false. A feasible scenario must return a
valid trajectory rather than `None`.
