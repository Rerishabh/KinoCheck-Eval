# KC-02: Grid planner without corner-cutting

Implement a planner for a disc robot of radius `r` navigating an occupancy grid. The robot must not approach any obstacle closer than `r + 0.01m` on any segment.

## Arena boundary

The outer arena boundary is the solid perimeter wall:

```text
[0, width] x [0, height]
```

where `width = cols * cell_size` and `height = rows * cell_size`. The entire path, including waypoints and the swept robot body, must remain within this arena and must maintain a distance of at least `radius + 0.01m` from every outer perimeter line:

- `x = 0`
- `x = width`
- `y = 0`
- `y = height`

This boundary-clearance rule applies in addition to the clearance requirement for obstacle cells.

Leaving the grid or allowing any path segment to come within radius + 0.01m of an outer perimeter line is forbidden.

## Input dictionary schema

The planner receives a dictionary with:

- `"grid"`: list of lists of integers (`0` or `1`), using Cartesian mapping where `grid[i][j]` spans:
  - `x` in `[i * cell_size, (i + 1) * cell_size]`
  - `y` in `[j * cell_size, (j + 1) * cell_size]`
- `"cell_size"`: float, in meters.
- `"radius"`: float, in meters.
- `"start"`: `[float, float]` in world meters, at the center of the start cell.
- `"goal"`: `[float, float]` in world meters, at the center of the goal cell.

## Output

Return a list of `[x, y]` coordinates starting exactly at `start` and ending at `goal`, or return `None` if the problem is provably impossible.

## Hard constraints

- Every path segment must satisfy both of these clearance requirements:
  - For every obstacle cell, `distance(segment, obstacle) >= radius + 0.01m`.
  - For each outer perimeter line (`x = 0`, `x = width`, `y = 0`, and `y = height`), `distance(segment, perimeter_line) >= radius + 0.01m`.
- The outer-boundary clearance rule is in addition to obstacle-cell clearance; satisfying one does not replace the other.
- Endpoints must match `start` and `goal` within `1e-9`.

## Return format

Return only the Python function in a single `python` code block named:

```python
plan(problem: dict) -> list[tuple[float, float]] | None
```
