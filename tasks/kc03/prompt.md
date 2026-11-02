# KC-03: Risk-Budget Pareto Path Planning

Implement `plan(problem)` and return a collision-free polyline from `start` to
`goal`, or `None` when no feasible path exists.

## Input

The problem is a dictionary containing `grid` and `risk_grid` in column-major
order (`grid[x][y]`), `cell_size`, `radius`, `start`, `goal`, `feasible`,
`risk_budget`, and `optimal_length`. Occupied grid cells contain `1`; free cells
contain `0`. The arena is `[0, width] x [0, height]`, where width and height
are the grid dimensions multiplied by `cell_size`.

## Output

Return a sequence of two-dimensional numeric points. The first and last points
must match `start` and `goal` within `1e-9`. Return `None` when the scenario is
infeasible.

## Continuous clearance

The swept disc must remain inside the arena with clearance at least
`radius + 0.01` from every perimeter. Every segment must also remain at least
`radius + 0.01` from every occupied obstacle cell, including obstacle corners.

## Risk budget

Risk is a piecewise-constant field. The line integral is computed exactly by
splitting every segment at crossed interior grid lines. For a path `gamma`,
the path is accepted only when:

`Risk(gamma) <= risk_budget + 1e-9`.

## Objective

Among risk-compliant collision-free paths, minimize Euclidean polyline length.
The normalized regret is:

`(path_length - optimal_length) / max(optimal_length, 1e-9)`.

The reference solution has zero regret within numerical tolerance.
