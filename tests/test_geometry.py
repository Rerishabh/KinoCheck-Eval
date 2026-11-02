import math

import pytest

from kinocheck.geometry import (
    compute_polyline_risk,
    create_obstacle_boxes,
    segment_min_distance,
    verify_polyline_clearance,
)


def test_create_obstacle_boxes_uses_x_outer_and_y_inner_indices():
    obstacles = create_obstacle_boxes([[0, 1], [1, 0]], 1.0)

    assert len(obstacles) == 2
    assert sorted(box.bounds for box in obstacles) == [
        (0.0, 1.0, 1.0, 2.0),
        (1.0, 0.0, 2.0, 1.0),
    ]


@pytest.mark.parametrize("cell_size", [0, -1, math.nan, math.inf, -math.inf])
def test_create_obstacle_boxes_rejects_invalid_cell_size(cell_size):
    with pytest.raises(ValueError):
        create_obstacle_boxes([[0]], cell_size)


@pytest.mark.parametrize("grid", [[], [[]], [[0], [0, 1]], [[0, 2]]])
def test_create_obstacle_boxes_rejects_invalid_grids(grid):
    with pytest.raises(ValueError):
        create_obstacle_boxes(grid, 1.0)


@pytest.mark.parametrize("radius", [0, -0.1, math.nan, math.inf, -math.inf])
def test_verify_polyline_clearance_rejects_invalid_radius(radius):
    with pytest.raises(ValueError):
        verify_polyline_clearance([(0, 0)], [], radius, 0.0)


@pytest.mark.parametrize("margin", [-0.1, math.nan, math.inf, -math.inf])
def test_verify_polyline_clearance_rejects_invalid_margin(margin):
    with pytest.raises(ValueError):
        verify_polyline_clearance([(0, 0)], [], 0.1, margin)


@pytest.mark.parametrize(
    "waypoints",
    [
        [(math.nan, 0), (1, 1)],
        [(0, math.inf), (1, 1)],
        [(0, 0), (math.nan, 1)],
        [(0, 0), (1, -math.inf)],
    ],
)
def test_verify_polyline_clearance_rejects_non_finite_waypoints(waypoints):
    with pytest.raises(ValueError):
        verify_polyline_clearance(waypoints, [], 0.1, 0.0)


@pytest.mark.parametrize("waypoints", [[], [(0.0, 0.0)]])
def test_verify_polyline_clearance_rejects_paths_with_fewer_than_two_waypoints(
    waypoints,
):
    assert verify_polyline_clearance(waypoints, [], 0.45, 0.01) == (
        False,
        "PATH_TOO_SHORT: Must have at least 2 waypoints.",
    )


def test_diagonal_corner_touching_obstacles_fails_clearance():
    obstacles = create_obstacle_boxes([[0, 1], [1, 0]], 1.0)

    assert segment_min_distance((0.5, 0.5), (1.5, 1.5), obstacles) == pytest.approx(0.0)
    assert verify_polyline_clearance(
        [(0.5, 0.5), (1.5, 1.5)], obstacles, 0.45, 0.01
    ) == (False, "INSUFFICIENT_CLEARANCE: 0m < 0.46m")


def test_polyline_routing_around_obstacle_is_clear():
    obstacles = create_obstacle_boxes([[0, 0, 0], [0, 1, 0], [0, 0, 0]], 1.0)

    result = verify_polyline_clearance(
        [(0.5, 0.5), (0.5, 2.5), (2.5, 2.5)], obstacles, 0.45, 0.01
    )

    assert result == (True, "CLEAR")


@pytest.mark.parametrize(
    ("x", "expected"),
    [(0.54, True), (0.55, False)],
)
def test_boundary_grazing_uses_inclusive_clearance_threshold(x, expected):
    obstacles = create_obstacle_boxes([[0, 0], [1, 0]], 1.0)

    result = verify_polyline_clearance(
        [(x, -1.0), (x, 2.0)], obstacles, 0.45, 0.01
    )

    assert result[0] is expected
    assert result[1] == "CLEAR" if expected else "INSUFFICIENT_CLEARANCE: 0.45m < 0.46m"


def test_stationary_point_distance_is_supported():
    obstacles = create_obstacle_boxes([[1]], 1.0)

    assert segment_min_distance((2.0, 0.5), (2.0, 0.5), obstacles) == pytest.approx(1.0)
    assert segment_min_distance((0.5, 0.5), (0.5, 0.5), obstacles) == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("path", "expected"),
    [
        ([(0.25, 0.5), (0.75, 0.5)], 0.5),
        ([(0.5, 0.25), (0.5, 0.75)], 0.5),
    ],
)
def test_compute_polyline_risk_within_one_cell(path, expected):
    assert compute_polyline_risk(path, [[1.0]], 1.0) == pytest.approx(expected)


def test_compute_polyline_risk_diagonal_crossing_multiple_cells():
    risk_grid = [[1.0, 2.0], [3.0, 4.0]]

    assert compute_polyline_risk(
        [(0.25, 0.25), (1.75, 1.75)], risk_grid, 1.0
    ) == pytest.approx(15 * math.sqrt(2) / 4)


@pytest.mark.parametrize(
    "path",
    [
        [(1.0, 0.25), (1.0, 1.75)],
        [(0.25, 1.0), (1.75, 1.0)],
    ],
)
def test_compute_polyline_risk_collinear_grid_lines_use_two_cell_average(path):
    assert compute_polyline_risk(path, [[1.0, 2.0], [3.0, 4.0]], 1.0) == pytest.approx(
        3.75
    )


def test_compute_polyline_risk_collinear_vertical_boundary_varies_by_y_cell():
    assert compute_polyline_risk(
        [(0.0, 0.0), (0.0, 3.0)], [[1.0, 2.0, 3.0]], 1.0
    ) == pytest.approx(6.0)


def test_compute_polyline_risk_interior_vertical_boundary_varies_by_cell():
    risk_grid = [[1.0, 2.0, 3.0], [3.0, 4.0, 5.0]]

    assert compute_polyline_risk(
        [(1.0, 0.0), (1.0, 3.0)], risk_grid, 1.0
    ) == pytest.approx(9.0)


def test_compute_polyline_risk_collinear_horizontal_boundary_varies_by_x_cell():
    assert compute_polyline_risk(
        [(0.0, 0.0), (3.0, 0.0)], [[1.0], [2.0], [3.0]], 1.0
    ) == pytest.approx(6.0)


def test_compute_polyline_risk_interior_horizontal_boundary_varies_by_cell():
    risk_grid = [[1.0, 3.0], [2.0, 4.0], [3.0, 5.0]]

    assert compute_polyline_risk(
        [(0.0, 1.0), (3.0, 1.0)], risk_grid, 1.0
    ) == pytest.approx(9.0)


def test_compute_polyline_risk_segment_starting_on_interior_grid_line():
    assert compute_polyline_risk(
        [(1.0, 0.5), (1.5, 0.5)], [[1.0], [3.0]], 1.0
    ) == pytest.approx(1.5)


def test_compute_polyline_risk_outer_perimeter_path():
    assert compute_polyline_risk(
        [(0.0, 0.0), (0.0, 2.0)], [[1.0, 2.0], [3.0, 4.0]], 1.0
    ) == pytest.approx(3.0)


def test_compute_polyline_risk_short_segment_crosses_grid_line():
    assert compute_polyline_risk(
        [(1.0 - 5e-7, 0.5), (1.0 + 5e-7, 0.5)], [[1.0], [3.0]], 1.0
    ) == pytest.approx(2e-6)


def test_compute_polyline_risk_near_endpoint_crossing():
    risk_grid = [[1.0], [3.0]]
    path = [(1.0 - 5e-10, 0.5), (1.0 + 1.0, 0.5)]
    expected = 5e-10 * 1.0 + (1.0 + 5e-10) * 3.0

    assert compute_polyline_risk(path, risk_grid, 1.0) == pytest.approx(
        expected, abs=1e-9
    )


def test_compute_polyline_risk_through_grid_vertex():
    assert compute_polyline_risk(
        [(0.0, 0.0), (2.0, 2.0)], [[1.0, 2.0], [3.0, 4.0]], 1.0
    ) == pytest.approx(5 * math.sqrt(2))


def test_compute_polyline_risk_ignores_zero_length_segments():
    assert compute_polyline_risk(
        [(0.25, 0.25), (0.25, 0.25), (0.75, 0.25)], [[2.0]], 1.0
    ) == pytest.approx(1.0)


@pytest.mark.parametrize("path", [[], [(0.5, 0.5)]])
def test_compute_polyline_risk_empty_or_single_point_path(path):
    assert compute_polyline_risk(path, [[2.0]], 1.0) == 0.0


def test_compute_polyline_risk_uses_column_major_non_square_grid():
    assert compute_polyline_risk(
        [(0.5, 2.1), (0.5, 2.9)], [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], 1.0
    ) == pytest.approx(2.4)


@pytest.mark.parametrize("cell_size", [0.0, -1.0, math.nan, math.inf])
def test_compute_polyline_risk_rejects_invalid_cell_size(cell_size):
    with pytest.raises(ValueError):
        compute_polyline_risk([], [[1.0]], cell_size)


@pytest.mark.parametrize(
    "risk_grid",
    [
        [],
        [[]],
        [[1.0], [2.0, 3.0]],
        [[-1.0]],
        [[math.nan]],
        [[math.inf]],
    ],
)
def test_compute_polyline_risk_rejects_invalid_grid(risk_grid):
    with pytest.raises(ValueError):
        compute_polyline_risk([], risk_grid, 1.0)


@pytest.mark.parametrize(
    "path",
    [
        [(-0.1, 0.5)],
        [(0.5, 1.1)],
        [(math.nan, 0.5)],
        [(0.5, math.inf)],
    ],
)
def test_compute_polyline_risk_rejects_invalid_path(path):
    with pytest.raises(ValueError):
        compute_polyline_risk(path, [[1.0]], 1.0)
