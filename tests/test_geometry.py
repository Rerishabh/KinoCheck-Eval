import math

import pytest

from kinocheck.geometry import (
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
