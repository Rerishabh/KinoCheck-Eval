import math

import pytest

from kinocheck.geometry import compute_menger_curvature


def test_straight_horizontal_points_have_zero_curvature():
    assert compute_menger_curvature([(0, 0), (1, 0), (2, 0), (3, 0)]) == [
        0.0,
        0.0,
        0.0,
        0.0,
    ]


def test_straight_diagonal_points_have_zero_curvature():
    assert compute_menger_curvature([(0, 0), (1, 1), (2, 2)]) == [0.0, 0.0, 0.0]


def test_unit_circle_points_have_unit_curvature():
    values = compute_menger_curvature([(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)])
    assert values == pytest.approx([1.0, 1.0, 1.0])


def test_right_angle_has_expected_menger_curvature():
    values = compute_menger_curvature([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert values == pytest.approx([math.sqrt(2.0)] * 3)


@pytest.mark.parametrize(
    "points",
    [[], [(0, 0)], [(0, 0), (1, 1)]],
)
def test_degenerate_point_counts_return_matching_zero_lists(points):
    assert compute_menger_curvature(points) == [0.0] * len(points)


def test_duplicate_consecutive_points_are_safe():
    values = compute_menger_curvature([(0, 0), (1, 1), (1, 1), (2, 2)])
    assert values == [0.0] * 4
    assert all(math.isfinite(value) for value in values)


def test_small_finite_coordinates_are_deterministic():
    points = [(0.0, 0.0), (1e-5, 0.0), (2e-5, 0.0)]
    first = compute_menger_curvature(points)
    assert first == compute_menger_curvature(points)
    assert all(math.isfinite(value) for value in first)


@pytest.mark.parametrize("points", ["points", {"x": 1}, [(0, 0, 0)], [0, 1]])
def test_invalid_point_structures_raise_value_error(points):
    with pytest.raises(ValueError):
        compute_menger_curvature(points)


def test_non_finite_coordinates_raise_value_error():
    with pytest.raises(ValueError):
        compute_menger_curvature([(0.0, 0.0), (float("nan"), 1.0), (2.0, 2.0)])


def test_input_points_are_not_mutated():
    points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]
    original = [point[:] for point in points]
    compute_menger_curvature(points)
    assert points == original
