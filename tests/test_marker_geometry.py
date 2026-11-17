import math

import pytest

from tb4_mission.marker_geometry import (intersect_support_surface, marker_center,
                                        rotate_vector, scan_points, transform_point)


def wall(x=3.0, start=-0.6, stop=0.6, step=0.05):
    return [(x, start+i*step, 0.2) for i in range(round((stop-start)/step)+1)]


def test_known_wall_unknown_marker_size():
    estimate = intersect_support_surface((0.0, 0.0, 0.7), (1.0, 0.0, 0.05), wall())
    assert estimate is not None
    assert (estimate.x, estimate.y, estimate.z) == pytest.approx((3.0, 0.0, 0.85))
    assert estimate.support >= 5


def test_camera_translation_is_not_ignored():
    # Camera is 20 cm forward and 30 cm left of laser, looking along fixed X.
    estimate = intersect_support_surface((0.2, 0.3, 0.8), (1.0, 0.0, 0.0), wall())
    assert (estimate.x, estimate.y, estimate.distance) == pytest.approx((3.0, 0.3, 2.8))


def test_rotated_optical_camera_and_laser():
    # Optical +Z rotated +90 deg about Y points along odom +X.
    q_camera = (0.0, math.sin(math.pi/4), 0.0, math.cos(math.pi/4))
    ray = rotate_vector((0.0, 0.0, 1.0), q_camera)
    assert ray == pytest.approx((1.0, 0.0, 0.0))
    q_scan = (0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))
    transformed = [transform_point((p[1], -p[0], p[2]), (0, 0, 0), q_scan) for p in wall()]
    estimate = intersect_support_surface((0, 0, .7), ray, transformed)
    assert (estimate.x, estimate.y) == pytest.approx((3.0, 0.0))


def test_invalid_scan_ranges_remain_gaps():
    points = scan_points([1.0, math.inf, math.nan, 0.01, 9.0], 0.0, 0.1, .1, 6.0)
    assert points[0] == (1.0, 0.0, 0.0)
    assert points[1:] == [None]*4


def test_no_interpolation_across_missing_beams():
    points = wall()
    points[11:14] = [None]*3
    assert intersect_support_surface((0, 0, .7), (1, 0, 0), points) is None


def test_no_interpolation_across_depth_jump():
    points = [(2, -.3, .2), (2, -.2, .2), (2, -.1, .2),
              (4, .1, .2), (4, .2, .2), (4, .3, .2)]
    assert intersect_support_surface((0, 0, .7), (1, 0, 0), points) is None


def test_too_few_returns_rejected():
    assert intersect_support_surface((0, 0, .7), (1, 0, 0), wall(start=-.05, stop=.05)) is None


def test_surface_noise_and_corner_rejected():
    points = [(3+(.12 if i % 2 else -.12), -.4+i*.05, .2) for i in range(17)]
    assert intersect_support_surface((0, 0, .7), (1, 0, 0), points, max_gap=.4) is None


def test_nearest_valid_surface_wins():
    estimate = intersect_support_surface((0, 0, .7), (1, 0, 0), wall(2)+[None]+wall(4))
    assert estimate.x == pytest.approx(2)


def test_near_sparse_occluder_does_not_fall_through_to_far_wall():
    sparse = wall(2, -.05, .05)
    assert intersect_support_surface((0, 0, .7), (1, 0, 0), sparse+[None]+wall(4)) is None


def test_away_grazing_vertical_and_impossible_height_rays_rejected():
    assert intersect_support_surface((0, 0, .7), (-1, 0, 0), wall()) is None
    assert intersect_support_surface((0, 0, .7), (0, 0, 1), wall()) is None
    assert intersect_support_surface((0, 0, .7), (1, 0, 2), wall()) is None
    assert intersect_support_surface((0, 0, .7), (math.nan, 0, 1), wall()) is None


def test_perspective_marker_center_uses_diagonals():
    assert marker_center([(0, 0), (4, 0), (3, 2), (1, 2)]) == pytest.approx((2, 4/3))
    with pytest.raises(ValueError):
        marker_center([(0, 0)]*4)
