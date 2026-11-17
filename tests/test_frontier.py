import math
import pytest

from tb4_mission.frontier import (GridMap, find_coverage_goal, find_frontiers,
                                 inflated_free, known_line_of_sight, reachable_distances)


def grid(rows, resolution=1.0, origin=(0.0, 0.0, 0.0)):
    symbols = {'.': 0, '#': 100, '?': -1}
    return GridMap(len(rows[0]), len(rows), resolution, *origin,
                   tuple(symbols[cell] for row in rows for cell in row))


def test_coordinates_respect_rotated_origin_and_cell_centers():
    value = grid(['..', '..'], 0.5, (2.0, -3.0, math.pi / 2))
    assert value.cell_to_world(0, 0) == pytest.approx((1.75, -2.75))
    for x in range(2):
        for y in range(2):
            assert value.world_to_cell(*value.cell_to_world(x, y)) == (x, y)
    assert value.world_to_cell(2.1, -3.1) == (-1, -1)


def test_reject_invalid_maps():
    with pytest.raises(ValueError):
        GridMap(2, 3, 1.0, 0, 0, 0, (0,))
    with pytest.raises(ValueError):
        GridMap(1, 1, 0.0, 0, 0, 0, (0,))


def test_no_diagonal_corner_cutting():
    value = grid(['.#', '#.'])
    distances = reachable_distances(value, inflated_free(value, 0.0), (0, 0))
    assert distances == {0: 0.0}


def test_unknown_is_not_traversable():
    value = grid(['.?.', '.?.', '.?.'])
    reachable = reachable_distances(value, inflated_free(value, 0.0), (0, 1))
    assert set(reachable) == {0, 3, 6}


def test_inflation_blocks_wall_clearance():
    value = grid(['.....', '..#..', '.....'], resolution=0.1)
    safe = inflated_free(value, 0.1)
    assert safe[0] == 1
    assert safe[6] == 0
    assert safe[7] == 0
    assert safe[8] == 0
    assert safe[2] == 0


def test_frontier_goal_is_in_known_space_and_faces_unknown():
    value = grid(['.........???'] * 8, resolution=0.1)
    result = find_frontiers(value, 0.25, 0.35, robot_radius=0.1,
                            standoff=0.15, min_cluster_cells=3,
                            min_goal_distance=0.1)
    assert result
    goal = result[0]
    assert 0.45 <= goal.x <= 0.75
    assert abs(goal.yaw) < 0.4
    cell = value.world_to_cell(goal.x, goal.y)
    assert value.data[cell[1] * value.width + cell[0]] == 0


def test_disconnected_frontier_is_not_selected():
    value = grid(['....#....???'] * 8, resolution=0.1)
    result = find_frontiers(value, 0.15, 0.35, robot_radius=0.0,
                            standoff=0.0, min_cluster_cells=2,
                            min_goal_distance=0.0)
    assert result == []


def test_blacklist_and_explored_map_return_no_goal():
    value = grid(['.........???'] * 8, resolution=0.1)
    assert not find_frontiers(value, 0.25, 0.35, robot_radius=0.0,
                               min_cluster_cells=2, blacklist=[(0.6, 0.4, 10.0)])
    assert not find_frontiers(grid(['........'] * 8), 2.5, 2.5)


def test_search_is_bounded_and_returns_only_settled_distances():
    value = grid(['........'] * 8)
    result = reachable_distances(value, inflated_free(value, 0), (0, 0), max_cells=4)
    assert len(result) == 4
    assert result[0] == 0


def test_frontier_goal_rotates_with_map_origin():
    value = grid(['.........???'] * 8, resolution=0.1,
                 origin=(3.0, -1.0, math.pi / 2))
    robot = value.cell_to_world(2, 3)
    result = find_frontiers(value, *robot, robot_radius=0.1,
                            standoff=0.15, min_cluster_cells=3,
                            min_goal_distance=0.1)
    assert result
    assert result[0].yaw == pytest.approx(math.pi / 2, abs=0.4)


def test_fully_known_room_still_requires_camera_coverage():
    value = grid(['####################'] + ['#..................#'] * 18 +
                 ['####################'], resolution=0.2)
    assert find_frontiers(value, 1.0, 1.0, robot_radius=0.2) == []
    goal = find_coverage_goal(value, 1.0, 1.0, observed=[(1.0, 1.0)],
                              robot_radius=0.2, observation_radius=1.0)
    assert goal is not None
    assert math.hypot(goal.x - 1.0, goal.y - 1.0) > 1.0
    safe = inflated_free(value, 0.2)
    x, y = value.world_to_cell(goal.x, goal.y)
    assert safe[y * value.width + x]


def test_coverage_does_not_cross_wall_and_exhausts_observed_room():
    value = grid(['..........#..........'] * 10, resolution=0.2)
    goal = find_coverage_goal(value, 0.7, 0.9, robot_radius=0.2)
    assert goal is not None and goal.x < 2.0
    assert find_coverage_goal(value, 0.7, 0.9, robot_radius=0.2,
                               observed=[(0.7, 0.9)], observation_radius=3.0) is None


def test_coverage_blacklist_rejects_failed_viewpoints():
    value = grid(['..........'] * 10, resolution=0.2)
    assert find_coverage_goal(value, 0.7, 0.9, robot_radius=0.1,
                               blacklist=[(1.0, 1.0, 5.0)]) is None


def test_coverage_does_not_count_views_through_parallel_corridor_wall():
    width, height = 12, 24
    data = [100] * (width * height)
    for y in range(2, 22):
        for x in (2, 3, 4, 6, 7, 8):
            data[y * width + x] = 0
    for y in range(19, 22):
        for x in range(2, 9):
            data[y * width + x] = 0
    value = GridMap(width, height, 0.25, 0.0, 0.0, 0.0, tuple(data))
    observed = [value.cell_to_world(3, y) for y in range(3, 21)]
    robot = value.cell_to_world(3, 10)
    goal = find_coverage_goal(value, *robot, observed=observed,
                              robot_radius=0.22, observation_radius=1.2)
    assert goal is not None
    assert value.world_to_cell(goal.x, goal.y)[0] >= 6
    assert not known_line_of_sight(value, *robot, *value.cell_to_world(7, 10))


def test_visibility_supercover_blocks_corners_unknown_and_stale_observation_cells():
    assert known_line_of_sight(grid(['..', '..']), 0.5, 0.5, 1.5, 1.5)
    assert not known_line_of_sight(grid(['.#', '..']), 0.5, 0.5, 1.5, 1.5)
    assert not known_line_of_sight(grid(['.?', '..']), 0.5, 0.5, 1.5, 1.5)
    value = grid(['#.........'] * 10, resolution=0.2)
    # A stale SLAM observation now inside a wall cannot hide the entire room.
    assert find_coverage_goal(value, 0.7, 0.9, robot_radius=0.1,
                               observed=[(0.1, 0.9)], observation_radius=5.0) is not None
