import math

from tb4_mission.mission_logic import GoalBlacklist, confirmed_target_ids, terminal_reason


def test_timer_takes_precedence_at_exact_deadline():
    assert terminal_reason(599.99, {1, 2}) is None
    assert terminal_reason(600.0, {1, 2}) == 'time_limit'
    assert terminal_reason(600.0, {1, 2, 3}) == 'time_limit'
    assert terminal_reason(30.0, [2, 2, 7]) is None
    assert terminal_reason(30.0, [2, 7, 29]) == 'targets_found'


def test_target_ids_require_real_coordinates_and_valid_original_dictionary_id():
    payload = {'targets': [
        {'id': 8, 'x': 1, 'y': 2, 'observations': 3},
        {'id': 8, 'x': 1, 'y': 2},
        {'id': 99, 'x': 1, 'y': math.nan},
        {'id': 1024, 'x': 1, 'y': 2},
        {'id': True, 'x': 1, 'y': 2},
        {'id': 1}, None,
    ]}
    assert confirmed_target_ids(payload) == {8}
    assert confirmed_target_ids({'targets': 'bad'}) == set()


def test_blacklist_expiration_and_world_space_radius():
    blacklist = GoalBlacklist(ttl=5.0, radius=0.6)
    blacklist.add(1.0, 2.0, 10.0)
    assert blacklist.active(14.99) == [(1.0, 2.0, 0.6)]
    assert blacklist.active(15.0) == []
