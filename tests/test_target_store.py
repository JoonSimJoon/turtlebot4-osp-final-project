import csv
import math

import pytest

from tb4_mission.target_store import TargetStore, write_targets_csv


def test_multiple_time_separated_observations_confirm_unknown_id():
    store = TargetStore(confirmations=3)
    assert store.observe(817, (2, 1, .7), 1.0)
    assert not store.observe(817, (2, 1, .7), 1.0)
    assert not store.observe(817, (2, 1, .7), 1.01)
    assert store.project((0, 0, 0), (0, 0, 0, 1)) == []
    assert store.observe(817, (2.02, 1.02, .7), 1.2)
    assert store.observe(817, (1.98, .98, .7), 1.4)
    target = store.project((0, 0, 0), (0, 0, 0, 1))[0]
    assert target == dict(id=817, x=2, y=1, z=.7, observations=3)


def test_outliers_and_old_timestamps_do_not_move_confirmed_target():
    store = TargetStore(confirmations=1)
    store.observe(3, (1, 2, .5), 5)
    assert not store.observe(3, (9, 9, .5), 6)
    assert not store.observe(3, (1, 2, .5), 4)
    assert not store.observe(3, (math.nan, 2, .5), 6)
    assert store.project((0, 0, 0), (0, 0, 0, 1))[0]["observations"] == 1


def test_expired_unconfirmed_track_restarts_confirmation():
    store = TargetStore(confirmations=3, candidate_window=2)
    store.observe(7, (1, 2, .5), 1)
    store.observe(7, (1, 2, .5), 1.2)
    store.observe(7, (3, 2, .5), 4)
    assert not store.tracks[7].confirmed
    assert store.tracks[7].observations == 1


def test_map_loop_closure_reprojects_existing_odom_observations():
    store = TargetStore(confirmations=1)
    store.observe(7, (2, 1, .5), 1)
    original = store.project((0, 0, 0), (0, 0, 0, 1))[0]
    corrected = store.project((1, -.5, 0), (0, 0, math.sqrt(.5), math.sqrt(.5)))[0]
    assert (original["x"], original["y"]) == (2, 1)
    assert (corrected["x"], corrected["y"]) == pytest.approx((0, 1.5))


def test_atomic_csv_empty_header_and_sorted_complete_replacement(tmp_path):
    path = write_targets_csv(tmp_path, [])
    assert path.read_text() == "id,x,y\n"
    write_targets_csv(tmp_path, [dict(id=81, x=1.25, y=-2.75), dict(id=3, x=0, y=0)])
    with path.open() as stream:
        assert list(csv.DictReader(stream)) == [
            dict(id="3", x="0.000000", y="0.000000"),
            dict(id="81", x="1.250000", y="-2.750000")]
    assert len(list(tmp_path.iterdir())) == 1


def test_reset_removes_confirmed_and_unconfirmed_tracks():
    store = TargetStore(confirmations=1)
    store.observe(1, (1, 1, 1), 1)
    store.clear()
    assert store.project((0, 0, 0), (0, 0, 0, 1)) == []
