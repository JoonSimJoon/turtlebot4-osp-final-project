import json
import math

import pytest

from tb4_mission.frontier import GridMap
from tb4_mission.map_io import atomic_write, save_map, write_json


def test_pgm_flips_ros_rows_and_preserves_unknown(tmp_path):
    grid = GridMap(3, 2, 0.05, -1.0, 2.0, math.pi / 2,
                   (0, 100, -1, 25, 65, 50))
    image, metadata = save_map(grid, tmp_path)
    payload = image.read_bytes()
    assert payload.startswith(b'P5\n# TurtleBot4 OSP mission\n3 2\n255\n')
    assert payload[-6:] == bytes([254, 0, 205, 254, 0, 205])
    contents = metadata.read_text()
    assert 'image: "map.pgm"' in contents
    assert 'origin: [-1, 2, 1.57079632679]' in contents
    assert 'resolution: 0.05' in contents
    assert 'negate: 0' in contents
    assert 'free_thresh: 0.196' in contents
    # Nav2 trinary loading uses (255-pixel)/255 and these YAML thresholds.
    restored = [0 if (255 - pixel) / 255 <= 0.196 else
                100 if (255 - pixel) / 255 >= 0.65 else -1
                for pixel in payload[-6:]]
    assert restored == [0, 100, -1, 0, 100, -1]


def test_atomic_replace_and_valid_json(tmp_path):
    path = tmp_path / 'nested' / 'summary.json'
    write_json(path, {'reason': 'targets_found', 'ids': [4, 32, 71]})
    assert json.loads(path.read_text())['ids'] == [4, 32, 71]
    atomic_write(path, '{"done": true}')
    assert json.loads(path.read_text()) == {'done': True}
    assert [p.name for p in path.parent.iterdir()] == ['summary.json']


def test_invalid_map_name_and_nonfinite_json_rejected(tmp_path):
    grid = GridMap(1, 1, 0.1, 0.0, 0.0, 0.0, (0,))
    with pytest.raises(ValueError):
        save_map(grid, tmp_path, '../outside')
    with pytest.raises(ValueError):
        write_json(tmp_path / 'bad.json', {'bad': math.nan})
    assert not (tmp_path / 'bad.json').exists()
