"""Atomic export of ROS-independent map data in Nav2 map_server format."""

import json
import os
from pathlib import Path
import tempfile


def atomic_write(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.' + path.name,
                                         delete=False) as stream:
            temporary = stream.name
            stream.write(payload.encode('utf-8') if isinstance(payload, str) else payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and os.path.exists(temporary):
            os.unlink(temporary)


def write_json(path, data):
    atomic_write(path, json.dumps(data, ensure_ascii=False, indent=2,
                                 allow_nan=False) + '\n')


def save_map(grid, output_dir, name='map'):
    """Save a trinary PGM and YAML preserving grid origin and vertical ordering."""
    if not name or Path(name).name != name:
        raise ValueError('Map name must be a filename without directories')
    output_dir = Path(output_dir)
    pixels = bytearray()
    for row in range(grid.height - 1, -1, -1):
        for value in grid.data[row * grid.width:(row + 1) * grid.width]:
            pixels.append(254 if 0 <= value <= 25 else 0 if value >= 65 else 205)
    header = f'P5\n# TurtleBot4 OSP mission\n{grid.width} {grid.height}\n255\n'.encode('ascii')
    image_path = output_dir / (name + '.pgm')
    yaml_path = output_dir / (name + '.yaml')
    atomic_write(image_path, header + pixels)
    metadata = (f'image: {json.dumps(image_path.name)}\nmode: trinary\n'
                f'resolution: {grid.resolution:.12g}\n'
                f'origin: [{grid.origin_x:.12g}, {grid.origin_y:.12g}, '
                f'{grid.origin_yaw:.12g}]\nnegate: 0\n'
                # The saved gray 205 must reload as unknown, not as free.
                'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    atomic_write(yaml_path, metadata)
    return image_path, yaml_path
