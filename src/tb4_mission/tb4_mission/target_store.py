"""Confirmation, fixed-frame storage, and atomic map-frame target exports."""

from dataclasses import dataclass, field
import csv
import json
import math
import os
from pathlib import Path
import statistics
import tempfile

from .marker_geometry import transform_point


@dataclass
class Track:
    samples: list = field(default_factory=list)
    observations: int = 0
    confirmed: bool = False
    last_stamp: float = -math.inf

    @property
    def point(self):
        return tuple(statistics.median(p[axis] for _, p in self.samples) for axis in range(3))


class TargetStore:
    def __init__(self, confirmations=3, max_spread=0.35, min_interval=0.15,
                 candidate_window=4.0, max_samples=25):
        if confirmations < 1 or max_spread <= 0 or min_interval < 0 or candidate_window <= 0:
            raise ValueError("Invalid confirmation parameters")
        self.confirmations = confirmations
        self.max_spread = max_spread
        self.min_interval = min_interval
        self.candidate_window = candidate_window
        self.max_samples = max_samples
        self.tracks = {}

    def observe(self, marker_id, point, stamp):
        """Return True when accepted; repeated/stale stamps and outliers are ignored."""
        if not isinstance(marker_id, int) or marker_id < 0:
            return False
        if len(point) != 3 or not all(math.isfinite(v) for v in (*point, stamp)):
            return False
        track = self.tracks.setdefault(marker_id, Track())
        if stamp-track.last_stamp < self.min_interval:
            return False
        if not track.confirmed and stamp-track.last_stamp > self.candidate_window:
            track.samples.clear()
            track.observations = 0
        if track.samples and math.dist(point[:2], track.point[:2]) > self.max_spread:
            return False
        track.samples.append((stamp, tuple(point)))
        track.samples = track.samples[-self.max_samples:]
        track.last_stamp = stamp
        track.observations += 1
        track.confirmed = track.observations >= self.confirmations
        return True

    def project(self, translation, quaternion):
        targets = []
        for marker_id, track in sorted(self.tracks.items()):
            if not track.confirmed:
                continue
            x, y, z = transform_point(track.point, translation, quaternion)
            targets.append(dict(id=marker_id, x=x, y=y, z=z, observations=track.observations))
        return targets

    def clear(self):
        self.tracks.clear()


def write_targets_csv(output_dir, targets):
    """Replace a complete CSV atomically; readers never see a partial row."""
    directory = Path(output_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "targets.csv"
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", newline="", encoding="utf-8",
                                         dir=directory, prefix=".targets-", delete=False) as stream:
            temp_name = stream.name
            writer = csv.writer(stream)
            writer.writerow(["id", "x", "y"])
            for target in sorted(targets, key=lambda item: item["id"]):
                writer.writerow([target["id"], f'{target["x"]:.6f}', f'{target["y"]:.6f}'])
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    finally:
        if temp_name and os.path.exists(temp_name):
            os.unlink(temp_name)
    return path


def write_finalized_receipt(output_dir, frame_id, targets):
    """Acknowledge a successful terminal CSV snapshot to the process supervisor."""
    directory = Path(output_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "targets_finalized.json"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory,
                                         prefix=".targets-finalized-", delete=False) as stream:
            temporary = stream.name
            json.dump({"finalized": True, "frame_id": frame_id,
                       "target_ids": sorted(item["id"] for item in targets),
                       "csv": "targets.csv"}, stream, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
    return path
