"""Small deterministic mission policies, independent of ROS and wall clocks."""

import math


def terminal_reason(elapsed, target_ids, time_limit=600.0, target_count=3):
    if elapsed >= time_limit:
        return 'time_limit'
    if len(set(target_ids)) >= target_count:
        return 'targets_found'
    return None


def confirmed_target_ids(payload):
    result = set()
    if not isinstance(payload, dict):
        return result
    targets = payload.get('targets', [])
    if not isinstance(targets, list):
        return result
    for target in targets:
        if not isinstance(target, dict):
            continue
        marker_id = target.get('id')
        if isinstance(marker_id, bool) or not isinstance(marker_id, int):
            continue
        if not 0 <= marker_id < 1024:
            continue
        try:
            finite = all(math.isfinite(float(target[key])) for key in ('x', 'y'))
        except (KeyError, TypeError, ValueError):
            continue
        if finite and target.get('observations', 1) >= 1:
            result.add(marker_id)
    return result


class GoalBlacklist:
    def __init__(self, ttl=60.0, radius=0.6):
        if ttl <= 0 or radius < 0:
            raise ValueError('Blacklist TTL must be positive and radius nonnegative')
        self.ttl, self.radius, self.entries = ttl, radius, []

    def add(self, x, y, now):
        self.entries.append((float(x), float(y), now + self.ttl))

    def active(self, now):
        self.entries = [entry for entry in self.entries if entry[2] > now]
        return [(x, y, self.radius) for x, y, _ in self.entries]
