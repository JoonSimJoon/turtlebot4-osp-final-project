"""ROS-independent frontier selection on an OccupancyGrid.

Only known free cells are traversed. Occupied cells are inflated by the robot
radius; diagonal moves may not cut through blocked corners. Goals stand back
from the unknown boundary so the robot footprint stays in observed space.
"""

from dataclasses import dataclass
import heapq
import math


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Map dimensions must be positive')
        if len(self.data) != self.width * self.height:
            raise ValueError('Map data size does not match dimensions')
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Map resolution must be positive and finite')
        if not all(math.isfinite(v) for v in
                   (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('Map origin must be finite')

    def world_to_cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((c * dx + s * dy) / self.resolution),
                math.floor((-s * dx + c * dy) / self.resolution))

    def cell_to_world(self, x, y):
        dx, dy = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * dx - s * dy, self.origin_y + s * dx + c * dy

    def inside(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def neighbors(self, index, diagonal=False):
        x, y = index % self.width, index // self.width
        offsets = ((1, 0), (-1, 0), (0, 1), (0, -1))
        if diagonal:
            offsets += ((1, 1), (1, -1), (-1, 1), (-1, -1))
        for dx, dy in offsets:
            if self.inside(x + dx, y + dy):
                yield (y + dy) * self.width + x + dx


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    yaw: float
    path_distance: float
    frontier_size: int
    score: float


def inflated_free(grid, robot_radius=0.22, occupied_threshold=65, free_threshold=25):
    """Return a byte mask of obstacle-clear known-free cell centers."""
    if robot_radius < 0 or not math.isfinite(robot_radius):
        raise ValueError('robot_radius must be nonnegative and finite')
    safe = bytearray(0 <= value <= free_threshold for value in grid.data)
    radius = math.ceil(robot_radius / grid.resolution)
    # A half cell diagonal accounts for the area represented by an obstacle cell.
    clearance = robot_radius / grid.resolution + math.sqrt(0.5)
    offsets = [(dx, dy) for dx in range(-radius - 1, radius + 2)
               for dy in range(-radius - 1, radius + 2)
               if dx * dx + dy * dy <= clearance * clearance]
    for index, value in enumerate(grid.data):
        if value < occupied_threshold:
            continue
        # Interior obstacle cells cannot affect free space beyond the boundary.
        if all(grid.data[neighbor] >= occupied_threshold for neighbor in grid.neighbors(index)):
            continue
        x, y = index % grid.width, index // grid.width
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if grid.inside(nx, ny):
                safe[ny * grid.width + nx] = 0
    return safe


def reachable_distances(grid, safe, start, max_cells=200000):
    """Dijkstra distance in cells; no diagonal corner cutting or unknown travel."""
    if not grid.inside(*start):
        return {}
    start_index = start[1] * grid.width + start[0]
    if not safe[start_index]:
        return {}
    distances, queue, settled = {start_index: 0.0}, [(0.0, start_index)], set()
    while queue and len(settled) < max_cells:
        distance, index = heapq.heappop(queue)
        if index in settled:
            continue
        settled.add(index)
        x, y = index % grid.width, index // grid.width
        for neighbor in grid.neighbors(index, diagonal=True):
            if not safe[neighbor]:
                continue
            nx, ny = neighbor % grid.width, neighbor // grid.width
            diagonal = nx != x and ny != y
            if diagonal and not (safe[y * grid.width + nx] and
                                 safe[ny * grid.width + x]):
                continue
            candidate = distance + (math.sqrt(2) if diagonal else 1)
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    # Only settled cells have a proven distance under a bounded search.
    return {index: distances[index] for index in settled}


def known_line_of_sight(grid, start_x, start_y, end_x, end_y, free_threshold=25):
    """Supercover ray through known-free cells, including both corner neighbors.

    The continuous endpoints matter: a prior observation is not generally at a
    cell center. Invalid, unknown or now-occupied observation cells never hide
    a candidate after a SLAM map correction.
    """
    if not all(math.isfinite(value) for value in (start_x, start_y, end_x, end_y)):
        return False
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)

    def coordinates(wx, wy):
        dx, dy = wx - grid.origin_x, wy - grid.origin_y
        return ((c * dx + s * dy) / grid.resolution,
                (-s * dx + c * dy) / grid.resolution)

    def free(x, y):
        return grid.inside(x, y) and 0 <= grid.data[y * grid.width + x] <= free_threshold

    sx, sy = coordinates(start_x, start_y)
    ex, ey = coordinates(end_x, end_y)
    x, y, end_cell_x, end_cell_y = math.floor(sx), math.floor(sy), math.floor(ex), math.floor(ey)
    if not free(x, y) or not free(end_cell_x, end_cell_y):
        return False
    dx, dy = ex - sx, ey - sy
    step_x = 1 if dx > 0 else -1 if dx < 0 else 0
    step_y = 1 if dy > 0 else -1 if dy < 0 else 0
    delta_x, delta_y = (1.0 / abs(dx) if dx else math.inf), (1.0 / abs(dy) if dy else math.inf)
    next_x = ((x + 1 - sx) / dx if dx > 0 else (sx - x) / -dx if dx < 0 else math.inf)
    next_y = ((y + 1 - sy) / dy if dy > 0 else (sy - y) / -dy if dy < 0 else math.inf)
    while (x, y) != (end_cell_x, end_cell_y):
        if abs(next_x - next_y) <= 1e-12:
            if not free(x + step_x, y) or not free(x, y + step_y):
                return False
            x, y = x + step_x, y + step_y
            next_x, next_y = next_x + delta_x, next_y + delta_y
        elif next_x < next_y:
            x, next_x = x + step_x, next_x + delta_x
        else:
            y, next_y = y + step_y, next_y + delta_y
        if not free(x, y):
            return False
    return True


def find_frontiers(grid, robot_x, robot_y, *, robot_radius=0.22,
                   min_cluster_cells=4, min_goal_distance=0.35,
                   standoff=0.35, max_cells=200000, blacklist=()):
    """Rank reachable frontier goals; blacklist entries are (x, y, radius)."""
    if len(grid.data) > max_cells * 5:
        raise ValueError('Map too large for configured bounded frontier search')
    safe = inflated_free(grid, robot_radius)
    reachable = reachable_distances(
        grid, safe, grid.world_to_cell(robot_x, robot_y), max_cells=max_cells)
    if not reachable:
        return []
    frontier = {index for index in reachable if any(
        grid.data[neighbor] < 0 for neighbor in grid.neighbors(index))}
    clusters = []
    while frontier:
        seed = frontier.pop()
        cluster, queue = [seed], [seed]
        while queue:
            for neighbor in grid.neighbors(queue.pop(), diagonal=True):
                if neighbor in frontier:
                    frontier.remove(neighbor)
                    cluster.append(neighbor)
                    queue.append(neighbor)
        if len(cluster) >= min_cluster_cells:
            clusters.append(cluster)
    goals = []
    radius_cells = math.ceil(max(standoff, robot_radius) / grid.resolution)
    unknown_clearance = max(robot_radius, standoff) / grid.resolution
    disk = [(dx, dy) for dx in range(-radius_cells, radius_cells + 1)
            for dy in range(-radius_cells, radius_cells + 1)
            if dx * dx + dy * dy <= unknown_clearance ** 2]
    for cluster in clusters:
        # A representative near the cluster center avoids selecting wall tips.
        cx = sum(index % grid.width for index in cluster) / len(cluster)
        cy = sum(index // grid.width for index in cluster) / len(cluster)
        representatives = sorted(cluster, key=lambda i:
                                 (i % grid.width - cx) ** 2 +
                                 (i // grid.width - cy) ** 2)[:8]
        candidates = set()
        for index in representatives:
            fx, fy = index % grid.width, index // grid.width
            for dx in range(-radius_cells - 2, radius_cells + 3):
                for dy in range(-radius_cells - 2, radius_cells + 3):
                    x, y = fx + dx, fy + dy
                    if grid.inside(x, y) and y * grid.width + x in reachable:
                        candidates.add(y * grid.width + x)
        viable = []
        for index in candidates:
            x, y = index % grid.width, index // grid.width
            if any(not grid.inside(x + dx, y + dy) or
                   grid.data[(y + dy) * grid.width + x + dx] < 0
                   for dx, dy in disk):
                continue
            wx, wy = grid.cell_to_world(x, y)
            if math.hypot(wx - robot_x, wy - robot_y) < min_goal_distance:
                continue
            if any(math.hypot(wx - bx, wy - by) <= radius
                   for bx, by, radius in blacklist):
                continue
            distance_to_frontier = math.hypot(x - cx, y - cy)
            viable.append((distance_to_frontier + reachable[index] * 0.15, index))
        if not viable:
            continue
        _, index = min(viable)
        x, y = grid.cell_to_world(index % grid.width, index // grid.width)
        unknown = {neighbor for boundary in cluster for neighbor in grid.neighbors(boundary)
                   if grid.data[neighbor] < 0}
        ux = sum(index % grid.width for index in unknown) / len(unknown)
        uy = sum(index // grid.width for index in unknown) / len(unknown)
        fx, fy = grid.cell_to_world(ux, uy)
        path_distance = reachable[index] * grid.resolution
        information = len(cluster) * grid.resolution
        score = information / (0.5 + path_distance)
        goals.append(FrontierGoal(x, y, math.atan2(fy - y, fx - x),
                                  path_distance, len(cluster), score))
    return sorted(goals, key=lambda goal: goal.score, reverse=True)


def find_coverage_goal(grid, robot_x, robot_y, *, observed=(),
                       robot_radius=0.22, spacing=1.0, observation_radius=1.2,
                       min_goal_distance=0.35, max_cells=200000, blacklist=()):
    """Choose a reachable unobserved viewpoint even in a fully mapped room.

    LiDAR can map an entire room before the RGB camera has approached its walls.
    A successful panoramic observation covers a disk around that robot position;
    remaining known-free viewpoints are sampled at roughly ``spacing`` meters.
    Wall-adjacent viewpoints receive priority because targets are on surfaces.
    """
    if spacing <= 0 or observation_radius <= 0:
        raise ValueError('Coverage spacing and observation radius must be positive')
    if len(grid.data) > max_cells * 5:
        raise ValueError('Map too large for configured bounded coverage search')
    safe = inflated_free(grid, robot_radius)
    reachable = reachable_distances(
        grid, safe, grid.world_to_cell(robot_x, robot_y), max_cells=max_cells)
    stride = max(1, round(spacing / grid.resolution))
    clearance_cells = math.ceil(robot_radius / grid.resolution)
    footprint = [(dx, dy) for dx in range(-clearance_cells, clearance_cells + 1)
                 for dy in range(-clearance_cells, clearance_cells + 1)
                 if dx * dx + dy * dy <= (robot_radius / grid.resolution) ** 2]
    # Sparse surface probes bound the work per candidate independently of map size.
    probe_step = max(1, round(0.25 / grid.resolution))
    probe_radius = max(1, round(1.8 / grid.resolution))
    probes = [(dx, dy) for dx in range(-probe_radius, probe_radius + 1, probe_step)
              for dy in range(-probe_radius, probe_radius + 1, probe_step)
              if dx * dx + dy * dy <= probe_radius * probe_radius]

    def select(indices):
        best = None
        for index in indices:
            cx, cy = index % grid.width, index // grid.width
            x, y = grid.cell_to_world(cx, cy)
            if math.hypot(x - robot_x, y - robot_y) < min_goal_distance:
                continue
            if any(math.hypot(x - ox, y - oy) <= observation_radius and
                   known_line_of_sight(grid, ox, oy, x, y) for ox, oy in observed):
                continue
            if any(math.hypot(x - bx, y - by) <= radius for bx, by, radius in blacklist):
                continue
            if any(not grid.inside(cx + dx, cy + dy) or
                   grid.data[(cy + dy) * grid.width + cx + dx] < 0
                   for dx, dy in footprint):
                continue
            wall = None
            for dx, dy in probes:
                if grid.inside(cx + dx, cy + dy) and grid.data[(cy + dy) * grid.width + cx + dx] >= 65:
                    if wall is None or dx * dx + dy * dy < wall[0]:
                        wall = dx * dx + dy * dy, cx + dx, cy + dy
            distance = reachable[index] * grid.resolution
            score = (1.5 if wall else 1.0) / (0.5 + distance)
            if best is None or score > best.score:
                if wall:
                    wx, wy = grid.cell_to_world(wall[1], wall[2])
                    yaw = math.atan2(wy - y, wx - x)
                else:
                    yaw = math.atan2(y - robot_y, x - robot_x)
                best = FrontierGoal(x, y, yaw, distance, 0, score)
        return best

    sampled = [index for index in reachable
               if index % grid.width % stride == 0 and index // grid.width % stride == 0]
    goal = select(sampled)
    if goal is not None:
        return goal
    # A thin corridor may fall between lattice samples. Keep this fallback bounded
    # and nearest-first so isolated unsampled reachable pockets remain visitable.
    return select(sorted(reachable, key=reachable.get)[::max(1, len(reachable) // 2000)])
