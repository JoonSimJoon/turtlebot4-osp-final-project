"""ROS-independent camera-ray / vertical LiDAR-surface intersection.

No marker width is assumed. Camera and scan points must already share a fixed
frame. A vertical supporting wall has the same XY at camera and laser height.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


@dataclass(frozen=True)
class SurfaceEstimate:
    x: float
    y: float
    z: float
    distance: float
    residual: float
    support: int


def marker_center(corners):
    """Projective square center is the intersection of image diagonals."""
    p, q, r, s = [tuple(float(v) for v in corner) for corner in corners]
    first, second = (r[0]-p[0], r[1]-p[1]), (s[0]-q[0], s[1]-q[1])
    denominator = _cross(first, second)
    if abs(denominator) < 1e-9:
        raise ValueError("Degenerate marker quadrilateral")
    fraction = _cross((q[0]-p[0], q[1]-p[1]), second)/denominator
    return p[0]+fraction*first[0], p[1]+fraction*first[1]


def detect_original_markers(image):
    """Support both OpenCV 4.5 (Ubuntu) and the newer ArucoDetector API."""
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            image, dictionary, parameters=cv2.aruco.DetectorParameters_create())
    if ids is None:
        return []
    return [(int(marker_id), corner.reshape(4, 2))
            for marker_id, corner in zip(ids.flatten(), corners)]


def rotate_vector(vector, quaternion):
    """Rotate a 3-vector using an (x,y,z,w) unit quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx + w*tx + y*tz-z*ty,
            vy + w*ty + z*tx-x*tz,
            vz + w*tz + x*ty-y*tx)


def transform_point(point, translation, quaternion):
    return tuple(a+b for a, b in zip(rotate_vector(point, quaternion), translation))


def scan_points(ranges, angle_min, angle_increment, range_min, range_max,
                translation=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
    """Preserve invalid beams as None so surfaces never bridge missing returns."""
    result = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            result.append(None)
            continue
        angle = angle_min + index*angle_increment
        result.append(transform_point((distance*math.cos(angle),
                                       distance*math.sin(angle), 0.0),
                                      translation, quaternion))
    return result


def _cross(a, b):
    return a[0]*b[1] - a[1]*b[0]


def _separation(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def _fit_line(points):
    n = len(points)
    cx, cy = sum(p[0] for p in points)/n, sum(p[1] for p in points)/n
    xx = sum((p[0]-cx)**2 for p in points)/n
    yy = sum((p[1]-cy)**2 for p in points)/n
    xy = sum((p[0]-cx)*(p[1]-cy) for p in points)/n
    if xx + yy < 1e-9:
        return None
    angle = 0.5*math.atan2(2*xy, xx-yy)
    along = (math.cos(angle), math.sin(angle))
    normal = (-along[1], along[0])
    residual = math.sqrt(sum(((p[0]-cx)*normal[0] +
                              (p[1]-cy)*normal[1])**2 for p in points)/n)
    return (cx, cy), along, normal, residual


def intersect_support_surface(origin: Sequence[float], direction: Sequence[float],
                              points: Sequence[Optional[Sequence[float]]], *,
                              max_gap=0.20, min_support=5, support_window=5,
                              max_residual=0.035, min_incidence=0.18,
                              min_distance=0.20, max_distance=6.0,
                              min_height=0.0, max_height=3.0):
    """Intersect a camera ray with the nearest continuous observed surface.

    A neighboring beam pair must bracket the ray. Invalid beams/depth jumps are
    never interpolated. The nearest pair is decisive: if its support is bad, we
    reject rather than incorrectly selecting a farther wall through an occluder.
    """
    if not all(math.isfinite(v) for v in (*origin, *direction)):
        return None
    horizontal_norm = math.hypot(direction[0], direction[1])
    if horizontal_norm < 1e-6:
        return None
    ray = (direction[0]/horizontal_norm, direction[1]/horizontal_norm)
    candidates = []
    for i in range(len(points)-1):
        p, q = points[i], points[i+1]
        if p is None or q is None or _separation(p, q) > max_gap:
            continue
        edge = (q[0]-p[0], q[1]-p[1])
        denominator = _cross(ray, edge)
        if abs(denominator) < 1e-9:
            continue
        offset = (p[0]-origin[0], p[1]-origin[1])
        distance = _cross(offset, edge)/denominator
        fraction = _cross(offset, ray)/denominator
        if min_distance <= distance <= max_distance and -1e-6 <= fraction <= 1+1e-6:
            candidates.append((distance, i))
    if not candidates:
        return None
    _, index = min(candidates)
    left, right = index, index+1
    for _ in range(support_window):
        if left <= 0 or points[left-1] is None or _separation(points[left-1], points[left]) > max_gap:
            break
        left -= 1
    for _ in range(support_window):
        if right+1 >= len(points) or points[right+1] is None or _separation(points[right], points[right+1]) > max_gap:
            break
        right += 1
    support = points[left:right+1]
    if len(support) < min_support:
        return None
    line = _fit_line(support)
    if line is None:
        return None
    center, along, normal, residual = line
    incidence = ray[0]*normal[0] + ray[1]*normal[1]
    if residual > max_residual or abs(incidence) < min_incidence:
        return None
    distance = ((center[0]-origin[0])*normal[0] +
                (center[1]-origin[1])*normal[1])/incidence
    if not min_distance <= distance <= max_distance:
        return None
    x, y = origin[0]+distance*ray[0], origin[1]+distance*ray[1]
    projection = (x-center[0])*along[0] + (y-center[1])*along[1]
    extents = [(p[0]-center[0])*along[0] + (p[1]-center[1])*along[1] for p in support]
    if not min(extents)-0.02 <= projection <= max(extents)+0.02:
        return None
    z = origin[2] + distance*direction[2]/horizontal_norm
    if not min_height <= z <= max_height:
        return None
    return SurfaceEstimate(x, y, z, distance, residual, len(support))
