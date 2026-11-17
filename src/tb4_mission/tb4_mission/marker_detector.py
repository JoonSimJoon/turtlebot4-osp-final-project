"""Detect variable-size ArUco targets using calibrated RGB + 2-D LiDAR."""

from collections import deque
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .marker_geometry import (detect_original_markers, intersect_support_surface,
                              marker_center, rotate_vector, scan_points)
from .target_store import TargetStore, write_finalized_receipt, write_targets_csv


def _stamp(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def _rigid(transform):
    translation, rotation = transform.transform.translation, transform.transform.rotation
    return ((translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w))


class MarkerDetector(Node):
    def __init__(self):
        super().__init__("marker_detector")
        defaults = {
            "output_dir": "results/latest", "map_frame": "map", "odom_frame": "odom",
            "base_frame": "base_link", "scan_topic": "scan",
            "image_topic": "/oakd/rgb/preview/image_raw",
            "camera_info_topic": "/oakd/rgb/preview/camera_info",
            "sync_tolerance_sec": 0.15, "sensor_max_age_sec": 1.5,
            "processing_hz": 5.0, "processing_delay_sec": 0.15, "confirmations": 3,
            "confirmation_spread_m": 0.35, "confirmation_interval_sec": 0.15,
            "confirmation_window_sec": 4.0, "surface_max_gap_m": 0.20,
            "surface_max_residual_m": 0.035, "surface_min_support": 5,
            "max_target_distance_m": 6.0, "max_confirmation_distance_m": 1.8,
            "tracking_enabled": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if self.p["processing_hz"] <= 0 or self.p["sensor_max_age_sec"] <= 0:
            raise ValueError("processing_hz and sensor_max_age_sec must be positive")
        self.store = TargetStore(self.p["confirmations"], self.p["confirmation_spread_m"],
                                 self.p["confirmation_interval_sec"], self.p["confirmation_window_sec"])
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.scans = deque(maxlen=80)
        self.images = deque(maxlen=20)
        self.image = None
        self.camera_info = None
        self.last_image_stamp = -math.inf
        self.last_ready = -math.inf
        self.last_warning = {}
        self.last_export = None
        self.tracking_active = self.p["tracking_enabled"]
        self.finalize_pending = False
        self.frozen_targets = None
        self.receipt_path = Path(self.p["output_dir"]).expanduser() / "targets_finalized.json"
        self.receipt_path.unlink(missing_ok=True)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.targets_pub = self.create_publisher(String, "mission/targets", transient)
        self.markers_pub = self.create_publisher(MarkerArray, "mission/markers", transient)
        self.ready_pub = self.create_publisher(Bool, "mission/perception_ready", 1)
        self.create_subscription(LaserScan, self.p["scan_topic"], self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.p["image_topic"], self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p["camera_info_topic"], self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "mission/state", self.on_state, transient)
        self.create_service(Trigger, "mission/reset_targets", self.reset_targets)
        self.create_timer(1.0/self.p["processing_hz"], self.process_image)
        self.create_timer(0.5, self.publish)
        self.write_export([])
        self.publish_targets([])
        self.get_logger().info("ArUco ORIGINAL + LiDAR surface localization ready; waiting for camera, scan and TF")

    def on_image(self, message):
        self.image = message
        self.images.append(message)

    def on_scan(self, message):
        self.scans.append(message)

    def on_info(self, message):
        self.camera_info = message

    def on_state(self, message):
        try:
            state = json.loads(message.data).get("state")
        except (ValueError, TypeError, AttributeError):
            return
        if state == "stopping":
            self.tracking_active = False
        elif state in ("finished", "error") and self.frozen_targets is None:
            self.tracking_active = False
            self.finalize_pending = True
            self.publish()

    def warn(self, reason):
        now = time.monotonic()
        if now - self.last_warning.get(reason, -math.inf) > 10.0:
            self.get_logger().warning(reason)
            self.last_warning[reason] = now

    def reset_targets(self, request, response):
        del request
        try:
            self.receipt_path.unlink(missing_ok=True)
        except OSError as exc:
            response.success = False
            response.message = "Unable to remove stale finalization receipt: " + str(exc)
            return response
        self.store.clear()
        self.tracking_active = self.p["tracking_enabled"]
        self.finalize_pending = False
        self.frozen_targets = None
        self.last_image_stamp = _stamp(self.image.header.stamp) if self.image is not None else -math.inf
        saved = self.write_export([])
        self.publish_targets([])
        response.success = saved
        response.message = "Target observations cleared" if saved else "Unable to write targets.csv"
        return response

    def process_image(self):
        info = self.camera_info
        if not self.images or info is None or not self.scans:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        eligible = [item for item in self.images
                    if now-_stamp(item.header.stamp) >= self.p["processing_delay_sec"]]
        if not eligible:
            return
        image = eligible[-1]
        stamp = _stamp(image.header.stamp)
        if stamp <= self.last_image_stamp:
            return
        self.last_image_stamp = stamp
        if stamp <= 0 or not -0.1 <= now-stamp <= self.p["sensor_max_age_sec"]:
            self.last_ready = -math.inf
            self.warn("Camera timestamp is stale or clocks differ; synchronize laptop and robot clocks")
            return
        if (info.width != image.width or info.height != image.height or
                not image.header.frame_id or info.header.frame_id != image.header.frame_id or
                info.k[0] <= 0 or info.k[4] <= 0 or
                not all(math.isfinite(value) for value in list(info.k)+list(info.d))):
            self.last_ready = -math.inf
            self.warn("CameraInfo dimensions, frame or calibration do not match RGB image")
            return
        scan = min(self.scans, key=lambda item: abs(_stamp(item.header.stamp)-stamp))
        if abs(_stamp(scan.header.stamp)-stamp) > self.p["sync_tolerance_sec"]:
            self.last_ready = -math.inf
            self.warn("No synchronized LiDAR scan for camera image")
            return
        if (not scan.header.frame_id or scan.angle_increment == 0 or
                not all(math.isfinite(value) for value in
                        (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)) or
                scan.range_min < 0 or scan.range_max <= scan.range_min or
                sum(math.isfinite(value) and scan.range_min <= value <= scan.range_max
                    for value in scan.ranges) < self.p["surface_min_support"]):
            self.last_ready = -math.inf
            self.warn("LiDAR scan has insufficient finite returns")
            return
        try:
            # Resolve each moving sensor at its own capture time in a fixed frame.
            camera_transform = self.tf.lookup_transform(
                self.p["odom_frame"], image.header.frame_id, Time.from_msg(image.header.stamp))
            scan_transform = self.tf.lookup_transform(
                self.p["odom_frame"], scan.header.frame_id, Time.from_msg(scan.header.stamp))
            base_transform = self.tf.lookup_transform(
                self.p["odom_frame"], self.p["base_frame"], Time.from_msg(image.header.stamp))
            origin, rotation = _rigid(camera_transform)
            base_origin, _ = _rigid(base_transform)
            scan_translation, scan_rotation = _rigid(scan_transform)
            gray = self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8")
            if info.distortion_model not in ("", "plumb_bob", "rational_polynomial", "equidistant"):
                raise ValueError("Unsupported camera distortion model: " + info.distortion_model)
            points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                 scan.range_min, scan.range_max, scan_translation, scan_rotation)
            detections = detect_original_markers(gray)
            self.last_ready = time.monotonic()
            if not self.tracking_active:
                return
            camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(info.d, dtype=np.float64)
            for marker_id, corners in detections:
                pixel = np.asarray(marker_center(corners), dtype=np.float64).reshape(1, 1, 2)
                if info.distortion_model == "equidistant":
                    normalized = cv2.fisheye.undistortPoints(pixel, camera_matrix, distortion)
                else:
                    normalized = cv2.undistortPoints(pixel, camera_matrix, distortion)
                u, v = normalized.reshape(2)
                ray = rotate_vector((float(u), float(v), 1.0), rotation)
                estimate = intersect_support_surface(
                    origin, ray, points, max_gap=self.p["surface_max_gap_m"],
                    max_residual=self.p["surface_max_residual_m"],
                    min_support=self.p["surface_min_support"],
                    max_distance=self.p["max_target_distance_m"])
                if (estimate is not None and math.hypot(estimate.x-base_origin[0],
                                                       estimate.y-base_origin[1]) <=
                        self.p["max_confirmation_distance_m"]):
                    self.store.observe(marker_id, (estimate.x, estimate.y, estimate.z), stamp)
        except (TransformException, ValueError, CvBridgeError, cv2.error) as exc:
            self.last_ready = -math.inf
            self.warn("Perception waiting: " + str(exc))

    def write_export(self, targets):
        # Export updates after loop closure even if there are no new detections.
        signature = tuple((item["id"], round(item["x"], 6), round(item["y"], 6)) for item in targets)
        if signature == self.last_export:
            return True
        try:
            write_targets_csv(self.p["output_dir"], targets)
            self.last_export = signature
            return True
        except OSError as exc:
            self.warn("Unable to save targets.csv: " + str(exc))
            return False

    def publish_targets(self, targets):
        self.targets_pub.publish(String(data=json.dumps({"frame_id": self.p["map_frame"],
                                                        "targets": targets})))
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        for target in targets:
            for label in (False, True):
                marker = Marker()
                marker.header.frame_id = self.p["map_frame"]
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "target_labels" if label else "targets"
                marker.id = target["id"]
                marker.type = Marker.TEXT_VIEW_FACING if label else Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = target["x"]
                marker.pose.position.y = target["y"]
                marker.pose.position.z = 0.35 if label else 0.12
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.22 if label else 0.16
                marker.color.r = 1.0 if label else 0.1
                marker.color.g = 0.9
                marker.color.b = 0.1
                marker.color.a = 1.0
                if label:
                    marker.text = f'ID {target["id"]}'
                message.markers.append(marker)
        self.markers_pub.publish(message)

    def finalize(self, targets):
        if not self.write_export(targets):
            return
        try:
            write_finalized_receipt(self.p["output_dir"], self.p["map_frame"], targets)
            self.frozen_targets = targets
        except OSError as exc:
            self.warn("Unable to acknowledge target finalization: " + str(exc))

    def publish(self):
        self.ready_pub.publish(Bool(data=time.monotonic()-self.last_ready <= self.p["sensor_max_age_sec"]))
        if self.frozen_targets is not None:
            self.publish_targets(self.frozen_targets)
            return
        if not any(track.confirmed for track in self.store.tracks.values()):
            self.publish_targets([])
            if self.finalize_pending:
                self.finalize([])
            return
        try:
            translation, rotation = _rigid(self.tf.lookup_transform(
                self.p["map_frame"], self.p["odom_frame"], Time()))
            targets = self.store.project(translation, rotation)
            self.publish_targets(targets)
            if self.finalize_pending:
                self.finalize(targets)
            else:
                self.write_export(targets)
        except TransformException as exc:
            self.warn("Confirmed targets waiting for map->odom TF: " + str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
