import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from tb4_mission.marker_geometry import detect_original_markers, marker_center


@pytest.mark.parametrize("marker_id,size", [(7, 100), (42, 140), (817, 180)])
def test_original_dictionary_variable_size_and_unknown_id(marker_id, size):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, size)
    image = np.full((size+80, size+80), 255, dtype=np.uint8)
    image[40:40+size, 40:40+size] = marker
    detections = detect_original_markers(image)
    assert len(detections) == 1
    actual_id, corners = detections[0]
    assert actual_id == marker_id
    assert marker_center(corners) == pytest.approx(((size+79)/2, (size+79)/2), abs=1.0)


def test_blank_frame_has_no_false_target():
    assert detect_original_markers(np.full((250, 250), 255, dtype=np.uint8)) == []


def test_ros_pipeline_localizes_confirms_and_exports(tmp_path, monkeypatch):
    """Exercise real ROS messages/TF/cv_bridge against synthetic camera+scan."""
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo, LaserScan
    from builtin_interfaces.msg import Time
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from tb4_mission.marker_detector import MarkerDetector
    import csv
    import json
    import math

    monkeypatch.setenv("ROS_DOMAIN_ID", "183")
    rclpy.init(args=["--ros-args", "-p", f"output_dir:={tmp_path}"])
    node = None
    try:
        node = MarkerDetector()
        for parent, child, translation, rotation in [
            ("odom", "camera_optical", (.2, .3, .7), (-.5, .5, -.5, .5)),
            ("odom", "laser", (0, 0, .2), (0, 0, 0, 1)),
            ("odom", "base_link", (0, 0, 0), (0, 0, 0, 1)),
            ("map", "odom", (0, 0, 0), (0, 0, 0, 1)),
        ]:
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            transform.transform.rotation.x = float(rotation[0])
            transform.transform.rotation.y = float(rotation[1])
            transform.transform.rotation.z = float(rotation[2])
            transform.transform.rotation.w = float(rotation[3])
            node.tf.set_transform_static(transform, "test")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 817, 140)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 817, 140)
        pixels = np.full((240, 240), 255, dtype=np.uint8)
        pixels[50:190, 50:190] = marker
        info = CameraInfo()
        info.header.frame_id = "camera_optical"
        info.width = info.height = 240
        info.k = [220., 0., 119.5, 0., 220., 119.5, 0., 0., 1.]
        info.d = [0.]*5
        info.distortion_model = "plumb_bob"
        node.on_info(info)
        now = node.get_clock().now().nanoseconds/1e9
        def feed(age, wall_distance):
            stamp = now-age
            timestamp = Time(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9))
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = "laser", timestamp
            scan.angle_min, scan.angle_increment = -.5, .01
            scan.range_min, scan.range_max = .1, 10.
            scan.ranges = [wall_distance/math.cos(-.5+i*.01) for i in range(101)]
            node.scans.append(scan)
            image = node.bridge.cv2_to_imgmsg(pixels, encoding="mono8")
            image.header.frame_id, image.header.stamp = "camera_optical", timestamp
            node.on_image(image)
            node.process_image()
        feed(1.1, 3.0)
        assert node.store.tracks == {}  # Seeing an ID beyond 2 m does not count as approached.
        for age in (.8, .55, .3):
            feed(age, 1.5)
        node.publish()
        with (tmp_path/"targets.csv").open() as stream:
            targets = list(csv.DictReader(stream))
        assert len(targets) == 1
        assert targets[0]["id"] == "817"
        assert (float(targets[0]["x"]), float(targets[0]["y"])) == pytest.approx((1.5, .3), abs=.01)
        assert node.store.tracks[817].observations == 3
        node.on_state(String(data='{"state":"stopping"}'))
        assert not node.tracking_active
        correction = TransformStamped()
        correction.header.frame_id, correction.child_frame_id = "map", "odom"
        correction.transform.rotation.w = 1.0
        correction.transform.translation.x = .4
        node.tf.set_transform_static(correction, "test")
        node.on_state(String(data='{"state":"finished"}'))
        receipt = json.loads((tmp_path/"targets_finalized.json").read_text())
        assert receipt == {"finalized": True, "frame_id": "map", "target_ids": [817], "csv": "targets.csv"}
        with (tmp_path/"targets.csv").open() as stream:
            frozen = list(csv.DictReader(stream))
        assert float(frozen[0]["x"]) == pytest.approx(1.9, abs=.01)
        correction.transform.translation.x = 1.0
        node.tf.set_transform_static(correction, "test")
        node.publish()
        with (tmp_path/"targets.csv").open() as stream:
            assert list(csv.DictReader(stream)) == frozen
        response = node.reset_targets(Trigger.Request(), Trigger.Response())
        assert response.success
        assert (tmp_path/"targets.csv").read_text() == "id,x,y\n"
        assert not (tmp_path/"targets_finalized.json").exists()
        node.on_state(String(data='{"state":"finished"}'))
        assert json.loads((tmp_path/"targets_finalized.json").read_text())["target_ids"] == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
