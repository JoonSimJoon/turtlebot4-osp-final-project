"""Read-only live checks for the laptop-to-TurtleBot4 ROS graph."""
import argparse
import math
import os
import sys
import time


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from nav_msgs.msg import OccupancyGrid, Odometry
    from tf2_ros import Buffer, TransformListener, TransformException

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=12.0)
    parser.add_argument('--camera', action='store_true')
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--scan-topic', default=os.getenv('SCAN_TOPIC', 'scan'))
    parser.add_argument('--image-topic', default=os.getenv('IMAGE_TOPIC', 'oakd/rgb/preview/image_raw'))
    parser.add_argument('--camera-info-topic', default=os.getenv('CAMERA_INFO_TOPIC', 'oakd/rgb/preview/camera_info'))
    parser.add_argument('--base-frame', default=os.getenv('BASE_FRAME', 'base_link'))
    parser.add_argument('--odom-frame', default=os.getenv('ODOM_FRAME', 'odom'))
    parser.add_argument('--map-frame', default=os.getenv('MAP_FRAME', 'map'))
    options, ros_args = parser.parse_known_args(args)
    if options.timeout <= 0 or not math.isfinite(options.timeout):
        parser.error('--timeout must be a finite positive number')
    rclpy.init(args=ros_args)

    class Doctor(Node):
        def __init__(self):
            super().__init__('tb4_connection_doctor')
            self.received = {}
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.subscriptions_keep = []
            streams = [('scan', LaserScan, options.scan_topic), ('odom', Odometry, 'odom')]
            if options.camera:
                streams += [('image', Image, options.image_topic), ('camera_info', CameraInfo, options.camera_info_topic)]
            for key, kind, topic in streams:
                self.subscriptions_keep.append(self.create_subscription(
                    kind, topic, lambda msg, k=key: self.received.update({k: (time.monotonic(), msg)}), qos_profile_sensor_data))
            if options.navigation:
                self.subscriptions_keep.append(self.create_subscription(
                    OccupancyGrid, 'map', lambda msg: self.received.update({'map': (time.monotonic(), msg)}),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            self.lifecycle_checks = {}
            if options.navigation:
                from lifecycle_msgs.srv import GetState
                for name in ('bt_navigator', 'behavior_server'):
                    self.lifecycle_checks[name] = [self.create_client(GetState, name + '/get_state'), None, False]
                from rclpy.action import ActionClient
                from nav2_msgs.action import NavigateToPose, Spin
                self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
                self.spin_client = ActionClient(self, Spin, 'spin')

        def checks(self):
            now = time.monotonic()
            checks = {}
            required = ['scan', 'odom'] + (['image', 'camera_info'] if options.camera else [])
            for key in required:
                item = self.received.get(key)
                # CameraInfo can legitimately publish once with a reliable stream.
                checks[key + ' 수신'] = bool(item and (key == 'camera_info' or now - item[0] < 3.0))
                if item and key != 'camera_info':
                    stamp = item[1].header.stamp
                    age = self.get_clock().now().nanoseconds * 1e-9 - stamp.sec - stamp.nanosec * 1e-9
                    checks[key + ' 시간 동기화 (<3s)'] = -1.0 <= age <= 3.0
            if 'scan' in self.received:
                msg = self.received['scan'][1]
                checks['유효 LiDAR 거리'] = any(math.isfinite(x) and msg.range_min <= x <= msg.range_max for x in msg.ranges)
            if 'camera_info' in self.received:
                msg = self.received['camera_info'][1]
                checks['카메라 보정값'] = bool(msg.width and msg.height and msg.k[0] > 0 and msg.k[4] > 0)
                if 'image' in self.received:
                    image = self.received['image'][1]
                    checks['영상·보정 해상도 일치'] = image.width == msg.width and image.height == msg.height
            pairs = [(options.odom_frame, options.base_frame)]
            if 'scan' in self.received:
                pairs.append((options.base_frame, self.received['scan'][1].header.frame_id))
            if options.camera and 'image' in self.received:
                pairs.append((options.base_frame, self.received['image'][1].header.frame_id))
            if options.navigation:
                pairs.append((options.map_frame, options.base_frame))
                checks['map 수신'] = 'map' in self.received and bool(self.received['map'][1].data)
                checks['NavigateToPose action'] = self.nav_client.server_is_ready()
                checks['Spin action'] = self.spin_client.server_is_ready()
                from lifecycle_msgs.srv import GetState
                for name, entry in self.lifecycle_checks.items():
                    client, future, active = entry
                    if future is not None and future.done():
                        try:
                            entry[2] = future.result().current_state.id == 3
                        except Exception:
                            entry[2] = False
                        entry[1] = None
                    if entry[1] is None and client.service_is_ready():
                        entry[1] = client.call_async(GetState.Request())
                    checks[name + ' ACTIVE'] = entry[2]
            for target, source in pairs:
                try:
                    if not source:
                        raise ValueError('empty source frame')
                    self.buffer.lookup_transform(target, source, Time())
                    checks[f'TF {target} <- {source}'] = True
                except (TransformException, ValueError):
                    checks[f'TF {target} <- {source or "(empty)"}'] = False
            return checks

    node = Doctor()
    result = {}
    try:
        print('ROS_DOMAIN_ID=' + os.getenv('ROS_DOMAIN_ID', '0'))
        print('RMW_IMPLEMENTATION=' + os.getenv('RMW_IMPLEMENTATION', '(ROS default)'))
        print('ROS_DISCOVERY_SERVER=' + os.getenv('ROS_DISCOVERY_SERVER', '(simple discovery)'))
        deadline = time.monotonic() + options.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            result = node.checks()
            if result and all(result.values()):
                break
        for label, ok in result.items():
            print(f'[{"OK" if ok else "FAIL"}] {label}')
        success = bool(result) and all(result.values())
        if not success:
            print('연결 미완료: LAN 주소/ROS_DOMAIN_ID/DDS 설정, 로봇 bringup, 토픽 이름, 양쪽 시계를 확인하세요.')
            if options.camera:
                print('도킹 중 OAK 카메라가 꺼질 수 있습니다. 시작 전에 로봇을 출발 위치에 두세요.')
        print('이 검사는 이동 명령을 보내지 않습니다.')
        return 0 if success else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
