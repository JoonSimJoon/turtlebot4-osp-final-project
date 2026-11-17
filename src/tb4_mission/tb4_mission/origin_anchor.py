"""Put fresh SLAM's odometry origin at the initial robot pose without resetting hardware.

slam_toolbox's map_start_pose only applies when loading a saved pose graph:
https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp
The static chain is map -> mission_odom -> odom -> base_link. Nav2 may keep using
hardware odom for its local costmap; TF resolves the entire chain.
"""
import math
import time


def inverse_planar_pose(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def main(args=None):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

    class OriginAnchor(Node):
        def __init__(self):
            super().__init__('origin_anchor')
            self.declare_parameter('odom_frame', 'odom')
            self.declare_parameter('base_frame', 'base_link')
            self.declare_parameter('anchor_frame', 'mission_odom')
            self.declare_parameter('timeout_sec', 30.0)
            self.odom = self.get_parameter('odom_frame').value
            self.base = self.get_parameter('base_frame').value
            self.anchor = self.get_parameter('anchor_frame').value
            if len({self.odom, self.base, self.anchor}) != 3:
                raise ValueError('odom_frame, base_frame, anchor_frame must be distinct')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.broadcaster = StaticTransformBroadcaster(self)
            self.started = time.monotonic()
            self.anchored = False
            self.timer = self.create_timer(0.1, self.capture)

        def capture(self):
            if self.anchored:
                return
            if time.monotonic() - self.started > self.get_parameter('timeout_sec').value:
                raise RuntimeError(f'No fresh TF {self.odom} -> {self.base}; cannot anchor SLAM')
            try:
                transform = self.buffer.lookup_transform(self.odom, self.base, Time())
            except TransformException:
                return
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if not -0.5 <= age <= 2.0:
                return
            p, q = transform.transform.translation, transform.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            x, y, angle = inverse_planar_pose(p.x, p.y, yaw)
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.anchor
            msg.child_frame_id = self.odom
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = -p.z
            msg.transform.rotation.z = math.sin(angle / 2)
            msg.transform.rotation.w = math.cos(angle / 2)
            self.broadcaster.sendTransform(msg)
            self.anchored = True
            self.get_logger().info(f'Start anchored: hardware odom pose ({p.x:.3f}, {p.y:.3f}, {yaw:.3f}); do not relocate robot')

    rclpy.init(args=args)
    node = None
    try:
        node = OriginAnchor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
