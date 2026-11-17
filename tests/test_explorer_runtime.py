"""ROS integration with synthetic sensors and fake Nav2 action servers.

These tests verify transport, lifecycle gating and mission orchestration. They
do not test Nav2 planning/controller performance or a physical TurtleBot4.
Run with TB4_RUN_ROS_TESTS=1 in a ROS Jazzy Python environment.
"""

import json
import os
from pathlib import Path
import time

import pytest

if os.environ.get('TB4_RUN_ROS_TESTS') != '1':
    pytest.skip('Set TB4_RUN_ROS_TESTS=1 in a ROS environment', allow_module_level=True)
os.environ.setdefault('ROS_DOMAIN_ID', '88')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from tb4_mission.explorer import Explorer


class RobotDouble(Node):
    def __init__(self, acceptance_delay=0.0):
        super().__init__('explorer_runtime_double')
        self.enabled = False
        self.perception_enabled = True
        self.complete_spins = False
        self.fully_known = False
        self.last_navigation_goal = None
        self.acceptance_delay = acceptance_delay
        self.reset_count = self.accepted = self.cancelled = self.completed = 0
        self.group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.perception_pub = self.create_publisher(Bool, 'mission/perception_ready', 10)
        self.target_pub = self.create_publisher(String, 'mission/targets', latched)
        self.tf = TransformBroadcaster(self)
        self.create_service(GetState, 'bt_navigator/get_state', self.lifecycle,
                            callback_group=self.group)
        self.create_service(GetState, 'behavior_server/get_state', self.lifecycle,
                            callback_group=self.group)
        self.create_service(Trigger, 'mission/reset_targets', self.reset,
                            callback_group=self.group)
        self.spin_server = ActionServer(
            self, Spin, 'spin', self.spin_execute, callback_group=self.group,
            goal_callback=self.accept, cancel_callback=self.cancel)
        self.nav_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.nav_execute,
            callback_group=self.group, goal_callback=self.accept, cancel_callback=self.cancel)
        self.create_timer(0.05, self.publish, callback_group=self.group)

    def lifecycle(self, _request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = 'active'
        return response

    def reset(self, _request, response):
        self.reset_count += 1
        empty = String()
        empty.data = '{"targets":[]}'
        self.target_pub.publish(empty)
        response.success, response.message = True, 'reset'
        return response

    def accept(self, _goal):
        self.accepted += 1
        time.sleep(self.acceptance_delay)
        return GoalResponse.ACCEPT

    def cancel(self, _handle):
        self.cancelled += 1
        return CancelResponse.ACCEPT

    def execute(self, handle, action_type):
        deadline = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                self.completed += 1
                return action_type.Result()
            time.sleep(0.01)
        handle.abort()
        self.completed += 1
        return action_type.Result()

    def spin_execute(self, handle):
        if self.complete_spins:
            handle.succeed()
            self.completed += 1
            return Spin.Result()
        return self.execute(handle, Spin)

    def nav_execute(self, handle):
        self.last_navigation_goal = handle.request.pose
        return self.execute(handle, NavigateToPose)

    def publish(self):
        if not self.enabled:
            return
        stamp = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = 'map', stamp
        grid.info.width, grid.info.height, grid.info.resolution = 40, 30, 0.1
        grid.info.origin.position.x = grid.info.origin.position.y = -1.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0 if self.fully_known or index % 40 < 30 else -1
                     for index in range(1200)]
        self.map_pub.publish(grid)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', stamp
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.angle_min, scan.angle_max, scan.angle_increment = -3.0, 3.0, 0.1
        scan.ranges = [2.0] * 61
        self.scan_pub.publish(scan)
        ready = Bool()
        ready.data = self.perception_enabled
        self.perception_pub.publish(ready)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        transform.header.stamp = stamp
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

    def send_targets(self):
        message = String()
        message.data = json.dumps({'targets': [
            {'id': marker_id, 'x': marker_id * 0.1, 'y': 0.5, 'observations': 3}
            for marker_id in (7, 18, 41)]})
        self.target_pub.publish(message)

    def destroy_node(self):
        self.spin_server.destroy()
        self.nav_server.destroy()
        super().destroy_node()


def spin_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError('Timed out waiting for ROS integration condition')


@pytest.fixture
def runtime(tmp_path):
    objects = []

    def create(*, limit=2.0, acceptance_delay=0.0):
        rclpy.init(args=['--ros-args', '-p', f'output_dir:={tmp_path}',
                        '-p', f'time_limit_sec:={float(limit)}',
                        '-p', 'cancel_timeout_sec:=3.0'])
        explorer, robot = Explorer(), RobotDouble(acceptance_delay)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(explorer)
        executor.add_node(robot)
        objects.extend((explorer, robot, executor))
        return explorer, robot, executor, tmp_path

    yield create
    if objects:
        explorer, robot, executor = objects
        if explorer.phase not in ('finished', 'error'):
            explorer._finish('test_cleanup')
            spin_until(executor, lambda: explorer.phase in ('finished', 'error'), timeout=4.0)
        executor.shutdown(timeout_sec=4.0)
        robot.destroy_node()
        explorer.destroy_node()
        rclpy.shutdown()


def start_when_ready(explorer, robot, executor):
    robot.enabled = True
    spin_until(executor, lambda: explorer._readiness(time.monotonic())[0])
    client = robot.create_client(Trigger, 'mission/start')
    spin_until(executor, client.service_is_ready)
    future = client.call_async(Trigger.Request())
    spin_until(executor, future.done)
    assert future.result().success
    return future.result()


def test_real_ros_readiness_time_limit_cancellation_and_map_export(runtime):
    explorer, robot, executor, output = runtime(limit=1.2)
    client = robot.create_client(Trigger, 'mission/start')
    spin_until(executor, client.service_is_ready)
    future = client.call_async(Trigger.Request())
    spin_until(executor, future.done)
    assert not future.result().success
    assert explorer.started is None and robot.accepted == 0
    start_when_ready(explorer, robot, executor)
    spin_until(executor, lambda: explorer.phase == 'finished')
    assert explorer.reason == 'time_limit'
    assert robot.reset_count == 1
    assert robot.accepted == 1 and robot.cancelled == 1 and robot.completed == 1
    assert (output / 'map.pgm').read_bytes().startswith(b'P5\n')
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['stop_confirmed'] is True and summary['reason'] == 'time_limit'
    assert (output / 'map.yaml').exists()


def test_three_targets_cancel_active_spin_before_deadline(runtime):
    explorer, robot, executor, output = runtime(limit=8.0)
    start_when_ready(explorer, robot, executor)
    spin_until(executor, lambda: explorer.goal_handle is not None)
    robot.send_targets()
    spin_until(executor, lambda: explorer.phase == 'finished')
    assert explorer.reason == 'targets_found'
    assert robot.cancelled == 1 and robot.completed == 1
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['target_ids'] == [7, 18, 41]
    assert summary['elapsed_sec'] < 8.0


def test_stop_cancels_a_goal_accepted_after_stop_request(runtime):
    explorer, robot, executor, _ = runtime(limit=8.0, acceptance_delay=0.6)
    start_when_ready(explorer, robot, executor)
    spin_until(executor, lambda: robot.accepted == 1)
    assert explorer.pending_goal is not None
    client = robot.create_client(Trigger, 'mission/stop')
    spin_until(executor, client.service_is_ready)
    future = client.call_async(Trigger.Request())
    spin_until(executor, future.done)
    assert future.result().success
    spin_until(executor, lambda: explorer.phase == 'finished')
    assert explorer.reason == 'operator_stop'
    assert robot.accepted == 1 and robot.cancelled == 1 and robot.completed == 1


def test_perception_loss_cancels_motion_and_saves_partial_map(runtime):
    explorer, robot, executor, output = runtime(limit=8.0)
    start_when_ready(explorer, robot, executor)
    spin_until(executor, lambda: explorer.goal_handle is not None)
    robot.perception_enabled = False
    spin_until(executor, lambda: explorer.phase == 'finished')
    assert explorer.reason == 'readiness_lost:camera_lidar_perception_not_ready'
    assert robot.cancelled == 1 and robot.completed == 1
    assert (output / 'map.pgm').exists()


def test_initial_spin_leads_to_navigation_for_camera_coverage_in_known_room(runtime):
    explorer, robot, executor, _ = runtime(limit=8.0)
    robot.complete_spins, robot.fully_known = True, True
    start_when_ready(explorer, robot, executor)
    spin_until(executor, lambda: robot.last_navigation_goal is not None)
    goal = robot.last_navigation_goal
    assert goal.header.frame_id == 'map'
    assert explorer.observed_positions
    assert explorer.action_kind == 'navigate'
    x, y = goal.pose.position.x, goal.pose.position.y
    assert (x * x + y * y) ** 0.5 > explorer.p['coverage_observation_radius']
    explorer._finish('test_coverage_verified')
    spin_until(executor, lambda: explorer.phase == 'finished')
    assert robot.cancelled == 1 and robot.completed == 2
