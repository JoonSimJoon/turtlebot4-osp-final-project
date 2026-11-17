"""TurtleBot4 mission orchestration through Nav2's collision-checked actions."""

import json
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import GridMap, find_coverage_goal, find_frontiers
from .map_io import save_map, write_json
from .mission_logic import GoalBlacklist, confirmed_target_ids, terminal_reason


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        defaults = {
            'output_dir': str(Path.home() / 'tb4_results'),
            'map_frame': 'map', 'base_frame': 'base_link',
            'map_topic': 'map', 'scan_topic': 'scan',
            'autostart': False, 'time_limit_sec': 600.0, 'target_count': 3,
            'robot_radius': 0.22, 'frontier_min_cells': 4,
            'frontier_standoff': 0.35, 'min_goal_distance': 0.35,
            'max_search_cells': 200000, 'planning_interval_sec': 2.0,
            'goal_timeout_sec': 70.0, 'progress_timeout_sec': 25.0,
            'scan_timeout_sec': 3.0, 'map_timeout_sec': 15.0,
            'perception_timeout_sec': 3.0, 'tf_timeout_sec': 3.0,
            'cancel_timeout_sec': 8.0, 'goal_response_timeout_sec': 10.0,
            'blacklist_ttl_sec': 60.0, 'blacklist_radius': 0.65,
            'spin_angle': 2.0 * math.pi, 'spin_timeout_sec': 40.0,
            'max_empty_spins': 2, 'scan_after_goal': True,
            'coverage_spacing': 1.0, 'coverage_observation_radius': 1.2,
            'navigator_lifecycle_service': 'bt_navigator/get_state',
            'behavior_lifecycle_service': 'behavior_server/get_state',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('time_limit_sec', 'target_count', 'robot_radius',
                    'goal_timeout_sec', 'progress_timeout_sec', 'scan_timeout_sec',
                    'map_timeout_sec', 'perception_timeout_sec', 'cancel_timeout_sec',
                    'spin_timeout_sec', 'max_search_cells', 'planning_interval_sec',
                    'coverage_spacing', 'coverage_observation_radius'):
            if self.p[key] <= 0:
                raise ValueError(f'{key} must be positive')
        self.output_dir = Path(self.p['output_dir']).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'mission/state', latched)
        self.create_subscription(OccupancyGrid, self.p['map_topic'], self._map, latched)
        self.create_subscription(LaserScan, self.p['scan_topic'], self._scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, 'mission/targets', self._targets, latched)
        self.create_subscription(Bool, 'mission/perception_ready', self._perception, 10)
        self.create_service(Trigger, 'mission/start', self._start_service)
        self.create_service(Trigger, 'mission/stop', self._stop_service)
        self.reset_client = self.create_client(Trigger, 'mission/reset_targets')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.lifecycle_clients = [self.create_client(GetState, self.p[key]) for key in
                                  ('navigator_lifecycle_service', 'behavior_lifecycle_service')]
        self.lifecycle_active = [False, False]
        self.lifecycle_updated = [0.0, 0.0]
        self.lifecycle_futures = [None, None]
        self.tf = Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.grid = None
        self.map_seen = self.scan_seen = self.perception_seen = 0.0
        self.perception_ready = False
        self.phase, self.reason = 'waiting', 'waiting_for_sensors_and_nav2'
        self.target_ids = set()
        self.started = None
        self.finished_at = None
        self.saved = False
        self.goal_handle = self.pending_goal = self.reset_future = None
        self.action_kind = None
        self.action_goal = None
        self.action_token = 0
        self.action_started = self.last_progress = self.cancel_started = 0.0
        self.best_remaining = math.inf
        self.cancel_requested = False
        self.cancel_future = None
        self.action_cancel_reason = None
        self.next_plan = self.last_lifecycle_poll = self.last_state = 0.0
        self.empty_spins = 0
        self.observed_positions = []
        self.need_spin = True
        self.blacklist = GoalBlacklist(self.p['blacklist_ttl_sec'], self.p['blacklist_radius'])
        self.last_readiness = 'No map yet'
        # A steady timer keeps the 600-second limit and sensor watchdog alive even
        # if simulated /clock is paused or a bag stops publishing.
        self.timer = self.create_timer(0.2, self._tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Mission waiting. Use mission/start, or enable autostart.')

    def _map(self, msg):
        if msg.header.frame_id.lstrip('/') != self.p['map_frame'].lstrip('/'):
            return
        q, origin = msg.info.origin.orientation, msg.info.origin.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            self.grid = GridMap(msg.info.width, msg.info.height, msg.info.resolution,
                                origin.x, origin.y, yaw, tuple(msg.data))
            self.map_seen = time.monotonic()
        except ValueError as error:
            self.get_logger().warning(f'Ignoring invalid map: {error}')

    def _scan(self, msg):
        if msg.ranges and msg.header.frame_id and msg.range_max > msg.range_min:
            self.scan_seen = time.monotonic()

    def _targets(self, msg):
        if self.phase != 'running':
            return
        try:
            self.target_ids = confirmed_target_ids(json.loads(msg.data))
        except (ValueError, TypeError):
            self.get_logger().warning('Ignoring invalid mission/targets message')

    def _perception(self, msg):
        self.perception_seen, self.perception_ready = time.monotonic(), bool(msg.data)

    def _poll_lifecycle(self, now):
        if now - self.last_lifecycle_poll < 1.0:
            return
        self.last_lifecycle_poll = now
        for index, client in enumerate(self.lifecycle_clients):
            pending = self.lifecycle_futures[index]
            if pending is not None and not pending.done():
                continue
            if not client.service_is_ready():
                self.lifecycle_active[index] = False
                continue
            future = client.call_async(GetState.Request())
            self.lifecycle_futures[index] = future
            future.add_done_callback(lambda f, i=index: self._lifecycle_result(f, i))

    def _lifecycle_result(self, future, index):
        try:
            self.lifecycle_active[index] = (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE)
            self.lifecycle_updated[index] = time.monotonic()
        except Exception as error:
            self.lifecycle_active[index] = False
            self.get_logger().warning(f'Nav2 lifecycle query failed: {error}')

    def _robot_pose(self):
        transform = self.tf.lookup_transform(self.p['map_frame'], self.p['base_frame'], Time())
        stamp = Time.from_msg(transform.header.stamp)
        # A zero stamp denotes a static transform. Live map/base must not be static.
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if stamp.nanoseconds == 0 or age < -0.2 or age > self.p['tf_timeout_sec']:
            raise ValueError(f'map -> base TF is stale ({age:.1f}s)')
        translation, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return translation.x, translation.y, yaw

    def _readiness(self, now):
        if self.grid is None or now - self.map_seen > self.p['map_timeout_sec']:
            return False, 'map_missing_or_stale'
        if now - self.scan_seen > self.p['scan_timeout_sec']:
            return False, 'scan_missing_or_stale'
        if not self.perception_ready or now - self.perception_seen > self.p['perception_timeout_sec']:
            return False, 'camera_lidar_perception_not_ready'
        if not all(self.lifecycle_active) or any(now - updated > 4.0 for updated in self.lifecycle_updated):
            return False, 'nav2_lifecycle_not_active'
        if not self.nav.server_is_ready() or not self.spin.server_is_ready():
            return False, 'nav2_action_servers_not_ready'
        if not self.reset_client.service_is_ready():
            return False, 'marker_reset_service_not_ready'
        try:
            self._robot_pose()
        except (TransformException, ValueError) as error:
            return False, f'tf_not_ready: {error}'
        return True, 'ready'

    def _start_service(self, _request, response):
        response.success, response.message = self._start()
        return response

    def _start(self):
        if self.phase != 'waiting':
            return False, 'A mission already started. Relaunch for a fresh map and output directory.'
        ready, detail = self._readiness(time.monotonic())
        if not ready:
            return False, detail
        self.started = time.monotonic()
        self.phase, self.reason = 'starting', 'resetting_targets'
        self.target_ids.clear()
        self.reset_future = self.reset_client.call_async(Trigger.Request())
        self.reset_future.add_done_callback(self._reset_done)
        self._publish_state()
        return True, 'Start accepted; clearing targets before autonomous motion.'

    def _reset_done(self, future):
        if self.phase != 'starting':
            return
        try:
            result = future.result()
            if not result.success:
                self._finish('target_reset_failed: ' + result.message)
                return
        except Exception as error:
            self._finish(f'target_reset_failed: {error}')
            return
        self.phase, self.reason = 'running', 'initial_observation'
        self.target_ids.clear()
        self.get_logger().info('Mission started; Nav2 controls all movement.')

    def _stop_service(self, _request, response):
        if self.phase in ('finished', 'error'):
            response.success, response.message = True, self.reason
        else:
            self._finish('operator_stop')
            response.success = True
            response.message = 'Cancellation requested; check mission/state for terminal confirmation.'
        return response

    def _tick(self):
        now = time.monotonic()
        self._poll_lifecycle(now)
        if now - self.last_state >= 1.0:
            self._publish_state()
            self.last_state = now
        if self.phase == 'waiting':
            ready, self.last_readiness = self._readiness(now)
            if self.p['autostart'] and ready:
                self._start()
            return
        if self.phase in ('starting', 'running'):
            reason = terminal_reason(now - self.started, self.target_ids,
                                     self.p['time_limit_sec'], self.p['target_count'])
            if reason:
                self._finish(reason)
                return
        if self.phase == 'starting':
            if now - self.started > 5.0:
                self._finish('target_reset_timeout')
            return
        if self.phase == 'stopping':
            if self.pending_goal is None and self.goal_handle is None:
                self._save_and_complete()
            elif now - self.cancel_started > self.p['cancel_timeout_sec']:
                self.phase = 'error'
                self.reason += ':cancel_unconfirmed'
                self.get_logger().error('Nav2 did not confirm stop. Check robot and use its emergency stop if needed.')
                self._save_and_complete(error=True)
            return
        if self.phase != 'running':
            return
        ready, detail = self._readiness(now)
        if not ready:
            self._finish('readiness_lost:' + detail)
            return
        if self.pending_goal is not None or self.goal_handle is not None:
            if self.cancel_requested:
                if now - self.cancel_started > self.p['cancel_timeout_sec']:
                    self._finish('action_cancel_timeout')
                return
            if self.pending_goal is not None and now - self.action_started > self.p['goal_response_timeout_sec']:
                self._finish('goal_acceptance_timeout')
            elif now - self.action_started > (self.p['spin_timeout_sec'] + 5.0
                                             if self.action_kind == 'spin' else self.p['goal_timeout_sec']):
                self._cancel_action('goal_timeout')
            elif self.action_kind == 'navigate' and now - self.last_progress > self.p['progress_timeout_sec']:
                self._cancel_action('no_progress')
            return
        if now < self.next_plan:
            return
        self.next_plan = now + self.p['planning_interval_sec']
        if self.need_spin:
            self.need_spin = False
            self._send_spin()
            return
        try:
            x, y, _ = self._robot_pose()
            goals = find_frontiers(
                self.grid, x, y, robot_radius=self.p['robot_radius'],
                min_cluster_cells=self.p['frontier_min_cells'],
                standoff=self.p['frontier_standoff'],
                min_goal_distance=self.p['min_goal_distance'],
                max_cells=self.p['max_search_cells'], blacklist=self.blacklist.active(now))
        except (TransformException, ValueError) as error:
            self._finish(f'planning_failed:{error}')
            return
        if goals:
            self.empty_spins = 0
            self._send_navigation(goals[0])
        else:
            coverage = find_coverage_goal(
                self.grid, x, y, observed=self.observed_positions,
                robot_radius=self.p['robot_radius'], spacing=self.p['coverage_spacing'],
                observation_radius=self.p['coverage_observation_radius'],
                min_goal_distance=self.p['min_goal_distance'],
                max_cells=self.p['max_search_cells'], blacklist=self.blacklist.active(now))
            if coverage is not None:
                self.empty_spins = 0
                self._send_navigation(coverage)
            elif self.empty_spins < self.p['max_empty_spins']:
                self.empty_spins += 1
                self._send_spin()
            else:
                self._finish('coverage_complete')

    def _dispatch(self, client, goal, kind, position=None):
        if self.phase != 'running' or self.pending_goal is not None or self.goal_handle is not None:
            return
        self.action_token += 1
        token = self.action_token
        self.action_kind, self.action_goal = kind, position
        self.action_started = self.last_progress = time.monotonic()
        self.best_remaining = math.inf
        self.cancel_requested, self.action_cancel_reason = False, None
        self.reason = kind
        try:
            self.pending_goal = client.send_goal_async(
                goal, feedback_callback=lambda feedback: self._feedback(feedback, token))
            self.pending_goal.add_done_callback(lambda future: self._goal_response(future, token))
        except Exception as error:
            self.pending_goal = None
            self._finish(f'goal_send_failed:{error}')

    def _send_navigation(self, frontier):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.p['map_frame']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = frontier.x, frontier.y
        goal.pose.pose.orientation.z = math.sin(frontier.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(frontier.yaw / 2.0)
        self._dispatch(self.nav, goal, 'navigate', (frontier.x, frontier.y))

    def _send_spin(self):
        goal = Spin.Goal()
        goal.target_yaw = float(self.p['spin_angle'])
        goal.time_allowance = Duration(sec=int(self.p['spin_timeout_sec']))
        self._dispatch(self.spin, goal, 'spin')

    def _feedback(self, message, token):
        if token != self.action_token or self.action_kind != 'navigate':
            return
        remaining = message.feedback.distance_remaining
        if math.isfinite(remaining) and remaining < self.best_remaining - 0.05:
            self.best_remaining, self.last_progress = remaining, time.monotonic()

    def _goal_response(self, future, token):
        if token != self.action_token:
            # Never leave a delayed accepted action running after a newer session.
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
            return
        self.pending_goal = None
        try:
            handle = future.result()
        except Exception as error:
            self._finish(f'goal_response_failed:{error}')
            return
        if not handle.accepted:
            self._action_complete(False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._action_result(f, token))
        if self.phase != 'running' or self.cancel_requested:
            self._request_cancel()

    def _action_result(self, future, token):
        if token != self.action_token:
            return
        try:
            result = future.result()
            success = (result.status == GoalStatus.STATUS_SUCCEEDED and
                       getattr(result.result, 'error_code', 0) == 0)
        except Exception as error:
            self.get_logger().warning(f'Action result unavailable: {error}')
            self._finish('action_result_unavailable')
            return
        self.goal_handle = None
        self._action_complete(success)

    def _action_complete(self, success):
        if not success and self.action_goal is not None:
            self.blacklist.add(*self.action_goal, time.monotonic())
        if self.phase == 'running' and self.action_kind == 'navigate' and success:
            self.need_spin = self.p['scan_after_goal']
        if self.phase == 'running' and self.action_kind == 'spin' and success:
            try:
                x, y, _ = self._robot_pose()
                self.observed_positions.append((x, y))
            except (TransformException, ValueError):
                pass
        self.action_kind = self.action_goal = None
        self.cancel_requested = False
        self.cancel_future = None
        self.next_plan = time.monotonic() + self.p['planning_interval_sec']
        if self.phase == 'stopping':
            self._save_and_complete()

    def _cancel_action(self, reason):
        if self.cancel_requested:
            return
        self.cancel_requested, self.action_cancel_reason = True, reason
        self.cancel_started = time.monotonic()
        self._request_cancel()

    def _request_cancel(self):
        if self.goal_handle is not None and self.cancel_future is None:
            try:
                self.cancel_future = self.goal_handle.cancel_goal_async()
                self.cancel_future.add_done_callback(self._cancel_response)
            except Exception as error:
                self.get_logger().error(f'Cancellation send failed: {error}')

    def _cancel_response(self, future):
        try:
            result = future.result()
            if not result.goals_canceling:
                self.get_logger().warning('Nav2 cancellation not acknowledged; waiting for terminal result.')
        except Exception as error:
            self.get_logger().warning(f'Cancellation response failed: {error}')
        # An acknowledgement is not termination. Keep the handle until its result.

    def _finish(self, reason):
        if self.phase in ('stopping', 'finished', 'error'):
            return
        self.phase, self.reason = 'stopping', reason
        self.cancel_started = time.monotonic()
        self.cancel_requested = True
        self._request_cancel()
        self._publish_state()
        if self.pending_goal is None and self.goal_handle is None:
            self._save_and_complete()

    def _save_and_complete(self, error=False):
        if self.saved:
            return
        self.saved = True
        self.finished_at = time.monotonic()
        exported, failures = [], []
        if self.grid is not None:
            try:
                exported = [str(path) for path in save_map(self.grid, self.output_dir)]
            except (OSError, ValueError) as exception:
                failures.append(str(exception))
        else:
            failures.append('No map was received; map files could not be saved.')
        self.phase = 'error' if error or failures else 'finished'
        summary = self._state()
        summary.update(map_files=exported, export_errors=failures,
                       stop_confirmed=not error and self.pending_goal is None and self.goal_handle is None)
        try:
            write_json(self.output_dir / 'summary.json', summary)
        except (OSError, ValueError) as exception:
            self.phase = 'error'
            self.get_logger().error(f'Could not save summary: {exception}')
        self.get_logger().info(f'Mission {self.phase}: {self.reason}; output {self.output_dir}')
        for failure in failures:
            self.get_logger().error(failure)
        self._publish_state()

    def _state(self):
        return {
            'state': self.phase, 'reason': self.reason,
            'elapsed_sec': 0.0 if self.started is None else round(
                (self.finished_at if self.finished_at is not None else time.monotonic()) - self.started, 2),
            'time_limit_sec': self.p['time_limit_sec'],
            'target_ids': sorted(self.target_ids), 'target_count': len(self.target_ids),
            'target_goal': self.p['target_count'], 'readiness': self.last_readiness,
            'output_dir': str(self.output_dir), 'action': self.action_kind,
        }

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps(self._state(), ensure_ascii=False)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Explorer()
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # CLI normally calls mission/stop before terminating the launch group.
        node.get_logger().warning('Interrupted. Requesting Nav2 cancellation before exit.')
        if rclpy.ok():
            node._finish('shutdown')
            deadline = time.monotonic() + node.p['cancel_timeout_sec']
            while rclpy.ok() and node.phase == 'stopping' and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        if not node.saved:
            node.reason = 'shutdown:cancel_unconfirmed'
            node._save_and_complete(error=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
