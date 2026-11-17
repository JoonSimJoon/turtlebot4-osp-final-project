"""Small pure helpers for tuning the installed Jazzy TurtleBot4 configuration."""
from copy import deepcopy
import math


def nav2_parameters(source, *, linear=0.2, angular=0.6, radius=0.2,
                    inflation=0.45, scan_topic='/scan', map_frame='map',
                    odom_frame='odom', base_frame='base_link'):
    if not all(math.isfinite(x) and x > 0 for x in (linear, angular, radius, inflation)):
        raise ValueError('speed, radius and inflation must be finite and positive')
    if linear > 0.5 or angular > 1.9 or inflation < radius:
        raise ValueError('Speed exceeds TB4 defaults or inflation radius is smaller than robot radius')
    result = deepcopy(source)
    controller = result['controller_server']['ros__parameters']['FollowPath']
    if controller.get('plugin') != 'nav2_mppi_controller::MPPIController':
        raise ValueError('Expected official Jazzy MPPI configuration; update turtlebot4_navigation')
    controller.update(vx_max=linear, vx_min=-linear / 2, vy_max=0.0,
                      wz_max=angular, ax_max=0.5, ax_min=-0.5, az_max=1.0,
                      visualize=False)
    smoother = result['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[linear, 0.0, angular],
                    min_velocity=[-linear / 2, 0.0, -angular],
                    max_accel=[0.5, 0.0, 1.0], max_decel=[-0.5, 0.0, -1.0])
    behavior = result['behavior_server']['ros__parameters']
    behavior.update(max_rotational_vel=angular, min_rotational_vel=min(0.15, angular),
                    rotational_acc_lim=1.0)
    for name in ('local_costmap', 'global_costmap'):
        params = result[name][name]['ros__parameters']
        params.pop('footprint', None)
        params['robot_radius'] = radius
        params['inflation_layer']['inflation_radius'] = inflation
        for layer_name in ('voxel_layer', 'obstacle_layer'):
            if layer_name in params:
                params[layer_name]['scan']['topic'] = scan_topic
    monitor = result['collision_monitor']['ros__parameters']
    for source_name in monitor.get('observation_sources', ['scan']):
        if source_name in monitor and monitor[source_name].get('type') == 'scan':
            monitor[source_name]['topic'] = scan_topic
    result['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    replace_frames(result, map_frame=map_frame, odom_frame=odom_frame, base_frame=base_frame)
    return result


def replace_frames(tree, *, map_frame, odom_frame, base_frame):
    values = {'map': map_frame, 'odom': odom_frame, 'base_link': base_frame}
    frame_keys = {'global_frame', 'local_frame', 'robot_base_frame', 'base_frame_id',
                  'odom_frame', 'odom_frame_id', 'base_frame', 'map_frame', 'global_frame_id', 'fixed_frame'}
    if isinstance(tree, dict):
        for key, value in tree.items():
            if key in frame_keys and isinstance(value, str) and value in values:
                tree[key] = values[value]
            elif isinstance(value, (dict, list)):
                replace_frames(value, map_frame=map_frame, odom_frame=odom_frame, base_frame=base_frame)
    elif isinstance(tree, list):
        for item in tree:
            replace_frames(item, map_frame=map_frame, odom_frame=odom_frame, base_frame=base_frame)
