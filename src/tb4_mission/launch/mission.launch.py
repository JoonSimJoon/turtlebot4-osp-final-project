"""TurtleBot4 Jazzy: installed vendor stack plus assignment mission nodes."""
from datetime import datetime
import json
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from tb4_mission.launch_config import nav2_parameters, replace_frames


def include_vendor(vendor, name, arguments):
    # The mission's autostart controls exploration only. Each vendor lifecycle stack
    # must activate independently, including when mission autostart is false.
    vendor_arguments = dict(arguments, autostart='true')
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor / 'launch' / name)),
                                 launch_arguments=vendor_arguments.items()),
    ])


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise RuntimeError('This project requires ROS 2 Jazzy (Ubuntu 24.04)')
    mode = value('mode')
    simulated = value('use_sim_time') == 'true'
    package = Path(get_package_share_directory('tb4_mission'))
    vendor = Path(get_package_share_directory('turtlebot4_navigation'))
    output = Path(value('output_dir')).expanduser().resolve()
    config_dir = output / 'config'
    config_dir.mkdir(parents=True, exist_ok=True)
    frames = dict(map_frame=value('map_frame'), odom_frame=value('odom_frame'), base_frame=value('base_frame'))
    scan = '/' + value('scan_topic').lstrip('/')
    shared = dict(use_sim_time=simulated, output_dir=str(output), scan_topic=scan, **frames)
    actions = []

    def read_config(name):
        with (vendor / 'config' / name).open() as stream:
            return yaml.safe_load(stream)

    def save_config(name, data):
        target = config_dir / name
        target.write_text(yaml.safe_dump(data, sort_keys=False))
        return str(target)

    def include(name, arguments):
        return include_vendor(vendor, name, arguments)

    if mode in ('slam', 'explore'):
        slam = read_config('slam.yaml')
        params = slam['slam_toolbox']['ros__parameters']
        params.update(mode='mapping', odom_frame=value('slam_odom_frame'),
                      map_frame=frames['map_frame'], base_frame=frames['base_frame'],
                      scan_topic=scan, map_file_name='', map_start_pose=[0.0, 0.0, 0.0],
                      map_update_interval=1.0, resolution=0.05,
                      minimum_travel_distance=0.10, minimum_travel_heading=0.10,
                      enable_interactive_mode=False)
        actions.extend([
            Node(package='tb4_mission', executable='origin_anchor', name='origin_anchor',
                 parameters=[{'use_sim_time': simulated, 'odom_frame': frames['odom_frame'],
                              'base_frame': frames['base_frame'], 'anchor_frame': value('slam_odom_frame')}], output='screen'),
            GroupAction([SetRemap(src='/scan', dst=scan),
                         include('slam.launch.py', {'sync': 'false', 'params': save_config('slam.yaml', slam),
                                                   'use_sim_time': value('use_sim_time')})]),
        ])
    else:
        map_path = Path(value('map')).expanduser()
        if not map_path.is_absolute() or not map_path.is_file():
            raise ValueError('navigate requires map:=/absolute/path/to/map.yaml')
        localization = read_config('localization.yaml')
        replace_frames(localization, **frames)
        localization['amcl']['ros__parameters']['scan_topic'] = scan
        localization.setdefault('map_server', {}).setdefault('ros__parameters', {})['frame_id'] = frames['map_frame']
        actions.append(include('localization.launch.py', {'map': str(map_path),
                       'params': save_config('localization.yaml', localization), 'use_sim_time': value('use_sim_time')}))

    if mode in ('explore', 'navigate'):
        nav = nav2_parameters(read_config('nav2.yaml'), linear=float(value('max_linear_speed')),
                              angular=float(value('max_angular_speed')), radius=float(value('robot_radius')),
                              inflation=float(value('inflation_radius')), scan_topic=scan, **frames)
        actions.append(include('nav2.launch.py', {'params_file': save_config('nav2.yaml', nav),
                                                  'use_sim_time': value('use_sim_time')}))
    if mode == 'explore' or value('marker_detection') == 'true':
        actions.append(Node(package='tb4_mission', executable='marker_detector', name='marker_detector',
                            parameters=[str(package / 'config' / 'mission.yaml'), shared,
                                        {'image_topic': value('image_topic'), 'camera_info_topic': value('camera_info_topic')}],
                            output='screen'))
    if mode == 'explore':
        # Explorer waits for live map, scan, TF, perception heartbeat and Nav2 lifecycle.
        actions.append(Node(package='tb4_mission', executable='explorer', name='explorer',
                            parameters=[str(package / 'config' / 'mission.yaml'), shared,
                                        {'autostart': value('autostart') == 'true',
                                         'robot_radius': float(value('robot_radius'))}], output='screen'))
    if value('rviz') == 'true':
        rviz_text = (package / 'config' / 'mission.rviz').read_text()
        rviz_text = rviz_text.replace('Fixed Frame: map', 'Fixed Frame: ' + frames['map_frame'])
        rviz_text = rviz_text.replace('Value: /scan', 'Value: ' + scan)
        rviz_path = config_dir / 'mission.rviz'
        rviz_path.write_text(rviz_text)
        actions.append(Node(package='rviz2', executable='rviz2', name='mission_rviz',
                            arguments=['-d', str(rviz_path)], parameters=[{'use_sim_time': simulated}], output='screen'))

    def process_exit(event, _context):
        if _context.is_shutdown:
            return []
        (output / 'failure.json').write_text(json.dumps({'returncode': event.returncode, 'reason': 'A ROS process exited unexpectedly'}))
        return [EmitEvent(event=Shutdown(reason=f'Process exited ({event.returncode}); shutting down whole mission'))]
    # Includes vendor nodes: a crash/normal unexpected exit cannot leave navigation running.
    return [RegisterEventHandler(OnProcessExit(on_exit=process_exit)), *actions]


def generate_launch_description():
    defaults = {
        'mode': 'explore', 'map': '',
        'output_dir': str(Path.home() / 'tb4_results' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')),
        'autostart': 'false', 'rviz': 'true', 'use_sim_time': 'false', 'marker_detection': 'false',
        'image_topic': 'oakd/rgb/preview/image_raw', 'camera_info_topic': 'oakd/rgb/preview/camera_info',
        'scan_topic': 'scan', 'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
        'slam_odom_frame': 'mission_odom', 'max_linear_speed': '0.20', 'max_angular_speed': '0.60',
        'robot_radius': '0.22', 'inflation_radius': '0.45',
    }
    arguments = []
    for name, default in defaults.items():
        choices = ['slam', 'explore', 'navigate'] if name == 'mode' else None
        if name in ('autostart', 'rviz', 'use_sim_time', 'marker_detection'):
            choices = ['true', 'false']
        arguments.append(DeclareLaunchArgument(name, default_value=default, choices=choices))
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
