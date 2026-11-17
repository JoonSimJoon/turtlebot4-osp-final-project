"""Use ROS launch itself to verify vendor activation never starts exploration."""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip('yaml')
pytest.importorskip('launch_ros')
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mission_launch', ROOT / 'src/tb4_mission/launch/mission.launch.py')
mission_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission_launch)


@pytest.mark.parametrize('name', ['slam.launch.py', 'nav2.launch.py', 'localization.launch.py'])
def test_vendor_autostart_is_true_and_parent_mission_stays_false(tmp_path, name):
    (tmp_path / 'launch').mkdir()
    receipt = tmp_path / 'vendor_autostart.txt'
    (tmp_path / 'launch' / name).write_text('''
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def record(context):
    Path(%r).write_text(LaunchConfiguration('autostart').perform(context))
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        OpaqueFunction(function=record),
    ])
''' % str(receipt))
    parent_values = []
    def record_parent(context):
        parent_values.append(LaunchConfiguration('autostart').perform(context))
        return []
    service = LaunchService(argv=[])
    service.include_launch_description(LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        mission_launch.include_vendor(tmp_path, name, {}),
        OpaqueFunction(function=record_parent),
    ]))
    assert service.run() == 0
    assert receipt.read_text() == 'true'
    assert parent_values == ['false']
