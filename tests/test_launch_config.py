from copy import deepcopy
import pytest
from tb4_mission.launch_config import nav2_parameters


def vendor_config():
    return {
        'controller_server': {'ros__parameters': {'enable_stamped_cmd_vel': True, 'FollowPath': {
            'plugin': 'nav2_mppi_controller::MPPIController', 'critics': ['CostCritic']}}},
        'velocity_smoother': {'ros__parameters': {}},
        'behavior_server': {'ros__parameters': {'global_frame': 'map', 'local_frame': 'odom'}},
        'planner_server': {'ros__parameters': {'GridBased': {'allow_unknown': True}}},
        'collision_monitor': {'ros__parameters': {'cmd_vel_out_topic': 'cmd_vel', 'base_frame_id': 'base_link',
                                                   'observation_sources': ['scan'], 'scan': {'type': 'scan', 'topic': 'scan'}}},
        **{name: {name: {'ros__parameters': {'footprint': '[test]', 'inflation_layer': {},
                                          'obstacle_layer': {'scan': {'topic': 'scan'}}}}}
           for name in ('local_costmap', 'global_costmap')},
    }


def test_tuning_preserves_collision_pipeline_and_matches_speed_limits():
    source = vendor_config()
    before = deepcopy(source)
    result = nav2_parameters(source, linear=.18, angular=.5, radius=.22, scan_topic='/laser')
    assert source == before
    assert result['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == 'cmd_vel'
    assert result['collision_monitor']['ros__parameters']['scan']['topic'] == '/laser'
    follow = result['controller_server']['ros__parameters']['FollowPath']
    smoother = result['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [follow['vx_max'], 0.0, follow['wz_max']]
    assert follow['critics'] == ['CostCritic']
    for name in ('local_costmap', 'global_costmap'):
        config = result[name][name]['ros__parameters']
        assert 'footprint' not in config
        assert config['robot_radius'] == .22
        assert config['obstacle_layer']['scan']['topic'] == '/laser'
    assert result['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False


@pytest.mark.parametrize('overrides', [{'linear': -1}, {'linear': float('nan')}, {'linear': 2},
                                       {'angular': 5}, {'radius': .5, 'inflation': .2}])
def test_unsafe_limits_rejected(overrides):
    with pytest.raises(ValueError):
        nav2_parameters(vendor_config(), **overrides)


def test_other_nav2_controller_not_silently_modified():
    source = vendor_config()
    source['controller_server']['ros__parameters']['FollowPath']['plugin'] = 'dwb_core::DWBLocalPlanner'
    with pytest.raises(ValueError, match='Jazzy'):
        nav2_parameters(source)
