"""CLI failure path must report missing sensors, never crash during construction."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

pytest.importorskip('rclpy')


@pytest.mark.parametrize('extra', [[], ['--camera'], ['--navigation']])
def test_disconnected_doctor_reports_missing_inputs(extra):
    if extra == ['--navigation']:
        pytest.importorskip('nav2_msgs')
    env = dict(os.environ, ROS_DOMAIN_ID='93', ROS_LOCALHOST_ONLY='1')
    source = str(Path(__file__).resolve().parents[1] / 'src' / 'tb4_mission')
    env['PYTHONPATH'] = source + os.pathsep + env.get('PYTHONPATH', '')
    result = subprocess.run(
        [sys.executable, '-m', 'tb4_mission.doctor', '--timeout', '0.15', *extra],
        capture_output=True, text=True, env=env, timeout=15)
    assert result.returncode == 1, result.stderr
    assert '[FAIL] scan 수신' in result.stdout
    assert '이 검사는 이동 명령을 보내지 않습니다.' in result.stdout
    assert 'Traceback' not in result.stderr
