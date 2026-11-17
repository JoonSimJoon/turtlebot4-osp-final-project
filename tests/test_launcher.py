import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('tb4_session', ROOT / 'scripts/session.py')
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


def test_map_path_with_spaces_is_one_argument(tmp_path):
    map_file = tmp_path / 'my classroom map.yaml'
    map_file.write_text('image: map.pgm\n')
    args = session.arguments(['run', 'navigate', str(map_file)])
    command = session.launch_command(args, tmp_path / 'my run')
    assert f'map:={map_file}' in command
    assert f'output_dir:={tmp_path / "my run"}' in command
    assert 'autostart:=false' in command


def test_exploration_is_explicit_start():
    args = session.arguments(['run', 'explore', '--no-rviz'])
    command = session.launch_command(args, Path('/tmp/run'))
    assert 'autostart:=true' in command
    assert 'rviz:=false' in command


@pytest.mark.parametrize('args', [[], ['run'], ['run', 'navigate'], ['run', 'slam', 'bad-map'], ['run', 'bogus']])
def test_invalid_cli_stops_before_ros(args):
    with pytest.raises(SystemExit):
        session.arguments(args)


def test_dry_run_never_calls_ros(tmp_path):
    env = dict(os.environ, ROS_DISTRO='jazzy')
    result = subprocess.run(['bash', str(ROOT / 'start.sh'), 'explore', '--dry-run'],
                            capture_output=True, text=True, cwd=tmp_path, env=env)
    assert result.returncode == 0, result.stderr
    assert 'ros2 launch tb4_mission mission.launch.py' in result.stdout
    assert 'autostart:=true' in result.stdout


def test_existing_wrong_ros_distro_rejected():
    result = subprocess.run(['bash', str(ROOT / 'start.sh'), 'explore', '--dry-run'],
                            capture_output=True, text=True, env=dict(os.environ, ROS_DISTRO='humble'))
    assert result.returncode != 0
    assert 'ROS_DISTRO=humble' in result.stderr


def test_cancel_waits_for_actual_terminal_confirmation(tmp_path, monkeypatch):
    import json
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: subprocess.CompletedProcess(args, 0, stdout='stop accepted\n'))
    (tmp_path / 'summary.json').write_text(json.dumps({'state': 'finished', 'stop_confirmed': True}))
    (tmp_path / 'targets_finalized.json').write_text(json.dumps({'finalized': True}))
    assert session.cancel_mission('explore', tmp_path)
    assert not (tmp_path / 'cancel_unconfirmed.json').exists()


def test_cancel_ack_without_terminal_stop_is_recorded_as_unconfirmed(tmp_path, monkeypatch):
    import json
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: subprocess.CompletedProcess(args, 0, stdout='stop accepted\n'))
    (tmp_path / 'summary.json').write_text(json.dumps({'state': 'error', 'stop_confirmed': False}))
    assert not session.cancel_mission('explore', tmp_path)
    assert (tmp_path / 'cancel_unconfirmed.json').is_file()


def test_stop_does_not_finish_before_target_export_ack(tmp_path, monkeypatch):
    import json
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: subprocess.CompletedProcess(args, 0, stdout='stop accepted\n'))
    (tmp_path / 'summary.json').write_text(json.dumps({'state': 'finished', 'stop_confirmed': True}))
    sleeps = []
    def finalize_after_wait(_duration):
        sleeps.append(True)
        (tmp_path / 'targets_finalized.json').write_text(json.dumps({'finalized': True}))
    monkeypatch.setattr(session.time, 'sleep', finalize_after_wait)
    assert session.cancel_mission('explore', tmp_path)
    assert sleeps, 'Robot stopped, but final marker export must also be acknowledged'


@pytest.mark.parametrize('returncode', [0, 1, None])
def test_stop_propagates_supervisor_result_after_process_exit(tmp_path, monkeypatch, returncode):
    import json
    state_path = tmp_path / 'session.json'
    output = tmp_path / 'mission-output'
    output.mkdir()
    state_path.write_text(json.dumps({
        'project': str(session.ROOT), 'supervisor_pid': 12345,
        'start_ticks': '123', 'output_dir': str(output),
    }))
    monkeypatch.setattr(session, 'STATE', state_path)
    alive = [True]
    monkeypatch.setattr(session, 'proc_start', lambda _pid: '123' if alive[0] else None)

    def terminate(_pid, _signal):
        alive[0] = False
        if returncode is not None:
            (output / 'shutdown.json').write_text(json.dumps({
                'supervisor_pid': 12345, 'start_ticks': '123', 'returncode': returncode,
            }))
        if returncode == 1:
            (output / 'cancel_unconfirmed.json').write_text('{"reason":"cancel unconfirmed"}')

    monkeypatch.setattr(session.os, 'kill', terminate)
    # Both a failed cleanup and a crash without a receipt must remain failures.
    assert session.stop() == (0 if returncode == 0 else 1)


@pytest.mark.parametrize('returncode', [0, 3])
def test_session_persists_exit_status_for_separate_stop_caller(tmp_path, monkeypatch, returncode):
    import io
    import json
    runtime = tmp_path / 'runtime'
    monkeypatch.setattr(session, 'RUNTIME', runtime)
    monkeypatch.setattr(session, 'STATE', runtime / 'session.json')
    monkeypatch.setenv('RESULTS_DIR', str(tmp_path / 'outputs'))
    monkeypatch.setattr(session, 'preflight', lambda _args: None)
    monkeypatch.setattr(session, 'proc_start', lambda _pid: '456')
    monkeypatch.setattr(session, 'shutdown_group', lambda _proc: None)

    class FinishedLaunch:
        pid = 23456
        stdout = io.BytesIO()

        def poll(self):
            return returncode

    monkeypatch.setattr(session.subprocess, 'Popen', lambda *_args, **_kwargs: FinishedLaunch())
    assert session.run(session.arguments(['run', 'slam', '--no-rviz'])) == returncode
    receipt_path, = (tmp_path / 'outputs').glob('*/shutdown.json')
    receipt = json.loads(receipt_path.read_text())
    assert receipt['returncode'] == returncode
    assert receipt['supervisor_pid'] == os.getpid()
    assert receipt['start_ticks'] == '456'
    assert not (runtime / 'session.json').exists()
