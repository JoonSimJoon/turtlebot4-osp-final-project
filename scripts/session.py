#!/usr/bin/env python3
"""Own one ROS launch process group; do not kill unrelated ROS processes."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import threading

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / '.runtime'
STATE = RUNTIME / 'session.json'


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['run', 'stop'])
    parser.add_argument('mode', nargs='?', choices=['explore', 'slam', 'navigate'])
    parser.add_argument('map', nargs='?')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    if args.action == 'run' and not args.mode:
        parser.error('run requires a mode')
    if args.mode == 'navigate':
        if not args.map:
            parser.error('navigate requires an absolute map.yaml path')
        map_path = Path(args.map).expanduser()
        if not map_path.is_absolute() or not map_path.is_file():
            parser.error('map must be an existing absolute file path')
        args.map = str(map_path.resolve())
    elif args.map:
        parser.error('a map is only accepted for navigate')
    return args


def launch_command(args, output):
    command = ['ros2', 'launch', 'tb4_mission', 'mission.launch.py',
               f'mode:={args.mode}', f'output_dir:={output}',
               f'autostart:={str(args.mode == "explore").lower()}',
               f'rviz:={"false" if args.no_rviz else os.getenv("RVIZ", "true")}']
    if args.map:
        command.append(f'map:={args.map}')
    settings = {
        'image_topic': ('IMAGE_TOPIC', 'oakd/rgb/preview/image_raw'),
        'camera_info_topic': ('CAMERA_INFO_TOPIC', 'oakd/rgb/preview/camera_info'),
        'scan_topic': ('SCAN_TOPIC', 'scan'), 'map_frame': ('MAP_FRAME', 'map'),
        'odom_frame': ('ODOM_FRAME', 'odom'), 'base_frame': ('BASE_FRAME', 'base_link'),
        'slam_odom_frame': ('SLAM_ODOM_FRAME', 'mission_odom'),
        'max_linear_speed': ('MAX_LINEAR_SPEED', '0.20'),
        'max_angular_speed': ('MAX_ANGULAR_SPEED', '0.60'),
        'robot_radius': ('ROBOT_RADIUS', '0.22'),
        'inflation_radius': ('INFLATION_RADIUS', '0.45'),
    }
    for name, (env, default) in settings.items():
        command.append(f'{name}:={os.getenv(env, default)}')
    return command


def proc_start(pid):
    """Linux PID reuse protection (start ticks after the process comm field)."""
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def stop():
    if not STATE.exists():
        print('실행 중인 프로젝트 세션이 없습니다.')
        return 0
    try:
        state = json.loads(STATE.read_text())
        pid = state['supervisor_pid']
        if state.get('project') != str(ROOT) or not state.get('start_ticks'):
            raise ValueError('session identity is missing')
        if proc_start(pid) != state['start_ticks']:
            print('이전 세션은 이미 종료되었습니다. 현재 프로세스에는 신호를 보내지 않습니다.')
            return 0
        os.kill(pid, signal.SIGTERM)
        print('목표 취소·지도 저장 후 실행 세션을 종료합니다.')
        deadline = time.monotonic() + 35
        while proc_start(pid) == state['start_ticks'] and time.monotonic() < deadline:
            time.sleep(0.2)
        if proc_start(pid) == state['start_ticks']:
            print('종료가 지연됩니다. 실행 터미널과 .runtime/session.json을 확인하세요.', file=sys.stderr)
            return 1
        receipt = json.loads((Path(state['output_dir']) / 'shutdown.json').read_text())
        if not isinstance(receipt, dict):
            raise ValueError('invalid shutdown receipt')
        if (receipt.get('supervisor_pid') != pid or
                receipt.get('start_ticks') != state['start_ticks']):
            raise ValueError('shutdown receipt does not belong to this session')
        if type(receipt.get('returncode')) is not int or receipt['returncode'] != 0:
            print(f"세션 종료 중 오류가 발생했습니다. 결과와 로그: {state['output_dir']}", file=sys.stderr)
            return 1
    except (OSError, KeyError, ValueError) as error:
        print(f'세션 정보를 검증할 수 없습니다: {error}', file=sys.stderr)
        return 1
    return 0


def checked(command, timeout=25):
    try:
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f'시간 초과: {shlex.join(command)}') from error
    if result.returncode:
        raise RuntimeError(f'{shlex.join(command)}\n{result.stdout}')
    return result.stdout


def preflight(args):
    timeout = float(os.getenv('PREFLIGHT_TIMEOUT', '20'))
    command = ['ros2', 'run', 'tb4_mission', 'doctor', '--timeout', str(timeout)]
    if args.mode == 'explore':
        command.append('--camera')
    print(checked(command, timeout=timeout + 10), end='')
    nodes = checked(['ros2', 'node', 'list'], timeout=15).splitlines()
    exclusive = {'slam_toolbox', 'amcl', 'map_server', 'bt_navigator',
                 'explorer', 'marker_detector', 'origin_anchor'}
    duplicates = [name for name in nodes if name.strip().rsplit('/', 1)[-1] in exclusive]
    if duplicates:
        raise RuntimeError('이미 실행 중인 지도/주행 노드를 먼저 종료하세요: ' + ', '.join(duplicates))


def cancel_mission(mode, output):
    if mode != 'explore':
        return True
    try:
        result = subprocess.run(['ros2', 'service', 'call', '/mission/stop',
                                 'std_srvs/srv/Trigger', '{}'], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=6)
        print(result.stdout, end='')
    except (subprocess.TimeoutExpired, OSError):
        print('mission/stop 응답 없음; 종료 상태를 기다립니다.', file=sys.stderr)
    # Explorer waits up to 8 s for the actual Nav2 terminal result, not only cancel ACK.
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            summary = json.loads((output / 'summary.json').read_text())
            if summary.get('state') in ('finished', 'error'):
                if not summary.get('stop_confirmed'):
                    break
                receipt = json.loads((output / 'targets_finalized.json').read_text())
                if receipt.get('finalized') is True:
                    return True
        except (OSError, ValueError):
            pass
        time.sleep(0.2)
    (output / 'cancel_unconfirmed.json').write_text(json.dumps({
        'reason': 'Nav2 cancellation or final target export was not confirmed; terminating owned launch group',
    }))
    print('주행 취소 또는 최종 마커 저장 완료를 확인하지 못했습니다. 소유한 ROS launch를 종료하며 cancel_unconfirmed.json에 기록합니다.', file=sys.stderr)
    return False


def shutdown_group(proc):
    if proc.poll() is not None:
        # launch may have failed before cleaning all child nodes; its group is still ours.
        pass
    for sig, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)]:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            proc.poll()
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.1)


def run(args):
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    output = Path(os.getenv('RESULTS_DIR', str(ROOT / 'results'))).expanduser().resolve() / stamp
    command = launch_command(args, output)
    if args.dry_run:
        print(f'ROS_DOMAIN_ID={os.getenv("ROS_DOMAIN_ID", "0")} '
              f'RMW_IMPLEMENTATION={os.getenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}')
        print('실제 프로세스 실행 및 로봇 이동 없음. 실행 예정 명령:')
        print(shlex.join(command))
        return 0
    RUNTIME.mkdir(exist_ok=True)
    with (RUNTIME / 'mission.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('프로젝트 세션이 이미 실행 중입니다. ./start.sh stop 을 사용하세요.')
        preflight(args)
        output.mkdir(parents=True, exist_ok=False)
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = str(output / 'ros_logs')
        request = {'stop': False}
        def on_signal(_number, _frame):
            request['stop'] = True
        old_handlers = {s: signal.signal(s, on_signal) for s in (signal.SIGINT, signal.SIGTERM)}
        proc = None
        exit_code = 1
        supervisor_ticks = proc_start(os.getpid())
        try:
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    env=env, start_new_session=True)
            state = {'project': str(ROOT), 'supervisor_pid': os.getpid(),
                     'start_ticks': supervisor_ticks, 'launch_pid': proc.pid,
                     'mode': args.mode, 'output_dir': str(output), 'command': command}
            STATE.write_text(json.dumps(state, ensure_ascii=False, indent=2))
            (output / 'session.json').write_text(json.dumps(state, ensure_ascii=False, indent=2))
            print(f'결과 디렉터리: {output}', flush=True)
            with (output / 'console.log').open('wb') as log:
                def pump_output():
                    while True:
                        chunk = proc.stdout.read1(65536)
                        if not chunk:
                            break
                        log.write(chunk)
                        log.flush()
                        sys.stdout.buffer.write(chunk)
                        sys.stdout.buffer.flush()
                reader = threading.Thread(target=pump_output, daemon=True)
                reader.start()
                while proc.poll() is None and not request['stop']:
                    time.sleep(0.2)
                confirmed = cancel_mission(args.mode, output) if request['stop'] else True
                return_code = proc.poll()
                shutdown_group(proc)
                reader.join(timeout=3)
            if request['stop']:
                exit_code = 0 if confirmed else 1
            else:
                exit_code = return_code or int((output / 'failure.json').exists())

        finally:
            if proc is not None:
                shutdown_group(proc)
            # The separate stop CLI cannot waitpid() this process. Persist its result
            # after cleanup, before removing the active-session record and exiting.
            try:
                (output / 'shutdown.json').write_text(json.dumps({
                    'supervisor_pid': os.getpid(), 'start_ticks': supervisor_ticks,
                    'returncode': exit_code, 'stop_requested': request['stop'],
                }))
            except OSError as error:
                exit_code = 1
                print(f'종료 결과를 저장하지 못했습니다: {error}', file=sys.stderr)
            STATE.unlink(missing_ok=True)
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
            print(f'세션 종료. 결과와 로그: {output}')
        return exit_code


def main(argv=None):
    args = arguments(sys.argv[1:] if argv is None else argv)
    try:
        return stop() if args.action == 'stop' else run(args)
    except (RuntimeError, ValueError, OSError) as error:
        print(f'실행 실패: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
