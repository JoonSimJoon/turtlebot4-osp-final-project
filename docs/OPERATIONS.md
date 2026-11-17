# 로봇 연결과 시연 운영

## 장비를 처음 받았을 때

TurtleBot4 모델과 Raspberry Pi의 ROS 배포판부터 확인합니다. 로봇에 이미 설치된 환경을 기준으로 노트북을 맞추세요. 이 프로젝트의 지원 구성은 Ubuntu 24.04 + Jazzy, namespace 없는 단일 로봇입니다. 로봇의 OS·펌웨어·네트워크를 자동 변경하지 않습니다.

로봇의 기존 설정으로 SSH 접속이 된다면 다음 읽기 전용 명령으로 배포판과 서비스를 확인할 수 있습니다.

```bash
ls /opt/ros
systemctl --type=service | grep -E 'turtlebot|create|oak'
```

`/opt/ros/humble`만 있다면 이 프로젝트의 Jazzy 설정으로 실행하지 않습니다. 과제 담당자와 배포판을 확인한 후 노트북/프로젝트를 맞춰야 합니다. 수업 로봇을 임의로 재설치하지 마세요.

## LAN 연결과 DDS

공식 기본 유선 설정은 로봇 `192.168.185.3`, 노트북은 예시 `192.168.185.2/24`입니다. 같은 주소를 양쪽에 넣으면 안 됩니다. 로봇과 같은 네트워크에 있는 것과 ROS 데이터가 발견되는 것은 별개입니다.

| 설정 | 확인할 내용 |
|---|---|
| `ROS_DOMAIN_ID` | 로봇과 노트북이 같아야 함. 여러 팀이 기본값 0을 공유하면 다른 로봇이 보일 수 있음 |
| `RMW_IMPLEMENTATION` | 학교 로봇의 설정과 일치. 예제는 `rmw_fastrtps_cpp` |
| `ROS_DISCOVERY_SERVER` | 서버를 사용하는 로봇에서만 해당 서버와 ID 슬롯을 맞춰 설정 |
| `ROS_SUPER_CLIENT` | Fast DDS Discovery Server에서 CLI로 전체 그래프를 검사하는 PC라면 공식 설정에 따라 사용 |
| `TB4_DISCOVERY_SETUP` | 기존 공식 user-PC discovery setup 파일이 있다면 그 경로 지정 |
| `ROS_LOCALHOST_ONLY` | 실물 연결에서는 1로 남아 있으면 안 됨. 로컬 검증에서만 1 사용 |
| 양쪽 시계 | 영상/스캔/TF 시각이 맞아야 함. NTP/chrony 등 기존 시간 동기화 상태 확인 |

설정은 `config/robot.env`에서 관리합니다. DDS 환경을 바꾼 터미널에서 기존 ROS daemon이 예전 설정을 유지한다면 `ros2 daemon stop` 후 다시 진단합니다. 프로젝트가 다른 ROS 작업의 daemon을 임의로 종료하지는 않습니다.

```bash
./start.sh doctor --camera --timeout 20
```

운영 체제 방화벽이나 DDS discovery mode 때문에 그래프가 보이지 않으면 공식 [네트워킹 문서](https://turtlebot.github.io/turtlebot4-user-manual/setup/networking.html)의 학교 설정과 비교합니다. IP가 통한다고 센서 수신을 생략하지 않습니다.

## 센서 토픽이 다를 때

공식 기본값은 아래와 같습니다. 실제 토픽은 로봇 이미지/카메라 설정에 따라 달라질 수 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
ros2 topic info /scan -v
ros2 topic info /oakd/rgb/preview/image_raw -v
ros2 topic info /oakd/rgb/preview/camera_info -v
ros2 run tf2_ros tf2_echo odom base_link
```

환경 파일을 적용하려면 프로젝트 터미널에서 `source scripts/common.sh; load_config; source_workspace`를 먼저 사용할 수 있습니다. 소스할 파일은 직접 편집한 신뢰하는 로컬 파일이어야 합니다.

RGB 원본과 대응하는 CameraInfo를 한 쌍으로 설정합니다. 압축 이미지 토픽은 이 노드의 입력 타입 `sensor_msgs/Image`와 다릅니다. 카메라 광학 프레임과 LiDAR 프레임 사이의 TF는 기존 로봇 bringup이 제공해야 합니다. 이름만 비슷한 임의의 static TF를 추가하면 위치가 틀어집니다.

기본 OAK preview는 해상도가 작아 멀리 있는 작은 마커를 못 읽을 수 있습니다. 실험에서 인식이 부족하면 로봇에 이미 제공되는 더 높은 해상도의 RGB Image와 정확히 대응하는 CameraInfo 토픽을 선택합니다. depth 토픽이 있다고 가정하지 않습니다. 도킹 상태의 카메라 절전도 먼저 확인합니다.

## 실행 순서

1. 배터리·로봇 bringup·LAN 연결을 확인합니다.
2. 노트북을 고정하고 로봇을 지정된 시작 위치와 방향에 둡니다.
3. `./start.sh doctor --camera`가 통과하는지 확인합니다.
4. 이전 SLAM·AMCL·Nav2 세션이 남아 있으면 종료합니다. 실행 도구는 중복 세션을 거부합니다.
5. 시작 신호에 `./start.sh explore`를 실행합니다. 준비 완료 뒤 자동으로 움직입니다.
6. RViz의 지도와 ID 표시를 관찰합니다. 3개 확인 또는 600초에서 이동을 취소합니다.
7. 종료 로그와 결과 폴더의 `summary.json`을 확인하고 RViz 화면을 제출용으로 캡처합니다. RViz는 결과 검토를 위해 계속 열려 있을 수 있습니다.
8. `./start.sh stop`으로 실행 세션을 정리합니다.

경기 운영에서는 연결·준비 시간도 실제 제한시간에 포함될 수 있습니다. 소프트웨어의 `elapsed_sec`는 센서 준비 후 임무 시작 시점 기준이므로, 심판의 시작 신호와 프로그램 실행 시간을 별도로 측정하세요. **제출 영상의 실제 소요 시간을 소프트웨어 타이머보다 우선**합니다.

## SLAM 연습과 지도 재사용

```bash
./start.sh slam
# 별도 터미널: 현재 지도를 저장
./start.sh save-map /절대/경로/practice-map
# 지도 작성 세션 종료 후
./start.sh stop
./start.sh navigate /절대/경로/practice-map/map.yaml
```

SLAM 모드 자체는 이동 명령을 보내지 않습니다. 연습 중 움직임은 공식 텔레옵 등을 별도 사용합니다. 저장 지도 내비게이션은 RViz에서 2D Pose Estimate를 지정해야 AMCL이 현재 위치를 찾을 수 있습니다. 이후 Nav2 Goal 도구로 목적지를 보냅니다. 과제 본 시연의 자동 탐색은 항상 새 지도에서 `explore`로 시작합니다.

## 주요 튜닝

`config/robot.env`의 값은 실행 시 적용됩니다. 알고리즘 기본값은 `src/tb4_mission/config/mission.yaml`에 있으며, 수정 후 `./start.sh build`를 실행합니다. 매 실행의 최종 SLAM/Nav2 설정은 결과 폴더 `config/`에 남습니다.

| 항목 | 기본/권장 출발점 | 영향 |
|---|---|---|
| `MAX_LINEAR_SPEED` | 0.20 m/s | 이동 속도. 최초 현장 검증 전 급격히 올리지 않음 |
| `MAX_ANGULAR_SPEED` | 0.60 rad/s | 회전 속도. 영상 흔들림/마커 관측 수에도 영향 |
| `ROBOT_RADIUS` | 0.22 m | 로봇과 적재물의 평면 외곽에 맞춤 |
| `INFLATION_RADIUS` | 0.45 m | Nav2가 장애물에서 여유를 두는 영역 |
| `time_limit_sec` | 600 s | 임무 종료 시간 |
| `target_count` | 3 | 확인해야 하는 서로 다른 ID 개수 |
| `confirmations` | 3회 | 서로 다른 시각의 일치 관측 필요 |
| `max_confirmation_distance_m` | 1.8 m | 로봇 중심과 표적 추정 위치가 가까워야 확인. 과제의 2 m 접근 평가를 고려한 여유 |
| `sync_tolerance_sec` | 0.15 s | 영상과 스캔의 최대 시각 차이 |

LiDAR 높이에서 작은 물체가 마커 앞을 가리면 마커의 실제 벽이 아닌 앞 물체에 위치가 잡힐 수 있습니다. 반복 관측과 표면 검사로 일부를 걸러도 구조적으로 모호한 경우는 남습니다. 반드시 벽·박스·가림 상황에서 알려진 실제 거리와 비교하세요.

## 문제가 생겼을 때

| 증상 | 우선 확인 |
|---|---|
| Mac에서 실행 거부 | 실물 실행 도구는 Ubuntu 전용. Mac은 `--dry-run`/테스트용 |
| scan/odom 모두 없음 | 도메인, DDS 설정, IP 대역, 로봇 bringup, 방화벽 |
| 카메라만 없음 | 도킹 절전, OAK 파이프라인, 토픽 이름 |
| `calibration` 또는 해상도 오류 | Image와 CameraInfo가 같은 스트림인지 확인 |
| `timestamp`/TF extrapolation | 로봇과 PC 시간 차이, TF 지연/프레임 이름 |
| `nav2_lifecycle_not_active` | SLAM map/TF가 준비됐는지, Nav2 콘솔 오류 |
| 지도는 나오는데 탐색 시작 안 됨 | `mission/state` readiness와 카메라 heartbeat 확인 |
| 같은 지점을 반복 실패 | 로봇 외곽/여유거리, 지도 장애물, 실물 통로 폭 비교 |
| `cancel_unconfirmed` | 종료 확인 실패. 실제 정지 여부와 통신을 확인하고 물리 정지 장치 사용 |
| 지도 파일 없음 | map 수신 전 종료 또는 저장 권한/디스크 오류. summary/log 확인 |

임무 상태 확인:

```bash
source scripts/common.sh
load_config
source_workspace
ros2 topic echo /mission/state
ros2 topic echo /mission/targets
```

## 실물 검증 기록 양식

- 로봇 모델 / ROS 버전 / 펌웨어 / 날짜:
- 노트북 OS / DDS / 도메인:
- 영상·CameraInfo·scan 토픽 / 프레임:
- 시작 원점과 축 방향 확인:
- 주행·장애물 회피·정지 성공 여부:
- 마커 ID별 실제 위치 / 추정 위치 / 오차:
- 3개 발견 시간 / 600초 종료 확인:
- 결과 폴더와 영상 경로:

이 기록을 채워야 실제 시연 성능을 설명할 수 있습니다.
