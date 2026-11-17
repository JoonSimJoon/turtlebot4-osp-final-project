# 검증 기록

검증일: 2026-11-17 (Asia/Seoul)

**초기 로컬 검증은 자동 테스트 78개 통과, 실패 0개, 생략 0개입니다.** 테스트 환경은 macOS arm64의 conda `ros-jazzy`, Python 3.12.14, OpenCV 4.13.0입니다. 이후 추가된 회귀 테스트와 Ubuntu 24.04 설치·빌드 검증 결과는 [GitHub Actions](https://github.com/JoonSimJoon/turtlebot4-osp-final-project/actions/workflows/ci.yml)의 최신 실행 및 첨부 보고서에서 확인할 수 있습니다. 실제 TurtleBot4 주행은 미검증입니다.

## 수행한 검증

| 범위 | 검증 내용 | 결과 |
|---|---|---|
| Frontier/coverage | 회전된 지도 원점, 장애물 여유 거리, 연결 영역, 대각선 모서리, 알려진 방 관측 지점, 벽 너머 관측 오인 방지 | 통과 |
| 지도 저장 | PGM 상하 반전, YAML 좌표·해상도, unknown 셀 재로드 보존, 원자적 쓰기 | 통과 |
| 임무 정책 | 서로 다른 ID 집계, 600초 경계, 실패 목적지 만료 | 통과 |
| 실제 ROS 액션 통신 | 가짜 Nav2 서버와 실제 ActionClient/ActionServer: 준비 전 거부, 초기화 후 회전, 시간/목표/센서 손실 종료, 늦은 목표 수락 취소, 알려진 공간 NavigateToPose 요청 | 통과 |
| ArUco/기하 | ID 7·42·817 및 서로 다른 영상 크기, 카메라–LiDAR 평행이동·회전, 표면 단절/잘못된 거리/이상치 | 통과 |
| 실제 ROS 마커 처리 | CameraInfo/Image/LaserScan/TF/cv_bridge 입력 → 근거리 확인 → map CSV, 먼 표적 미확인, 최종 좌표 고정·완료 영수증·초기화 | 통과 |
| 시작 원점 | 평행이동·회전 역변환 | 통과 |
| Launch 설정 | 실제 LaunchService로 vendor autostart 분리, 속도/스캔/프레임 설정, 공식 Jazzy Nav2 YAML 적용 | 통과 |
| CLI | 경로에 공백, 잘못된 배포판·지도 입력, dry-run, 취소 확인·CSV 완료 대기 | 통과 |
| Doctor | 실제 ROS 노드 생성, 센서 없는 상태를 정상 실패로 보고, camera/navigation 옵션 | 통과 |
| 패키지 | 일반 colcon build, 실행 파일 4개 발견, launch --show-args | 통과 |
| 정적 검사 | Bash 구문, Python 컴파일 | 통과 |

ROS 통합 테스트의 주행 서버는 테스트용 대역입니다. 실제 Nav2 플래너·컨트롤러·충돌 회피 성능을 실행한 결과가 아닙니다. 합성 마커 입력도 실제 OAK 카메라의 노출·흔들림·보정 오차를 재현하지 않습니다.

검토에서 발견한 자동 시작 설정 충돌, 최종 CSV 저장 전 종료 가능성, unknown 지도 셀이 free로 변하는 저장 임계값, 벽 너머 관측 완료 판정, ROS Node 예약 속성명 충돌을 수정하고 회귀 검증했습니다. 독립 검토의 남은 중대 지적은 없습니다.

## 테스트 재실행

ROS Jazzy 환경과 OpenCV가 설치된 상태에서 프로젝트 루트에서 실행합니다. 아래 네트워크 환경변수는 테스트 명령에만 적용하며 실제 로봇 실행 터미널에 영구 적용하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
TB4_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 \
PYTHONPATH="$PWD/src/tb4_mission" \
python3 -m pytest -q
```

로컬 conda 검증에 사용한 명령:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
TB4_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 \
conda run --no-capture-output -n ros-jazzy python -m pytest -q \
  --junitxml=.verification/pytest.xml
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`은 기존 conda 환경의 launch_testing 플러그인과 pytest 버전 충돌을 피하기 위한 테스트 러너 설정입니다. 프로젝트 테스트 자체는 생략하지 않습니다. ROS/OpenCV가 없는 일반 Python에서는 관련 테스트가 생략되므로 그 결과를 위 78개 전체 통과와 혼동하지 마세요.

로컬 검증용 빌드는 별도 경로를 사용했습니다.

```bash
conda run --no-capture-output -n ros-jazzy colcon \
  --log-base .verification/log build --base-paths src \
  --build-base .verification/build --install-base .verification/install
```

현재 conda setuptools는 `--symlink-install` 경로의 `--editable` 옵션을 지원하지 않아 일반 빌드로 수정했습니다. 배포용 `scripts/build.sh`도 일반 빌드를 사용합니다. 소스를 수정한 뒤 다시 빌드해야 설치된 노드에 반영됩니다.

로컬 테스트를 위해 기존 conda 환경에 Nav2 및 geographic 메시지 인터페이스와 그 ROS 의존 패키지를 추가했습니다. 기존 패키지의 버전은 변경하지 않았습니다.

## 아직 장비에서 확인할 내용

- 학교 로봇의 실제 ROS 배포판, DDS 도메인/discovery, Ethernet 주소, 센서 토픽.
- 실제 센서와 연결한 공식 SLAM/Nav2 전체 launch 실행. CI의 Ubuntu 설치·빌드와 launch 인수 검사는 센서 연결을 대신하지 않습니다.
- 실제 시작 위치의 map 원점, 센서 TF·시간 동기화, odom 드리프트.
- 로봇+노트북 외곽에 맞는 반경, 장애물 회피, 통신 손실 시 실제 정지.
- 3개 마커의 실제 인식률, 2 m 접근, 지도상 위치 오차, 10분 내 완료 시간.
- 최종 저장 지도와 CSV가 실제 공간과 일치하는지, RViz 화면 제출.

이 항목들은 코드·통신 테스트로 대신할 수 없습니다. [운영 가이드](OPERATIONS.md)의 실물 기록 양식으로 첫 연결 테스트를 남기세요.
