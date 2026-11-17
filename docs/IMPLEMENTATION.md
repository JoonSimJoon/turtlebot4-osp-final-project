# 구현 설명

## 구성

ROS 패키지는 `src/tb4_mission`의 ament_python 패키지입니다. 기존 TurtleBot4 드라이버를 다시 띄우지 않고 노트북에서 SLAM·Nav2·과제 노드를 실행합니다.

| 파일 | 책임 |
|---|---|
| `launch/mission.launch.py` | 모드별 공식 SLAM/Nav2/AMCL과 프로젝트 노드 실행 |
| `tb4_mission/origin_anchor.py` | 시작 시 odom 위치를 한 번 기록해 출발 좌표계 생성 |
| `tb4_mission/launch_config.py` | 설치된 공식 Jazzy Nav2 설정의 속도·반경·토픽 조정 |
| `tb4_mission/frontier.py` | 지도 좌표 변환, 장애물 여유거리, 도달 가능 영역, 탐색 후보 |
| `tb4_mission/explorer.py` | 센서 준비, Nav2 액션, 시간 제한, 종료·저장 |
| `tb4_mission/mission_logic.py` | 중복 ID 제거, 종료 판정, 실패 목적지 임시 제외 |
| `tb4_mission/marker_geometry.py` | ArUco 검출과 카메라 광선–LiDAR 표면 교차 |
| `tb4_mission/target_store.py` | 반복 관측 확인·중앙값 추정·좌표 변환·CSV 저장 |
| `tb4_mission/marker_detector.py` | 영상/스캔 시간 정합, TF, RViz 표적·준비 상태 발행 |
| `tb4_mission/map_io.py` | OccupancyGrid를 PGM/YAML로 저장 |
| `tb4_mission/doctor.py` | 이동 없이 실제 수신 데이터와 TF를 확인 |
| `scripts/session.py` | 단일 세션 잠금, 사전 진단, 로그, 소유 프로세스 종료 |

## 좌표와 SLAM

로봇 odom은 전원을 켠 뒤 이미 움직였을 수 있으므로 과제 출발점이 `(0,0,0)`이라고 가정할 수 없습니다. 시작 시 odom→base_link 위치 `(x₀,y₀,θ₀)`를 받아 그 역변환을 `mission_odom→odom`에 고정합니다. SLAM은 `mission_odom`을 odometry frame으로 사용합니다.

```text
map → mission_odom → odom → base_link → lidar / camera_optical
        SLAM 기준      기존 로봇 좌표     센서 외부 보정
```

SLAM의 `map_start_pose` 설정만으로 새 지도의 시작 odom이 리셋되는 것은 아닙니다. 이 구현은 하드웨어 odom을 변경하지 않고 별도 기준 프레임을 둡니다. 저장 지도 AMCL 모드에서는 이 시작 기준 노드를 띄우지 않습니다.

지도 셀 변환은 origin의 x/y뿐 아니라 yaw 회전도 반영합니다. 지도 원점 좌표(이미지의 왼쪽 아래)와 `map` 좌표의 `(0,0)`은 다릅니다. 출발점이 `map=(0,0)`이더라도 저장 YAML의 origin은 음수일 수 있습니다.

## 탐색과 Nav2

OccupancyGrid에서 알려진 빈 셀만 이동 가능하게 두고 장애물을 로봇 반경만큼 확장합니다. 대각선 경로가 두 장애물 사이 모서리를 뚫지 않도록 검사합니다. Dijkstra로 현재 로봇과 연결된 영역을 구한 뒤 미지 공간에 접한 셀을 묶습니다.

경계 자체보다 안쪽의 안전한 셀을 목적지로 선택하고, 경계 크기와 이동 거리를 이용해 우선순위를 정합니다. 알려진 방 전체가 LiDAR에 한 번에 보이는 경우에는 미지 영역 경계만으로 표적 주변을 방문할 수 없으므로, 알려진 빈 공간의 관측 지점을 방문하는 보완 탐색을 사용합니다. 관측 위치에서는 Nav2 Spin으로 주변을 확인합니다.

탐색 노드는 `/cmd_vel`을 직접 발행하지 않습니다. `NavigateToPose`, `Spin` 액션을 사용하므로 실제 속도·충돌 검사는 Nav2와 TurtleBot4가 수행합니다. Jazzy의 `TwistStamped`, velocity smoother, collision monitor 구성을 공식 설정에서 유지합니다.

## 마커 위치 추정

1. 카메라 영상에서 `DICT_ARUCO_ORIGINAL` 마커와 ID를 찾습니다. 알려진 ID 목록은 없습니다.
2. 사각형의 대각선 교점으로 중심 픽셀을 계산하고 CameraInfo의 내부 보정·왜곡 계수로 광선을 구합니다.
3. 영상과 가까운 시각의 스캔을 선택합니다. 카메라와 LiDAR를 각자의 측정 시각 TF로 odom 좌표에 변환합니다.
4. LiDAR의 연속된 표면 중 카메라 광선과 교차하는 가까운 표면을 찾고, 주변 점들에 직선을 맞춥니다. 간격·잔차·입사각·높이 조건이 나쁜 경우 관측을 버립니다.
5. 지지면이 수직으로 바닥까지 이어진다는 과제 조건을 이용해 광선과 지지면의 교차점을 표적 위치로 사용합니다. 마커의 실제 너비는 입력하지 않습니다.
6. 기본 1.8 m 이내에서 서로 다른 시각의 일치 관측이 3회 쌓여야 확인된 표적으로 등록합니다. 반복 샘플의 중앙값을 저장합니다.
7. 최신 map→odom TF로 바꾸어 지도 위에 ID와 점을 표시하고 CSV를 갱신합니다.

이는 RGB 영상과 2D LiDAR만으로 위치를 추정하는 방법입니다. 마커 앞에서 LiDAR 높이만 가리는 장애물은 본래 표면과 구분이 어려울 수 있습니다. 표면이 휘어 있거나 마커가 LiDAR가 관측하지 못하는 위치에 있으면 거부될 수 있습니다. 실물 테스트로 인식 거리와 위치 오차를 측정해야 합니다.

## 임무 상태

```text
waiting → starting → running → stopping → finished
                                    └──→ error
```

`waiting`에서 지도·스캔·TF·영상 처리·Nav2 lifecycle을 검사합니다. 시작 시 표적 저장소 초기화를 확인한 후 움직입니다. ROS 시뮬레이션 시간이 멈추더라도 실제 경과시간 제한이 유지되도록 steady timer와 monotonic 시간을 사용합니다.

목표 취소 요청을 보내는 것과 로봇의 액션 종료를 확인하는 것은 다릅니다. 코드는 결과를 기다린 뒤 저장하며, 확인되지 않은 경우 `cancel_unconfirmed`로 남깁니다. 늦게 수락된 목표도 종료 후 이동하지 않도록 취소합니다. 임무당 한 번만 시작하며 새 시연은 새 launch와 새 지도에서 수행합니다.

`stopping`부터 마커 신규 관측을 멈추고 최종 상태에서 CSV/RViz 좌표를 한 번 더 갱신한 뒤 고정합니다. 지도와 CSV는 별도 ROS 콜백에서 저장되므로 센서 처리 지연이나 마지막 SLAM 보정이 큰 환경에서는 마지막 좌표 일치도 함께 확인해야 합니다.

## 결과 파일

PGM은 ROS OccupancyGrid의 행 순서를 이미지 좌표에 맞게 뒤집습니다. 빈 공간 254, 장애물 0, 미지 공간 205로 저장하고 YAML에 해상도·origin·임계값을 기록합니다. 파일은 임시 파일에 완전히 쓴 뒤 교체해 중간 쓰기 내용을 읽는 문제를 줄입니다. PGM과 YAML 두 파일을 묶은 단일 트랜잭션은 아니므로 저장 도중 강제 전원 차단까지 복구하는 방식은 아닙니다.

`targets.csv`에는 `id,x,y`를 저장합니다. `summary.json`에는 종료 사유·경과시간·목표 ID·저장 오류·정지 확인 상태가 들어갑니다. 준비 전에 종료해 지도가 없거나 디스크 쓰기에 실패한 경우 성공으로 표시하지 않습니다.

## 보고서에 사용할 설명과 직접 측정할 항목

알고리즘 선택 이유, TF 구조, 크기를 모르는 마커에 LiDAR를 사용한 이유, 종료 상태 처리는 이 문서와 코드로 설명할 수 있습니다. 주행 성공률·지도 정확도·마커 좌표 오차·3개 발견 시간·수동 개입 횟수는 실제 실험으로 측정해야 합니다. 로컬 합성 입력 테스트는 통신 인터페이스와 제어 흐름의 검증 자료이며 실제 로봇 성능 수치가 아닙니다.
