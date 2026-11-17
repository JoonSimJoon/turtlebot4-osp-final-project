#!/usr/bin/env bash
set -euo pipefail
TB4_ENTRY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
  cat <<'HELP'
TurtleBot4 OSP 프로젝트 (Ubuntu 24.04 / ROS 2 Jazzy)
  ./start.sh setup                    최초 의존성 설치 + 빌드
  ./start.sh build                    변경 코드 빌드
  ./start.sh doctor [--camera] [--navigation] [--timeout 20]
  ./start.sh explore [--no-rviz]       SLAM + Nav2 + 자율 탐사 + 마커 (시작 명령)
  ./start.sh slam [--no-rviz]          SLAM + RViz (자율 이동 없음)
  ./start.sh navigate /abs/map.yaml   저장 지도 + Nav2 + RViz (초기 위치 지정 필요)
  ./start.sh save-map [output_dir]    실행 중인 SLAM 지도를 파일로 저장
  ./start.sh stop                     이 프로젝트가 실행한 세션만 종료
  ./start.sh explore --dry-run        실제 실행 없이 명령/설정 확인 (macOS 가능)
HELP
}
if [[ $# -eq 0 ]]; then
  if [[ ! -t 0 ]]; then usage; exit 2; fi
  printf '\n1) 자율 탐사 시작  2) SLAM  3) 저장 지도 주행  4) 연결 진단\n5) 최초 설치       6) 빌드  7) 종료\n'
  read -r -p '선택: ' task_choice
  case "$task_choice" in
    1) set -- explore;; 2) set -- slam;;
    3) read -r -p '지도 YAML 절대 경로: ' task_map; set -- navigate "$task_map";;
    4) set -- doctor --camera;; 5) set -- setup;; 6) set -- build;; 7) set -- stop;;
    *) usage; exit 2;;
  esac
fi
case "$1" in
  -h|--help|help) usage;;
  setup|build|stop) task_action="$1"; shift; [[ $# -eq 0 ]] || { usage; exit 2; }; exec "$TB4_ENTRY_ROOT/scripts/$task_action.sh";;
  explore|slam|navigate) exec "$TB4_ENTRY_ROOT/scripts/run.sh" "$@";;
  save-map) shift; exec "$TB4_ENTRY_ROOT/scripts/save_map.sh" "$@";;
  doctor)
    shift
    source "$TB4_ENTRY_ROOT/scripts/common.sh"
    require_linux
    load_config
    source_workspace
    exec ros2 run tb4_mission doctor "$@"
    ;;
  *) usage >&2; exit 2;;
esac
