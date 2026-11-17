#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
[[ $# -le 1 ]] || { echo '사용법: ./start.sh save-map [output_dir]' >&2; exit 2; }
require_linux
load_config
source_workspace
task_map_dir="${1:-${RESULTS_DIR:-$TB4_ROOT/results}/map-$(date +%Y%m%d-%H%M%S)}"
mkdir -p -- "$task_map_dir"
task_map_dir="$(cd -- "$task_map_dir" && pwd)"
if [[ -e "$task_map_dir/map.yaml" || -e "$task_map_dir/map.pgm" ]]; then
  echo "이미 지도가 있습니다. 다른 출력 디렉터리를 지정하세요: $task_map_dir" >&2
  exit 1
fi
ros2 run nav2_map_server map_saver_cli -f "$task_map_dir/map" --ros-args \
  -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
[[ -s "$task_map_dir/map.yaml" && -s "$task_map_dir/map.pgm" ]] || { echo '지도 파일 생성에 실패했습니다.' >&2; exit 1; }
printf '지도 저장 완료: %s/map.yaml\n' "$task_map_dir"
