#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
require_linux
load_config
source_ros
cd -- "$TB4_ROOT"
/usr/bin/colcon build --packages-select tb4_mission
printf '\n빌드 완료. LAN 연결과 config/robot.env를 확인한 뒤 ./start.sh doctor 를 실행하세요.\n'
