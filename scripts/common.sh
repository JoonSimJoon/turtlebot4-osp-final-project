#!/usr/bin/env bash
# Shared by scripts, not meant to be run directly.
set -euo pipefail
TB4_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export TB4_ROOT
load_config() {
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo "현재 터미널에 ROS_DISTRO=$ROS_DISTRO 가 설정되어 있습니다. 새 터미널에서 ROS 2 Jazzy만 source 한 뒤 실행하세요." >&2
    return 1
  fi
  local config_file="${TB4_CONFIG:-$TB4_ROOT/config/robot.env}"
  if [[ ! -f "$config_file" ]]; then
    config_file="$TB4_ROOT/config/robot.env.example"
    printf '설정: 기본 예제 사용 (%s)\n' "$config_file" >&2
  fi
  # Preserve a supplied discovery environment unless the user config overrides it.
  set -a
  if [[ -n "${TB4_DISCOVERY_SETUP:-}" ]]; then
    source "$TB4_DISCOVERY_SETUP"
  fi
  source "$config_file"
  if [[ -n "${TB4_DISCOVERY_SETUP:-}" ]]; then
    source "$TB4_DISCOVERY_SETUP"
    source "$config_file"
  fi
  set +a
  [[ "${ROS_DISTRO:-}" == jazzy ]] || { echo '이 프로젝트는 ROS 2 Jazzy 전용입니다. 로봇과 노트북 버전을 확인하세요.' >&2; return 1; }
  [[ -z "${ROBOT_NAMESPACE:-}" ]] || { echo '현재 버전은 namespace 없는 단일 로봇을 지원합니다. ROBOT_NAMESPACE를 확인하세요.' >&2; return 1; }
  [[ "${ROS_DOMAIN_ID:-0}" =~ ^[0-9]+$ ]] && (( 10#$ROS_DOMAIN_ID <= 232 )) || { echo 'ROS_DOMAIN_ID는 0~232 정수여야 합니다.' >&2; return 1; }
}
require_linux() {
  if [[ "$(uname -s)" != Linux ]]; then
    echo '실물 로봇 실행/설치는 Ubuntu 24.04 + ROS 2 Jazzy 노트북에서 하세요. macOS에서는 --dry-run 및 Python 단위 테스트만 가능합니다.' >&2
    return 1
  fi
}
source_ros() {
  local setup=/opt/ros/jazzy/setup.bash
  [[ -f "$setup" ]] || { echo 'ROS 2 Jazzy가 없습니다. ./start.sh setup 을 먼저 실행하세요.' >&2; return 1; }
  set +u
  source "$setup"
  set -u
}
source_workspace() {
  source_ros
  [[ -f "$TB4_ROOT/install/setup.bash" ]] || { echo '빌드 결과가 없습니다. ./start.sh build 를 실행하세요.' >&2; return 1; }
  set +u
  source "$TB4_ROOT/install/setup.bash"
  set -u
}
