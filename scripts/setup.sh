#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
require_linux
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] || { echo 'Ubuntu 24.04만 지원합니다. 기존 OS/로봇 펌웨어를 자동 변경하지 않습니다.' >&2; exit 1; }
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
  echo "현재 ROS_DISTRO=$ROS_DISTRO 입니다. Jazzy 환경에서 다시 실행하세요." >&2; exit 1
fi
load_config
sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates
sudo add-apt-repository -y universe
if ! apt-cache show ros-jazzy-ros-base >/dev/null 2>&1; then
  # Official ROS apt repository package; download a release asset, never pipe code to a shell.
  # https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
  task_tmp_dir="$(mktemp -d)"
  trap 'rm -rf -- "$task_tmp_dir"' EXIT
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$task_tmp_dir/release.json"
  task_version="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$task_tmp_dir/release.json")"
  [[ "$task_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'ROS apt release 버전을 확인할 수 없습니다.' >&2; exit 1; }
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$task_version/ros2-apt-source_${task_version}.noble_all.deb" -o "$task_tmp_dir/ros2-apt-source.deb"
  sudo dpkg -i "$task_tmp_dir/ros2-apt-source.deb"
fi
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base ros-jazzy-turtlebot4-desktop ros-jazzy-turtlebot4-navigation \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-cv-bridge ros-jazzy-tf2-ros-py ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rviz2 \
  python3-colcon-common-extensions python3-numpy python3-opencv \
  python3-yaml python3-pytest python3-setuptools
/usr/bin/python3 -c 'import cv2; assert hasattr(cv2, "aruco"), "OpenCV aruco module missing"'
if [[ ! -f "$TB4_ROOT/config/robot.env" ]]; then
  cp -- "$TB4_ROOT/config/robot.env.example" "$TB4_ROOT/config/robot.env"
fi
"$TB4_ROOT/scripts/build.sh"
printf '\n최초 설치 완료. config/robot.env를 로봇 설정에 맞추세요. 네트워크/로봇 설정은 자동 변경하지 않았습니다.\n'
