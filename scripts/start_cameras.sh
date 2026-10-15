#!/usr/bin/env bash
set -euo pipefail
# 启动三台 RealSense(front / left / right)。
# 新机器先用 `rs-enumerate-devices --short` 查串号,再在环境变量里覆盖:
#   FRONT_SERIAL=_xxxx ./scripts/start_cameras.sh
FRONT_SERIAL="${FRONT_SERIAL:-_151222073259}"
LEFT_SERIAL="${LEFT_SERIAL:-_254622072400}"
RIGHT_SERIAL="${RIGHT_SERIAL:-_254522075411}"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

exec ros2 launch realsense2_camera multi_camera_3cam_color.launch.py \
  "front_serial:=$FRONT_SERIAL" "left_serial:=$LEFT_SERIAL" "right_serial:=$RIGHT_SERIAL"
