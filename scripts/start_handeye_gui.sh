#!/usr/bin/env bash
set -euo pipefail
# 启动手眼标定 GUI。
# 与原机保持一致:清掉 Conda / 旧 build 环境变量,只用本仓库的 install 覆盖层。
export HANDEYE_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
  -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 \
  /bin/bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    source "$HANDEYE_WS_DIR/install/setup.bash"
    exec /usr/bin/python3 -m handeye_calibration_ros.calibration_gui "$@"
  ' handeye-gui "$@"
