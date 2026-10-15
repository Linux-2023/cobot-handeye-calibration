# cobot-handeye-calibration

协作臂(AGILEX PiPER 双臂)手眼标定一体包:标定包(含本地定制的 PyQt5 GUI)、ArUco 检测、RealSense 驱动源码与启动脚本,单仓库自包含,新机器 clone 后一次构建即可使用。

## 目录结构

```
src/handeye_calibration_ros   手眼标定包 + GUI(基于 agilexrobotics/handeye_calibration_ros@f2ef3ea 定制)
src/aruco_ros                 ArUco 检测(pal-robotics/aruco_ros, humble 分支,未改动)
src/realsense-ros             RealSense 驱动(Intel realsense-ros 4.56.4,含自定义 multi_camera_3cam_color.launch.py)
scripts/start_cameras.sh      启动 front/left/right 三台 RealSense
scripts/start_handeye_gui.sh  启动标定 GUI
```

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- librealsense2(含 udev rules,按 [Intel 官方说明](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) 安装,否则相机打不开)
- 其余 Python/ROS 依赖用 rosdep 安装(见下)

## 构建

```bash
cd cobot-handeye-calibration
sudo apt install python3-pyqt5 python3-opencv python3-scipy python3-rosdep
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
```

## 运行

```bash
# 1. 查看本机相机串号并启动相机(新相机需覆盖串号)
rs-enumerate-devices --short
FRONT_SERIAL=_xxxxxxxx LEFT_SERIAL=_xxxxxxxx RIGHT_SERIAL=_xxxxxxxx ./scripts/start_cameras.sh

# 2. 另开终端启动标定 GUI,在界面里填入机械臂位姿话题、marker 位姿话题、图像话题与保存路径
./scripts/start_handeye_gui.sh
```

标定结果 JSON 保存在 GUI 里填写的目录;每台机器人本体的标定结果互相独立,不要混用。

## 上游与许可

- [agilexrobotics/handeye_calibration_ros](https://github.com/agilexrobotics/handeye_calibration_ros)(GUI 为本仓库新增定制)
- [pal-robotics/aruco_ros](https://github.com/pal-robotics/aruco_ros)
- [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)(4.56.4 + 自定义多相机 launch)

各子目录保留其原始 Apache-2.0 许可证,改动处见各包内文件。
