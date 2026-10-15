# Hand-Eye Calibration Package Based on ROS2

|UBUNTU|ROS|PYTHON|OPENCV|STATE|
|---|---|---|---|---|
|![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![python](https://img.shields.io/badge/python-3.10-blue.svg)|![opencv](https://img.shields.io/badge/opencv-4.5.0-blue.svg)|![Pass](https://img.shields.io/badge/Pass-green.svg)|

## Hand-Eye Calibration

By collecting multiple sets of end-effector poses from the robotic arm and camera-recognized calibration board poses as input, two types of calibration results can be obtained:
- Eye in hand: The transformation matrix between the robotic arm's end-effector and the camera.
- Eye to hand: The transformation matrix between the robotic arm's base and the camera.

## 1. Installation
### 1.1 Dependencies
```
$ sudo apt install libopencv-dev python3-opencv
$ sudo apt-get install ros-$ROS_DISTRO-tf-transformations
```

### 1.2 Drivers
For testing, we used the `Original ArUco` dictionary calibration board and the `piper` robotic arm.

- [Online Calibration Board Generator]( https://chev.me/arucogen/)
- [Camera Recognition](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)
- [Piper Robotic Arm Program](https://github.com/agilexrobotics/piper_ros/tree/humble)


### 1.3 Build from source
```
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone 
$ cd ..
$ colcon build --symlink-install
```

## 2. Directly run
### 2.1 Start Camera
- Start the camera node according to the actual setup.

### 2.2 Start Robot Arm
```
$ ros2 launch piper start_single_piper.launch.py can_port:=can0
```
- The robotic arm must be in **teaching mode**.

### 2.3 Start Camera Recognition
```
$ ros2 launch aruco_ros single.launch
```
- Need to correct the **marker's size** and **id**, as well as the **image topic** and **frame_id**.

### 2.4 Start Camera Calibration 
When collecting data, it is recommended to move the robot slowly and collect more angle information.

Usage: `enter` collects a set of data, `d` deletes a set of data, `q` calculates the calibration result and prints it out,  `c` exit.

#### 2.4.1 Eye in Hand
```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_in_hand
```
- Collection Instructions: The camera is fixed at the end of the robotic arm, and the calibration board is placed flat on the table. Operate the robotic arm to allow the camera to recognize the calibration board on the table.

#### 2.4.2 Eye to Hand

```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_to_hand
```
- Collection Instructions: The camera is fixed at a specific position, and the calibration board is fixed at the end of the robotic arm. Operate the robotic arm to allow the camera to recognize the calibration board at the arm's end.


#### 2.4.3 Parameters

|param|type|default|Description|
|---|---|---|---|
|mode|string|eye_in_hand|hand-eye calibration mode|
|min_num|int|10|minimum number of data sets|
|piper_topic|string|piper_ctrl_node/end_pose|robotic arm's end-effector(geometry_msgs/Pose)|
|marker_topic|string|aruco_single/pose|camera-recognized calibration board pose topic（geometry_msgs/PoseStamped）|

## 3. Desktop calibration GUI

Install the system packages `python3-pyqt5`, `python3-scipy`, `python3-opencv`, and `ros-humble-cv-bridge`, then build the package using system Python rather than Conda. Keep the robot, camera, and marker detector running. The GUI only subscribes to data; it never controls or starts the robot.

```bash
source /opt/ros/humble/setup.bash
source ~/hand_eye/ros2_ws/install_system/setup.bash
ros2 run handeye_calibration_ros handeye_calibration_gui --ros-args \
  -p piper_topic:=/piper_right_ctrl_node/end_pose \
  -p marker_topic:=/aruco_single/pose \
  -p image_topic:=/aruco_single/result \
  -p mode:=eye_in_hand \
  -p result_save_path:=/home/agilex/hand_eye/results/right_wrist
```

The Chinese GUI displays live detection images, data freshness, and a sample table. It supports capture, undo, confirmed clearing, solve, and save. Connection settings are editable only while disconnected with no samples. Capture waits for new messages after the click, times out after 3 seconds, and rejects stale input. Robot `Pose` has no timestamp: pairing uses receipt time, not hardware synchronization. Keep the robot stationary until capture completes.

The default minimum is 10 samples (configurable, at least 5); use 15–25 diverse multi-axis poses. Stationary and single-axis rotation data are rejected. Saved JSON retains the CLI `position`, `orientation`, and `rpy` fields and adds the matrix, source frame, direction, configuration, and collected samples. Translation is in meters, Euler angles in radians, quaternions in XYZW order. The matrix maps camera coordinates into the input end-effector frame in `eye_in_hand`, or into the input robot base frame in `eye_to_hand`.

On this workstation, `bash ~/hand_eye/start_handeye_gui.sh` loads the system ROS environment and launches the same GUI. Adjust setup paths for other workspaces.
