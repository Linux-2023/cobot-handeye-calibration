from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

# 3-camera, COLOR-ONLY RealSense launch for ARIO (data_tools) data collection.
#
# Why: 4x RealSense at 1280x720@30 color + 640x480@30 depth saturate the USB bus,
# so camera_l/camera_r get throttled (<25Hz) and camera_f_r drops out entirely.
# The data_tools aloha capture only consumes /camera_*/color/image_raw, so here we
#   - drop the (dead) front-right camera, and
#   - disable depth/infra/pointcloud streams,
# which frees enough bandwidth for the three color streams to run >=25Hz.
#
# Cameras kept: camera_l (left wrist), camera_f_l (front), camera_r (right wrist).
# Streams are published in each camera's native orientation.


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    # Streams to disable on every camera to save USB bandwidth (capture uses color only).
    color_only = {
        'rgb_camera.color_profile': '1280,720,30',
        'enable_color': 'true',
        'enable_depth': 'false',
        'enable_infra': 'false',
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'pointcloud.enable': 'false',
    }

    def cam(name, serial_cfg):
        args = {
            'serial_no': LaunchConfiguration(serial_cfg),
            'camera_namespace': '',
            'camera_name': name,
        }
        args.update(color_only)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rs_launch]),
            launch_arguments=args.items())

    return LaunchDescription([
        DeclareLaunchArgument('left_serial',  default_value='_254622072400'),
        DeclareLaunchArgument('front_serial', default_value='_151222073259'),
        DeclareLaunchArgument('right_serial', default_value='_254522074588'),
        cam('camera_l',   'left_serial'),
        cam('camera_f_l', 'front_serial'),
        cam('camera_r',   'right_serial'),
    ])
