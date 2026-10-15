from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('left_serial', default_value='_254522075411'),
        DeclareLaunchArgument('front_serial', default_value='_254622073589'),
        DeclareLaunchArgument('right_serial', default_value='_254522077523'),
        
        DeclareLaunchArgument('front_right_serial', default_value='_254622074071'),
        # Node(
        #     package='sensor_tools',
        #     executable='usb_camera.py',
        #     name='camera_fisheye',
        #     parameters=[{'camera_port': 50,
        #                  'camera_fps': 30,
        #                  'camera_height': 720,
        #                  'camera_width': 1280,
        #                  'camera_frame_id': "camera_fisheye_link"}],
        #     remappings=[
        #         ('/camera_rgb/color/image_raw', '/camera_fisheye/color/image_raw'),
        #         ('/camera_rgb/color/camera_info', '/camera_fisheye/color/camera_info')
        #     ],
        #     respawn=False,
        #     output='screen'
        # ),
        #左边摄像头
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
            launch_arguments={'serial_no': LaunchConfiguration('left_serial'),
                            'camera_namespace': "",
                            'camera_name': "camera_l",
                            'rgb_camera.color_profile': '1280,720,30', 
                            'depth_module.color_profile': '640,480,30', 
                            'depth_module.depth_profile': '640,480,30',
                            'depth_module.infra_profile': '640,480,30',
                            'rotation_filter.enable': 'false',
                            'rotation_filter.rotation': '0.0'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
            launch_arguments={'serial_no': LaunchConfiguration('front_serial'),
                            'camera_namespace': "",
                            'camera_name': "camera_f_l",
                            'rgb_camera.color_profile': '1280,720,30', 
                            'depth_module.color_profile': '640,480,30', 
                            'depth_module.depth_profile': '640,480,30',
                            'depth_module.infra_profile': '640,480,30',
                            'rotation_filter.enable': 'true',
                            'rotation_filter.rotation': '180.0'}.items()
            
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
            launch_arguments={'serial_no': LaunchConfiguration('front_right_serial'),
                            'camera_namespace': "",
                            'camera_name': "camera_f_r",
                            'rgb_camera.color_profile': '1280,720,30', 
                            'depth_module.color_profile': '640,480,30', 
                            'depth_module.depth_profile': '640,480,30',
                            'depth_module.infra_profile': '640,480,30',
                            'rotation_filter.enable': 'false',
                            'rotation_filter.rotation': '0.0'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
            launch_arguments={'serial_no': LaunchConfiguration('right_serial'),
                            'camera_namespace': "",
                            'camera_name': "camera_r",
                            'rgb_camera.color_profile': '1280,720,30', 
                            'depth_module.color_profile': '640,480,30', 
                            'depth_module.depth_profile': '640,480,30',
                            'depth_module.infra_profile': '640,480,30',
                            'rotation_filter.enable': 'false',
                            'rotation_filter.rotation': '0.0'}.items()
        )
    ])

