import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('hikrobot_camera')
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file, description='Path to config file'),
        DeclareLaunchArgument('camera_ip', default_value='', description='Target camera IP'),
        DeclareLaunchArgument('camera_sn', default_value='', description='Target camera SN'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw', description='Image topic name'),

        # 【新增】发布静态 TF 变换：从 map 到 camera_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_optical_frame']
        ),

        Node(
            package='hikrobot_camera',
            executable='camera_node',
            name='hikrobot_camera_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'camera_ip': LaunchConfiguration('camera_ip'),
                    'camera_sn': LaunchConfiguration('camera_sn'),
                    'image_topic': LaunchConfiguration('image_topic')
                }
            ]
        )
    ])