import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    namespace = LaunchConfiguration('namespace', default='')
    return LaunchDescription([
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            output='screen',
            parameters=[
                {'frame_id': frame_id},
                {'namespace': namespace},
            ]),
        ])
