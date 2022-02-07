import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cfg_file = os.path.join(
        get_package_share_directory('tianracer_rviz'), 'rviz_cfg', 'tianbot_slam_toolbox.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', cfg_file],
            output='screen',
        )
    ])
