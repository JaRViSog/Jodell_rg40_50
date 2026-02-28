from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    

    return LaunchDescription([
        Node(
            package='ros2_jodell_epg40_050_gripper_rs485_gui',
            executable='ros2_jodell_epg40_050_gripper_rs485_gui',
            name='ros2_jodell_epg40_050_gripper_rs485_gui',
            output='screen',
        )
    ])
