import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_jodell_epg40_050_gripper_rs485_interface',  
            executable='ros2_jodell_epg40_050_gripper_rs485_interface', 
            name='ros2_jodell_epg40_050_gripper_rs485_interface',
        )
    ])
