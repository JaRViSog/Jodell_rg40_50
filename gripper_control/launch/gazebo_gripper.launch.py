from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_share = FindPackageShare('gripper_control').find('gripper_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Gripper.urdf')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'gripper',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file])
            }]
        )
        
    ])
