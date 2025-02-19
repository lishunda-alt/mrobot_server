import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mr_joystick = Node(
        package='robot_server',
        name='robot_server',
        executable='robot_server',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(mr_joystick)
    return ld