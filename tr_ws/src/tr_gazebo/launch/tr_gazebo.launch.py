import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    gazebo_dir = get_package_share_directory('tr_gazebo')

    world_path = os.path.join(gazebo_dir, 'worlds', 'sub_and_ball.world')

    gazebo_command = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path]
    )

    ld.add_action(gazebo_command)
    return ld