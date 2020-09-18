import launch
from launch_ros.actions import Node

def generate_launch_description():

    ld = launch.LaunchDescription()

    gazebo_controller = Node(
        name='controller',
        namespace='/tr/gazebo_controller',
        package='tr_gazebo_controller',
        executable='gazebo_controller',
        remappings=[('/tr/gazebo_controller/output', '/tr/gazebo_drivers/thrusters/force')]
    )

    ld.add_action(gazebo_controller)

    return ld