import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():

    ld = launch.LaunchDescription()

    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/tr',
        package='tr_pipeline_manager',
        executable='pipeline_manager',
        output='screen'
    )

    pipeline_container = ComposableNodeContainer(
        name='pipeline',
        namespace='/tr',
        package='rclcpp_components',
        executable='component_container'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(pipeline_container)

    return ld