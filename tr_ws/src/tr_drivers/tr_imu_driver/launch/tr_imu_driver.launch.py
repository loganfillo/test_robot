import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    imu_driver = ComposableNode(
        name='driver',
        namespace='/tr/drivers/imu',
        package='phidgets_spatial',
        plugin='phidgets::SpatialRosI'
    )

    imu_filter = ComposableNode(
        name='filter',
        namespace='/tr/drivers/imu',
        package='imu_filter_madgwick',
        plugin='ImuFilterMadgwickRos'
    )

    imu_container = ComposableNodeContainer(
        name='imu_container',
        namespace='/tr/drivers/imu',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            imu_driver,
            imu_filter
        ],
        output='screen'
    )

    ld.add_action(imu_container)

    return ld