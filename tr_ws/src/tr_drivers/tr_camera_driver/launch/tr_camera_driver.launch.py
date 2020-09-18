import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_dir = get_package_share_directory('tr_camera_driver')
    cam_calib_file = os.path.join(pkg_dir, 'config', 'camera.yaml')

    front_camera = ComposableNode(
        name='driver',
        namespace='/tr/drivers/camera/front',
        package='usb_camera_driver',
        plugin='usb_camera_driver::CameraDriver',
        parameters=[
            {
                'image_width': 640,
                'image_height': 480,
                'camera_id': 0,
                'camera_calibration_file': 'file://' + cam_calib_file
            }
        ],
    )

    front_camera_rect = ComposableNode(
        name='rectifier',
        namespace='/tr/drivers/camera/front',
        package='image_proc',
        plugin='image_proc::RectifyNode',
    )

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='/tr/drivers/camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            front_camera,
            front_camera_rect
        ],
        output='screen'
    )

    ld.add_action(camera_container)

    return ld