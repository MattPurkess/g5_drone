from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_control')
    config = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            remappings=[
                ('image_rect', '/drone/down_camera/image_raw'),
                ('camera_info', '/drone/down_camera/camera_info'),
            ],
            parameters=[config],
            output='screen',
        ),
    ])
