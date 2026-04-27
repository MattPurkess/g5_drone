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
        # cam to body transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_body_tf',
            # args: x y z  qx qy qz qw  parent_frame  child_frame
            # cam is 5cm below CoM, rotated 90 deg around Y
            arguments=['0', '0', '-0.05',
                    '0', '0.7071', '0', '0.7071',
                    'base_link', 'camera_down_link'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_camera_down_tf',
            arguments=[
                '--x',     '0.0',
                '--y',     '0.0',
                '--z',     '0.0',
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
               '--frame-id',       'camera_down_link',
                '--child-frame-id', 'camera_link',
            ],
            parameters=[{'use_sim_time': True}]
        ),
        
    ])
