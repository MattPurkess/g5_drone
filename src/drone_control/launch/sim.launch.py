from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_control')
    worlds_path = os.path.join(pkg_share, 'worlds')
    mavros_params = os.path.join(pkg_share, 'config', 'mavros_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            worlds_path
        ),

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/PX4-Autopilot && '
                'PX4_GZ_WORLD=lawn '
                'PX4_SIM_MODEL=gz_x500 '
                'make px4_sitl gz_x500'
            ],
            output='screen'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[mavros_params],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--',
                'bash', '-c',
                '~/QGroundControl-x86_64.AppImage; exec bash'
            ],
            output='screen'
        )
    ])
