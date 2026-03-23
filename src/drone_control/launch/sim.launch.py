from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_control')
    worlds_path = os.path.join(pkg_share, 'worlds')

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
            parameters=[{'fcu_url': 'udp://:14540@localhost:14557'}],
            output='screen'
        ),
    ])
