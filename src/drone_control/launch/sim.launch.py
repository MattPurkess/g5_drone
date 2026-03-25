from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_control')
    worlds_path = os.path.join(pkg_share, 'worlds')
    world_file = os.path.join(worlds_path, 'france.sdf')
    heightmaps_path = os.path.join(worlds_path, 'heightmaps')
    meshes_path = os.path.join(worlds_path, 'meshes')
        
    mavros_params = os.path.join(pkg_share, 'config', 'mavros_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge_depth.yaml')

    px4_worlds_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'mkdir -p {px4_worlds_dir} && '
                f'ln -sfn {world_file} {px4_worlds_dir}/france.sdf && '
                f'ln -sfn {heightmaps_path} {px4_worlds_dir}/heightmaps && '
                f'ln -sfn {meshes_path} {px4_worlds_dir}/meshes'
            ],
            output='screen'
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=worlds_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/PX4-Autopilot && '
                'PX4_GZ_WORLD=france '
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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
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
