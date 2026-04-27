from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    #~ Path defs
    pkg_share = get_package_share_directory('drone_control')
    worlds_path = os.path.join(pkg_share, 'worlds')
    france_world_file = os.path.join(worlds_path, 'france.sdf')
    campus_world_file = os.path.join(worlds_path, 'ELEC330Campus.sdf')
    meshes_path = os.path.join(worlds_path, 'meshes')
    
    models_path = os.path.join(pkg_share, 'models')
        
    mavros_params = os.path.join(pkg_share, 'config', 'mavros_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge_depth.yaml')

    px4_worlds_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds')


    return LaunchDescription([

        #~ Links to PX4
        #Make this for the drone model xx bc px4 launches gazebo
        # - need camera forward & camera down - do this on x5000 depth (matt needs depth)
        
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'mkdir -p {px4_worlds_dir} && ' # world dest
                f'ln -sfn {france_world_file} {px4_worlds_dir}/france.sdf && ' # links world to PX4 location
                f'ln -sfn {campus_world_file} {px4_worlds_dir}/ELEC330Campus.sdf && '
                f'ln -sfn {meshes_path} {px4_worlds_dir}/meshes'
                
            ],
            output='screen'
        ),

        #~ Get apriltag sdf for gazebo world
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join(
                path for path in [
                    worlds_path,
                    models_path,
                    os.environ.get('GZ_SIM_RESOURCE_PATH', '')
                ] if path
            )
        ),

        SetEnvironmentVariable(
            name='SDF_PATH',
            value=':'.join(
                path for path in [
                    models_path,
                    os.environ.get('SDF_PATH', '')
                ] if path
            )
        ),
        #~

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/PX4-Autopilot && '
                'PX4_GZ_WORLD=ELEC330Campus '
                'PX4_SIM_MODEL=gz_x500_mono_cam_down '
                'make px4_sitl gz_x500_mono_cam_down'
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
        # publish map -> odom so RViz and other nodes see a single tree with 'map' as the root
        # temp fix, need the SLAM node to work here
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            # args: x y z  yaw pitch roll  parent_frame  child_frame
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
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
