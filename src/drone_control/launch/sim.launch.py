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
    survey_model_dir = os.path.join(models_path, 'x500_depth_survey')

    mavros_params = os.path.join(pkg_share, 'config', 'mavros_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge_depth.yaml')
    rtabmap_config = os.path.join(pkg_share, 'config', 'rtab_config.yaml')
    apriltag_config = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    px4_worlds_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds')
    px4_models_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')

    return LaunchDescription([

        #~ Links to PX4
        #Make this for the drone model xx bc px4 launches gazebo
        # - need camera forward & camera down - do this on x5000 depth (matt needs depth)
        
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'mkdir -p {px4_worlds_dir} && '
                f'mkdir -p {px4_models_dir} && '
                f'ln -sfn {france_world_file} {px4_worlds_dir}/france.sdf && '
                f'ln -sfn {campus_world_file} {px4_worlds_dir}/ELEC330Campus.sdf && '
                f'ln -sfn {meshes_path} {px4_worlds_dir}/meshes && '
                f'ln -sfn {survey_model_dir} {px4_models_dir}/x500_depth_survey'
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
                # 'PX4_GZ_WORLD=france '
                # 'PX4_SIM_MODEL=gz_x500_mono_cam_down '
                # 'make px4_sitl gz_x500_mono_cam_down'
                'make px4_sitl && '
                'PX4_SYS_AUTOSTART=4002 '
                'PX4_GZ_WORLD=ELEC330Campus '
                'PX4_SIM_MODEL=gz_x500_depth_survey '
                'PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" '
                './build/px4_sitl_default/bin/px4 -i 0'
            ],
            output='screen'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[mavros_params, {'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config, {'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            remappings=[
                ('/rgb/image', '/x500/rgbd/image_raw'),
                ('/rgb/camera_info', '/x500/rgbd/camera_info'),
                ('/depth/image', '/x500/rgbd/depth_image'),
                ('/scan_cloud', '/x500/scan/points'),
                ('/odom', '/mavros/local_position/odom'),
                ('/imu', '/mavros/imu/data'),
            ],
            parameters=[rtabmap_config],
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            remappings=[
                ('image_rect', '/x500/down_camera/image_raw'),
                ('camera_info', '/x500/down_camera/camera_info'),
            ],
            parameters=[apriltag_config, {'use_sim_time': True}],
            output='screen',
        ),



        # base_link -> rgbd_link (body-axes mount point on the drone)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_rgbd_tf',
            arguments=[
                '--x', '0.12',
                '--y', '0.03',
                '--z', '0.242',
                '--roll', '0',
                '--pitch', '1',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'rgbd_link'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # rgbd_link -> rgbd_optical_frame (ROS optical convention)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rgbd_to_optical_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '-1.5708',
                '--pitch', '0',
                '--yaw', '-1.5708',
                '--frame-id', 'rgbd_link',
                '--child-frame-id', 'rgbd_optical_frame'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # base_link -> camera_link (mono down-camera from merged x500_mono_cam_down)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_mono_cam_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.10',
                '--roll', '3.14159',
                '--pitch', '0',
                '--yaw', '-1.5708',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.15',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'lidar_link'
            ],
            parameters=[{'use_sim_time': True}],
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
