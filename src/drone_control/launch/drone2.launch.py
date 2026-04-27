from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/PX4-Autopilot && '
                'PX4_GZ_STANDALONE=1 '
                'PX4_SYS_AUTOSTART=4001 '
                'PX4_GZ_WORLD=ELEC330Campus '
                'PX4_SIM_MODEL=gz_x500 '
                'PX4_GZ_MODEL_POSE="0,2,0.1,0,0,0" '
                './build/px4_sitl_default/bin/px4 -i 1'
            ],
            output='screen'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone2',
            parameters=[
                {
                    'fcu_url': 'udp://:14541@localhost:14557',
                    'use_sim_time': True,
                    'local_position.frame_id': 'drone2/odom',
                    'local_position.tf.send': True,
                    'local_position.tf.frame_id': 'drone2/odom',
                    'local_position.tf.child_frame_id': 'drone2/base_link',
                },
            ],
            output='screen'
        ),
    ])
