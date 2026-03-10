# How to run drone + control package 

Navigate to PX4 Firmware directory
--
cd ~/PX4-Autopilot

# Launch PX4 with Gazebo Classic (replace 'iris' with desired drone model)
make px4_sitl gazebo

# Start up QGroundControl
-- not sure of command --

# Source your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch MAVROS 
ros2 launch mavros mavros_launch.py 

# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch the manual control node
ros2 run drone_control takeoff_land
