# Drone Control Package

This ROS2 package provides takeoff, landing, and teleoperation control for drones using MAVROS.

## Features

- Automatic arming and takeoff to a specified hover altitude when prompted
- Teleoperation control via `/cmd_vel` topic
- Up/Down movement functionality
- Automatic landing and disarming when prompted

## Topics

### Input Topics

- `/cmd_vel` (geometry_msgs/Twist): Teleoperation velocity commands
- `/move_up` (std_msgs/Float32): Move up by specified distance in meters
- `/move_down` (std_msgs/Float32): Move down by specified distance in meters
- `/set_vertical_velocity` (std_msgs/Float32): Set vertical velocity (positive = up, negative = down) in m/s

### Services

- `/stop_vertical` (std_srvs/Trigger): Stop vertical movement

## Usage

1. Launch the drone control node:
   ```bash
   ros2 run drone_control takeoff_land
   ```

2. The option to arm the drone and takeoff using "a" will be given. After pressing "a", the drone will automatically take off to 5m altitude and enter teleop mode.

3. Use the following commands for up/down control:

   **Move up by 2 meters:**
   ```bash
   ros2 topic pub /move_up std_msgs/Float32 "data: 2.0"
   ```

   **Move down by 1 meter:**
   ```bash
   ros2 topic pub /move_down std_msgs/Float32 "data: 1.0"
   ```

   **Set upward velocity of 0.5 m/s:**
   ```bash
   ros2 topic pub /set_vertical_velocity std_msgs/Float32 "data: 0.5"
   ```

   **Set downward velocity of 0.3 m/s:**
   ```bash
   ros2 topic pub /set_vertical_velocity std_msgs/Float32 "data: -0.3"
   ```

   **Stop vertical movement:**
   ```bash
   ros2 service call /stop_vertical std_srvs/Trigger
   ```

4. To land and disarm the drone, press "d" within the terminal. This will automatically land the drone using AUTO.LAND mode and then disarm the drone.

## Teleoperation

In teleop mode, you can also control the drone using `/cmd_vel` with:
- `linear.x`: Forward/backward velocity
- `linear.y`: Left/right velocity  
- `linear.z`: Up/down velocity
- `angular.z`: Yaw rotation

## Parameters

- `hover_alt`: Hover altitude in meters (default: 5.0)</content>
<parameter name="filePath">/home/matt/group5_drone_ws/src/drone_control/README.md
