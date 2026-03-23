# Drone Control Package

## Usage

1. Build the workspace and source it:
```
   cd ~/group5_drone_ws
   colcon build
   source install/setup.bash
```

2. Launch the simulation:
```
ros2 launch drone_control sim.launch.py
```
This launches the Gazebo world, PX4, QGroundControl and MAVROS

3. In a separate terminal run the manual control node:
```
ros2 run drone_control takeoff_land
```
In the control terminal:
Press a to arm and take off
Press d to land and disarm

4. Use the teleop in another terminal to move the drone
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use "Holonomic mode" (caps lock) to control the drone's NSEW movement

