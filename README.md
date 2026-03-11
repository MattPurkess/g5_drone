## How to run drone + manual control package 

Terminal 1: Launch PX4 gazebo simulation
--
```
cd ~/PX4-Autopilot  
make px4_sitl gz_x500 
```

Terminal 2: Launch MAVROS 
--
```
ros2 run mavros mavros_node \  
--ros-args -p fcu_url:=udp://:14540@localhost:14557
```

Terminal 3: Start up QGroundControl
--
```
chmod +x ./QGroundControl-x86_64.AppImage  
./QGroundControl-x86_64.AppImage
```

Terminal 4: Run the manual control ROS 2 node
--
```
cd ~/g5_drone
colcon build
source install/setup.bash 
ros2 run drone_control takeoff_land  
```
Use keyboard inputs to control arming and disarming.
> a = arm and takeoff
> d = land and disarm

Terminal 5: Start teleop keyboard node
--
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard  
```
Use holonomic mode (caps) to control NSEW movement.  
> t  = up  
> b = down  

Terminal 6 (optional): Check mavros status
--
```
ros2 topic echo /mavros/state
```



