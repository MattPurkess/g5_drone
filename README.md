# G5 Drone — Autonomous Waypoint Navigation

## Overview

This branch adds an autonomous waypoint navigation node to the G5 drone package.
The drone arms, climbs to cruise altitude, visits a sequence of predefined waypoints,
dwells at each one, returns to the origin, and lands automatically — no manual input required.

---

## Requirements

- ROS 2 Jazzy
- PX4 Autopilot (SITL) with Gazebo Harmonic
- MAVROS2 (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
- QGroundControl (`~/QGroundControl-x86_64.AppImage`)

---

## Build

```bash
cd ~/g5_drone
colcon build --packages-select drone_control
source install/setup.bash
```

---

## Running the Full System

### Step 1 — Launch the simulation

Open a terminal and run:

```bash
cd ~/g5_drone
source install/setup.bash
ros2 launch drone_control sim.launch.py
```

This starts Gazebo, PX4 SITL, MAVROS, QGroundControl and RViz together.
Wait until:
- The drone is visible on the spawn platform in Gazebo
- QGroundControl shows **3D GPS Lock**
- RViz shows the TF frames

This can take up to 60 seconds on first launch as PX4 compiles.

### Step 2 — Run the waypoint node

Open a second terminal:

```bash
cd ~/g5_drone
source install/setup.bash
ros2 run drone_control waypoint_nav
```

The node will:
1. Wait for MAVROS connection
2. Pre-stream setpoints for 2 seconds
3. Switch to OFFBOARD mode and arm
4. Climb to cruise altitude
5. Visit each waypoint in sequence, dwelling at each one
6. Return to origin and land via AUTO.LAND

### Step 3 — (Optional) Record a bag for analysis

In a third terminal, start recording before the mission begins:

```bash
cd ~/g5_drone
ros2 bag record -o waypoint_mission_$(date +%H%M%S) \
    /mavros/local_position/pose \
    /mavros/state \
    /mavros/setpoint_position/local
```

---

## Mission Configuration

Edit the `MISSION` list at the top of `src/drone_control/drone_control/waypoint_nav.py`
to change the waypoints. Coordinates are in ENU local frame:

- `+X` = East (metres from spawn)
- `+Y` = North (metres from spawn)
- `+Z` = Altitude (metres)
- `yaw` = heading in radians (0 = East, π/2 = North)
- `dwell` = seconds to hold position at each waypoint

```python
MISSION = [
    Waypoint(  0.0,  0.0, 10.0, yaw=0.0,          dwell=3.0, name='Home'),
    Waypoint( 10.0,  0.0, 10.0, yaw=0.0,          dwell=5.0, name='WP1 – East'),
    Waypoint( 10.0, 10.0, 10.0, yaw=math.pi/2,    dwell=5.0, name='WP2 – North-East'),
    Waypoint(  0.0, 10.0, 10.0, yaw=math.pi,      dwell=5.0, name='WP3 – North'),
    Waypoint(  0.0,  0.0, 10.0, yaw=-math.pi/2,   dwell=5.0, name='Return'),
]
```

After editing, rebuild before running:

```bash
cd ~/g5_drone
colcon build --packages-select drone_control
source install/setup.bash
```

Key tuning parameters at the top of `waypoint_nav.py`:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `CRUISE_ALT` | 5.0 m | Initial climb altitude before mission starts |
| `ACCEPT_RADIUS` | 1.0 m | 3D distance to consider a waypoint reached |
| `SETPOINT_HZ` | 20 | Setpoint publish rate — do not go below 10 |
| `FLY_TIMEOUT` | 60 s | Max time per waypoint leg before warning |

---

## Trajectory Analysis

Three bags were recorded at different acceptance radii for comparison.
To regenerate the plots:

```bash
cd ~/g5_drone
python3 scripts/plot_mission.py
```

Outputs:
- `waypoint_trajectory.png` — planned vs actual top-down path comparison
- `waypoint_altitude.png` — altitude profile for each run

---

## Troubleshooting

**Drone fails to arm**
- Wait longer after launch for GPS lock — QGroundControl must show 3D GPS
- In the PX4 terminal at the `pxh>` prompt, run `commander check` to see which pre-arm check is failing
- If running without QGroundControl, first set: `param set COM_RCL_EXCEPT 4`

**MAVROS crashes mid-flight with `Promise already satisfied`**
- Known MAVROS2 race condition on CPU-limited machines
- Relaunch the sim and try again — it is intermittent
- Reducing mission size (shorter legs, lower altitude) reduces exposure time

**Mode set: OFFBOARD fails**
- Setpoints must stream for at least 2 seconds before the mode switch
- Check `ros2 topic hz /mavros/setpoint_position/local` — should show ~20 Hz

**Drone reaches wrong positions**
- Verify coordinate frame: +X is East, +Y is North from the spawn point
- Check `ros2 topic echo /mavros/local_position/pose` to confirm pose is reading correctly

---

## Package Structure

```
src/drone_control/
├── drone_control/
│   ├── takeoff_land.py      # Manual arm/teleop/land node (keyboard)
│   └── waypoint_nav.py      # Autonomous waypoint navigation node (this branch)
├── launch/
│   └── sim.launch.py        # Full simulation launch
├── config/
│   ├── mavros_params.yaml
│   ├── gz_bridge_depth.yaml
│   └── rviz_config.rviz
├── worlds/
│   └── france.sdf
scripts/
└── plot_mission.py          # Trajectory analysis and plotting
```

# CMD's to install correct packages for apriltag
### Python libraries
pip3 install pupil-apriltags transformations Pillow numpy --break-system-packages

### ROS2 AprilTag packages
sudo apt install -y \
  ros-jazzy-apriltag-ros \
  ros-jazzy-apriltag-msgs \
  ros-jazzy-tf-transformations \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-ros
  