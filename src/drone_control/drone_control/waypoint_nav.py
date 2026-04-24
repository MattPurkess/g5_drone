#!/usr/bin/env python3
"""
Autonomous waypoint navigation node for Asssignment 3. 
Launch this first : ros2 launch drone_control sim.launch.py

Then once terminal indicates TAKEOFF READY, then run this node:
    ros2 run drone_control waypoint_nav

The node arms, climbs to cruise altitude, visits each waypoint in MISSION,
dwells at each one, then returns to origin and lands via AUTO.LAND.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from dataclasses import dataclass, field
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]  # [x, y, z, w]

# Mission definition
# Edit these waypoints freely.  Coordinates are ENU local frame:
#   +X = East,  +Y = North,  +Z = Up
#   Origin (0, 0, 0) = drone spawn point on the france.sdf platform.
#   yaw is in radians: 0 = face East, pi/2 = face North

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0       # radians
    dwell: float = 5.0     # seconds to hold position at this waypoint
    name: str = ''


MISSION = [
    Waypoint(  0.0,   0.0, 10.0, yaw=0.0,            dwell=3.0,  name='Home (hover)'),
    Waypoint( 10.0,   0.0, 10.0, yaw=0.0,            dwell=5.0,  name='WP1 – East'),
    Waypoint( 10.0,  10.0, 10.0, yaw=math.pi / 2,   dwell=5.0,  name='WP2 – North-East'),
    Waypoint(  0.0,  10.0, 10.0, yaw=math.pi,        dwell=5.0,  name='WP3 – North'),
    Waypoint(  0.0,   0.0, 10.0, yaw=-math.pi / 2,  dwell=5.0,  name='Return to origin'),
]

# ------------------
# Tuning parameters
# ------------------
CRUISE_ALT      = 5.0   # metres – default takeoff and cruise altitude
ACCEPT_RADIUS   = 1.0    # metres – 3D distance to consider waypoint reached
SETPOINT_HZ     = 20     # Hz – must be >= 10 for PX4 Offboard mode
LAND_TIMEOUT    = 60     # max wait for AUTO.LAND to complete in seconds
FLY_TIMEOUT     = 60     # bmax travel time per waypoint before warning in seconds
#------------------

class WaypointNavNode(Node):
    """Fly the drone through MISSION then land."""

    def __init__(self):
        super().__init__('waypoint_nav_node')

        # ── state ────────────────────────────────────────────────────────────
        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.have_pose    = False

        # ── subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            State, '/mavros/state',
            lambda m: setattr(self, 'mavros_state', m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_cb,
            qos_profile_sensor_data,
        )

        # ── publishers ───────────────────────────────────────────────────────
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # ── service clients ──────────────────────────────────────────────────
        self.arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL,  '/mavros/cmd/land')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        self.have_pose    = True

    # ── helpers ──────────────────────────────────────────────────────────────

    def _spin(self, duration: float = 0.05):
        """Process callbacks for `duration` seconds."""
        rclpy.spin_once(self, timeout_sec=duration)

    def _call_service(self, client, request, timeout_sec: float = 5.0):
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        res = self._call_service(self.mode_client, req)
        ok = res is not None and res.mode_sent
        if ok:
            self.get_logger().info(f'Mode set: {mode}')
        else:
            self.get_logger().error(f'Failed to set mode: {mode}')
        return ok

    def _arm(self, value: bool) -> bool:
        req = CommandBool.Request()
        req.value = value
        res = self._call_service(self.arm_client, req)
        ok = res is not None and res.success
        state = 'Armed' if value else 'Disarmed'
        if ok:
            self.get_logger().info(state)
        else:
            self.get_logger().error(f'Failed: {state}')
        return ok

    def _dist_to(self, wp: Waypoint) -> float:
        """3-D Euclidean distance from current pose to waypoint."""
        dx = self.current_pose.pose.position.x - wp.x
        dy = self.current_pose.pose.position.y - wp.y
        dz = self.current_pose.pose.position.z - wp.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _make_setpoint(self, x: float, y: float, z: float,
                       yaw: float = 0.0) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])
        return ps

    def _publish_wp(self, wp: Waypoint):
        self.sp_pub.publish(self._make_setpoint(wp.x, wp.y, wp.z, wp.yaw))

    def _fly_to(self, wp: Waypoint):
        """Publish setpoint until within ACCEPT_RADIUS or timeout."""
        self.get_logger().info(
            f'  -> {wp.name}  ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f} m)'
        )
        start = time.time()
        while rclpy.ok():
            self._publish_wp(wp)
            self._spin(1.0 / SETPOINT_HZ)
            dist = self._dist_to(wp)
            if dist < ACCEPT_RADIUS:
                self.get_logger().info(f'     Reached {wp.name} (err {dist:.2f} m)')
                return
            elapsed = time.time() - start
            if elapsed > FLY_TIMEOUT:
                self.get_logger().warn(
                    f'     Timeout flying to {wp.name} (dist {dist:.2f} m) — continuing'
                )
                return
            # Log progress every 5 s
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                self.get_logger().info(
                    f'     {wp.name}: {dist:.2f} m remaining …'
                )

    def _dwell(self, wp: Waypoint):
        """Hold position at waypoint for wp.dwell seconds."""
        self.get_logger().info(f'     Dwelling {wp.dwell:.0f} s at {wp.name}')
        end = time.time() + wp.dwell
        while rclpy.ok() and time.time() < end:
            self._publish_wp(wp)
            self._spin(1.0 / SETPOINT_HZ)

    # ── main sequence ────────────────────────────────────────────────────────

    def run(self):
        mission_start = time.time()

        # 1. Wait for MAVROS connection
        self.get_logger().info('Waiting for MAVROS connection …')
        while rclpy.ok() and not self.mavros_state.connected:
            self._spin(0.1)
        self.get_logger().info('Connected.')

        # 2. Wait for first pose
        self.get_logger().info('Waiting for pose …')
        while rclpy.ok() and not self.have_pose:
            self._spin(0.1)
        self.get_logger().info(
            f'Pose received: ({self.current_pose.pose.position.x:.2f}, '
            f'{self.current_pose.pose.position.y:.2f}, '
            f'{self.current_pose.pose.position.z:.2f})'
        )

        # 3. Pre-stream setpoints: PX4 requires setpoints already flowing
        #    before it will accept an OFFBOARD mode switch
        self.get_logger().info('Pre-streaming setpoints (2 s) …')
        first_wp = MISSION[0]
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 2.0:
            self._publish_wp(first_wp)
            self._spin(1.0 / SETPOINT_HZ)

        # 4. Switch to OFFBOARD and arm
        if not self._set_mode('OFFBOARD'):
            self.get_logger().error('Aborting — could not set OFFBOARD mode.')
            return
        if not self._arm(True):
            self.get_logger().error('Aborting — could not arm.')
            return

        # 5. Climb to cruise altitude before starting mission
        self.get_logger().info(f'Climbing to {CRUISE_ALT} m …')
        climb_wp = Waypoint(
            x=self.current_pose.pose.position.x,
            y=self.current_pose.pose.position.y,
            z=CRUISE_ALT,
            name='Climb',
        )
        while rclpy.ok() and \
                abs(self.current_pose.pose.position.z - CRUISE_ALT) > 0.3:
            self._publish_wp(climb_wp)
            self._spin(1.0 / SETPOINT_HZ)
        self.get_logger().info('Cruise altitude reached.')

        # 6. Execute mission
        self.get_logger().info(
            f'Starting mission — {len(MISSION)} waypoints'
        )
        for i, wp in enumerate(MISSION, start=1):
            self.get_logger().info(f'Waypoint {i}/{len(MISSION)}: {wp.name}')
            self._fly_to(wp)
            self._dwell(wp)

        self.get_logger().info('Mission complete. Returning to origin …')

        # 7. Return to origin at cruise altitude, then descend slowly
        origin_cruise = Waypoint(0.0, 0.0, CRUISE_ALT, name='Origin (cruise)')
        self._fly_to(origin_cruise)
        self._dwell(origin_cruise)

        # 8. Land via AUTO.LAND — same approach as takeoff_land.py
        self.get_logger().info('Switching to AUTO.LAND …')
        if not self._set_mode('AUTO.LAND'):
            self.get_logger().error('Could not switch to AUTO.LAND — manual intervention needed')
            return

        # Wait for PX4 to detect touchdown and auto-disarm
        self.get_logger().info('Landing … waiting for disarm')
        t_land = time.time()
        while rclpy.ok() and (time.time() - t_land) < LAND_TIMEOUT:
            self._spin(0.1)
            if not self.mavros_state.armed:
                self.get_logger().info('Landed and disarmed successfully.')
                break
        else:
            self.get_logger().warn('Land timeout — drone may still be airborne.')

        elapsed = time.time() - mission_start
        self.get_logger().info(f'Total flight time: {elapsed:.1f} s')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
