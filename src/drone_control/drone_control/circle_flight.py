#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0
    dwell: float = 5.0
    name: str = ''


# Mission parameters
TARGET_ALTITUDE = 8.0
CENTER_X = 0.0
CENTER_Y = 0.0
RADIUS = 10.0
LINEAR_SPEED = 2.0
CIRCLE_LAPS = 1.0

MISSION = [
    Waypoint(0.0, 0.0, TARGET_ALTITUDE, name='Centre'),
]

# Tuning
CRUISE_ALT = TARGET_ALTITUDE
ACCEPT_RADIUS = 0.1
SETPOINT_HZ = 20
DT = 1.0 / SETPOINT_HZ
LAND_TIMEOUT = 60
FLY_TIMEOUT = 60


class WaypointNavNode(Node):

    def __init__(self):
        super().__init__('waypoint_nav_node')

        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.have_pose = False

        self.create_subscription(State, '/mavros/state',
                                 lambda m: setattr(self, 'mavros_state', m),
                                 qos_profile_sensor_data)

        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._pose_cb,
                                 qos_profile_sensor_data)

        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def _pose_cb(self, msg):
        self.current_pose = msg
        self.have_pose = True

    def _spin(self, duration=DT):
        rclpy.spin_once(self, timeout_sec=duration)

    def _call_service(self, client, request):
        client.wait_for_service()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        res = self._call_service(self.mode_client, req)
        return res.mode_sent

    def _arm(self, val):
        req = CommandBool.Request()
        req.value = val
        res = self._call_service(self.arm_client, req)
        return res.success

    # âœ… FIXED: all values explicitly cast to float
    def _make_sp(self, x, y, z, yaw=0.0):
        ps = PoseStamped()

        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'

        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])

        return ps

    def _publish(self, x, y, z, yaw=0.0):
        self.sp_pub.publish(self._make_sp(x, y, z, yaw))

    def _dist(self, wp):
        dx = self.current_pose.pose.position.x - wp.x
        dy = self.current_pose.pose.position.y - wp.y
        dz = self.current_pose.pose.position.z - wp.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _fly_to(self, wp):
        start = time.time()
        while rclpy.ok():
            self._publish(wp.x, wp.y, wp.z, wp.yaw)
            self._spin()

            if self._dist(wp) < ACCEPT_RADIUS:
                return

            if time.time() - start > FLY_TIMEOUT:
                self.get_logger().warn("Fly timeout")
                return

    def _dwell(self, wp):
        end = time.time() + wp.dwell
        while time.time() < end:
            self._publish(wp.x, wp.y, wp.z, wp.yaw)
            self._spin()

    def _fly_circle(self):
        omega = LINEAR_SPEED / RADIUS
        d_theta = omega * DT
        theta = 0.0
        final = 2.0 * math.pi * CIRCLE_LAPS

        while theta <= final:
            x = CENTER_X + RADIUS * math.cos(theta)
            y = CENTER_Y + RADIUS * math.sin(theta)
            z = TARGET_ALTITUDE
            yaw = theta + math.pi

            self._publish(x, y, z, yaw)
            self._spin()

            theta += d_theta

    def run(self):

        while not self.mavros_state.connected:
            self._spin(0.1)

        while not self.have_pose:
            self._spin(0.1)

        # âœ… FIXED: floats here too
        t0 = time.time()
        while time.time() - t0 < 2:
            self._publish(0.0, 0.0, TARGET_ALTITUDE)
            self._spin()

        self._set_mode('OFFBOARD')
        self._arm(True)

        # climb (safe)
        climb_wp = Waypoint(
            float(self.current_pose.pose.position.x),
            float(self.current_pose.pose.position.y),
            float(CRUISE_ALT),
            name='Climb'
        )

        self._fly_to(climb_wp)

        # centre
        self._fly_to(MISSION[0])
        self._dwell(MISSION[0])

        # move to circle start
        start_wp = Waypoint(
            CENTER_X + RADIUS,
            CENTER_Y,
            TARGET_ALTITUDE,
            name='Circle start'
        )

        self._fly_to(start_wp)

        # circle
        self._fly_circle()

        # land
        self._set_mode('AUTO.LAND')


def main():
    rclpy.init()
    node = WaypointNavNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



