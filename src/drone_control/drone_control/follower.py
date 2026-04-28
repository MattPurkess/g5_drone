#!/usr/bin/env python3
"""
Follower node for drone2.

Reads drone1's local pose, then streams setpoints to drone2 such that drone2
hovers 1 m above drone1's current XY in real time. The commanded XY is also
clamped to lie within MAX_XY_OFFSET of drone1's XY as a defensive bound.

Run AFTER both sims and drone2 MAVROS are up:
    ros2 launch drone_control sim.launch.py
    ros2 launch drone_control drone2.launch.py
    ros2 run drone_control follower
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


# Tuning
HOVER_OFFSET_Z = 2.0      # metres above drone1
MAX_XY_OFFSET = 5.0       # metres, hard cap on commanded XY distance from drone1
SETPOINT_HZ = 20
DT = 1.0 / SETPOINT_HZ
LOG_INTERVAL = 5.0

# Each PX4 instance publishes pose in its own local frame, with origin at that
# drone's spawn pose. We translate drone1's pose into drone2's local frame by
# adding (drone1_spawn - drone2_spawn). Update these if the launch poses change.
DRONE1_SPAWN_WORLD = (0.0, 0.0, 0.1)   # sim.launch.py:    PX4_GZ_MODEL_POSE
DRONE2_SPAWN_WORLD = (0.0, 2.0, 0.1)   # drone2.launch.py: PX4_GZ_MODEL_POSE
LEADER_TO_FOLLOWER_OFFSET = (
    DRONE1_SPAWN_WORLD[0] - DRONE2_SPAWN_WORLD[0],
    DRONE1_SPAWN_WORLD[1] - DRONE2_SPAWN_WORLD[1],
    DRONE1_SPAWN_WORLD[2] - DRONE2_SPAWN_WORLD[2],
)
2

class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower_node')

        self.leader_pose = None
        self.follower_pose = None
        self.follower_state = State()

        # Leader = drone1 (un-namespaced MAVROS topics).
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._leader_cb, qos_profile_sensor_data)

        # Follower = drone2. Its mavros launch uses namespace='drone2', which
        # overrides mavros's default /mavros group — plugin topics live
        # directly under /drone2/..., NOT /drone2/mavros/...
        self.create_subscription(
            PoseStamped, '/drone2/local_position/pose',
            self._follower_cb, qos_profile_sensor_data)
        self.create_subscription(
            State, '/drone2/state',
            lambda m: setattr(self, 'follower_state', m),
            qos_profile_sensor_data)

        self.sp_pub = self.create_publisher(
            PoseStamped, '/drone2/setpoint_position/local', 10)

        self.arm_client = self.create_client(CommandBool, '/drone2/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/drone2/set_mode')

    def _leader_cb(self, msg):
        self.leader_pose = msg

    def _follower_cb(self, msg):
        self.follower_pose = msg

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
        return res is not None and res.mode_sent

    def _arm(self, val):
        req = CommandBool.Request()
        req.value = val
        res = self._call_service(self.arm_client, req)
        return res is not None and res.success

    def _publish_target(self, x, y, z):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self.sp_pub.publish(ps)

    def _compute_target(self):
        # Drone1's pose in drone1's local frame.
        lx = self.leader_pose.pose.position.x
        ly = self.leader_pose.pose.position.y
        lz = self.leader_pose.pose.position.z

        # Translate into drone2's local frame.
        ox, oy, oz = LEADER_TO_FOLLOWER_OFFSET
        tx = lx + ox
        ty = ly + oy
        tz = lz + oz + HOVER_OFFSET_Z

        # Defensive clamp so the commanded XY is never more than MAX_XY_OFFSET
        # from drone1's translated XY (unreachable with the default target,
        # but cheap insurance if the logic is ever extended).
        leader_in_follower_frame_x = lx + ox
        leader_in_follower_frame_y = ly + oy
        dx = tx - leader_in_follower_frame_x
        dy = ty - leader_in_follower_frame_y
        d = math.hypot(dx, dy)
        if d > MAX_XY_OFFSET:
            scale = MAX_XY_OFFSET / d
            tx = leader_in_follower_frame_x + dx * scale
            ty = leader_in_follower_frame_y + dy * scale

        return tx, ty, tz

    def run(self):
        self.get_logger().info('Waiting for drone1 pose...')
        while rclpy.ok() and self.leader_pose is None:
            self._spin(0.1)

        self.get_logger().info('Waiting for drone2 pose...')
        while rclpy.ok() and self.follower_pose is None:
            self._spin(0.1)

        self.get_logger().info('Waiting for drone2 MAVROS connection...')
        while rclpy.ok() and not self.follower_state.connected:
            self._spin(0.1)

        # Pre-stream setpoints at drone2's current XY at altitude
        # (leader.z + 1 m, or drone2's current z, whichever is higher) so
        # OFFBOARD-on-arm climbs vertically rather than darting horizontally.
        fx = self.follower_pose.pose.position.x
        fy = self.follower_pose.pose.position.y
        fz = self.follower_pose.pose.position.z
        prestream_z = max(fz, self.leader_pose.pose.position.z + HOVER_OFFSET_Z)

        self.get_logger().info('Pre-streaming setpoints...')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 2.0:
            self._publish_target(fx, fy, prestream_z)
            self._spin()

        if not self._set_mode('OFFBOARD'):
            self.get_logger().error('Could not set drone2 to OFFBOARD')
            return

        if not self._arm(True):
            self.get_logger().error('Could not arm drone2')
            return

        self.get_logger().info('Following drone1 — Ctrl-C to stop.')
        last_log = time.time()

        while rclpy.ok():
            tx, ty, tz = self._compute_target()
            self._publish_target(tx, ty, tz)
            self._spin()

            now = time.time()
            if now - last_log >= LOG_INTERVAL:
                # Bring everything into drone2's frame for a fair comparison.
                ox, oy, _ = LEADER_TO_FOLLOWER_OFFSET
                lx = self.leader_pose.pose.position.x + ox
                ly = self.leader_pose.pose.position.y + oy
                fx = self.follower_pose.pose.position.x
                fy = self.follower_pose.pose.position.y
                fz = self.follower_pose.pose.position.z
                d_xy = math.hypot(fx - lx, fy - ly)
                self.get_logger().info(
                    f'leader_in_d2=({lx:.1f}, {ly:.1f})  '
                    f'follower=({fx:.1f}, {fy:.1f}, {fz:.1f})  '
                    f'xy_gap={d_xy:.2f} m'
                )
                last_log = now


def main():
    rclpy.init()
    node = FollowerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
