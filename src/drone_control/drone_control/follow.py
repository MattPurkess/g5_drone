#!/usr/bin/env python3
"""
follow.py — drone2 hovers HEIGHT_OFFSET metres above drone1, tracking drone1
in real time as it moves.

Usage:
    ros2 launch drone_control sim.launch.py
    ros2 launch drone_control drone2.launch.py
    ros2 run drone_control follow
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


# World-frame spawn poses (must match the launch files).
DRONE1_SPAWN_WORLD = (0.0, 0.0, 0.1)
DRONE2_SPAWN_WORLD = (0.0, 2.0, 0.1)

HEIGHT_OFFSET = 2.0          # metres above drone1's current Z

SETPOINT_HZ = 20
DT = 1.0 / SETPOINT_HZ
LOG_INTERVAL = 5.0
PRESTREAM_SECONDS = 2.0


class FollowNode(Node):

    def __init__(self):
        super().__init__('follow_node')

        self.leader_pose = None
        self.follower_pose = None
        self.follower_state = State()

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            lambda m: setattr(self, 'leader_pose', m),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/drone2/local_position/pose',
            lambda m: setattr(self, 'follower_pose', m),
            qos_profile_sensor_data)
        self.create_subscription(
            State, '/drone2/state',
            lambda m: setattr(self, 'follower_state', m),
            qos_profile_sensor_data)

        self.sp_pub = self.create_publisher(
            PoseStamped, '/drone2/setpoint_position/local', 10)

        self.arm_client = self.create_client(CommandBool, '/drone2/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/drone2/set_mode')

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

        # Pre-stream at drone2's current XY at a safe altitude so OFFBOARD-on-arm
        # climbs vertically before chasing drone1.
        fx = self.follower_pose.pose.position.x
        fy = self.follower_pose.pose.position.y
        fz = self.follower_pose.pose.position.z
        prestream_z = max(fz, HEIGHT_OFFSET + 1.0)

        self.get_logger().info('Pre-streaming setpoints...')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < PRESTREAM_SECONDS:
            self._publish_target(fx, fy, prestream_z)
            self._spin()

        if not self._set_mode('OFFBOARD'):
            self.get_logger().error('Could not set drone2 to OFFBOARD')
            return

        if not self._arm(True):
            self.get_logger().error('Could not arm drone2')
            return

        self.get_logger().info(
            f'Following drone1 at +{HEIGHT_OFFSET:.1f} m altitude.'
        )
        last_log = time.time()

        while rclpy.ok():
            # Recompute target every loop from drone1's *current* pose.
            d1_wx = self.leader_pose.pose.position.x + DRONE1_SPAWN_WORLD[0]
            d1_wy = self.leader_pose.pose.position.y + DRONE1_SPAWN_WORLD[1]
            d1_wz = self.leader_pose.pose.position.z + DRONE1_SPAWN_WORLD[2]

            tx = d1_wx - DRONE2_SPAWN_WORLD[0]
            ty = d1_wy - DRONE2_SPAWN_WORLD[1]
            tz = (d1_wz + HEIGHT_OFFSET) - DRONE2_SPAWN_WORLD[2]

            self._publish_target(tx, ty, tz)
            self._spin()

            now = time.time()
            if now - last_log >= LOG_INTERVAL:
                d2_wx = self.follower_pose.pose.position.x + DRONE2_SPAWN_WORLD[0]
                d2_wy = self.follower_pose.pose.position.y + DRONE2_SPAWN_WORLD[1]
                d2_wz = self.follower_pose.pose.position.z + DRONE2_SPAWN_WORLD[2]
                xy_gap = math.hypot(d1_wx - d2_wx, d1_wy - d2_wy)
                z_gap = d2_wz - d1_wz
                self.get_logger().info(
                    f'd1=({d1_wx:+.1f}, {d1_wy:+.1f}, {d1_wz:.1f})  '
                    f'd2=({d2_wx:+.1f}, {d2_wy:+.1f}, {d2_wz:.1f})  '
                    f'xy_gap={xy_gap:.2f}  z_gap={z_gap:+.2f}'
                )
                last_log = now


def main():
    rclpy.init()
    node = FollowNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
