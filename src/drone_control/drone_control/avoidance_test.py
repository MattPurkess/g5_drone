#!/usr/bin/env python3
"""
avoidance_test.py — Test that PX4 collision prevention halts the drone before
hitting a known obstacle.

Sequence:
  1. Arm and takeoff to TEST_ALT
  2. Command a setpoint TARGET_X metres straight ahead (deliberately past a
     known obstacle — e.g. one of the pine trees in front of spawn)
  3. Stream that setpoint and watch the drone's actual progress
  4. After TEST_DURATION seconds, log the maximum X reached and land

If collision prevention is working, the drone stops near the obstacle
(distance ~ CP_DIST, default 3 m) rather than reaching the commanded X.

Prereqs:
  ros2 run drone_control lidar_to_obstacle    # in another terminal
  ros2 run drone_control enable_avoidance     # one-shot, then exits

Usage:
  ros2 run drone_control avoidance_test
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


TEST_ALT = 5.0           # metres
TARGET_X = 30.0          # metres east of spawn (well past any obstacle)
TEST_DURATION = 30.0     # seconds to give the drone to try
LOG_INTERVAL = 2.0


class AvoidanceTest(Node):

    def __init__(self):
        super().__init__('avoidance_test')

        self.state = State()
        self.pose = PoseStamped()
        self.nearest_obstacle = float('inf')

        self.create_subscription(
            State, '/mavros/state',
            lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            lambda m: setattr(self, 'pose', m), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/mavros/obstacle/send',
            self._scan_cb, qos_profile_sensor_data)

        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

    def _scan_cb(self, msg: LaserScan):
        finite = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        self.nearest_obstacle = min(finite) if finite else float('inf')

    def _spin(self, dt=0.05):
        rclpy.spin_once(self, timeout_sec=dt)

    def _publish(self, x, y, z):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self.sp_pub.publish(ps)

    def _call(self, client, req):
        client.wait_for_service()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result()

    def _set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        res = self._call(self.mode_client, req)
        return res is not None and res.mode_sent

    def _arm(self, val):
        req = CommandBool.Request()
        req.value = val
        res = self._call(self.arm_client, req)
        return res is not None and res.success

    def run(self):
        self.get_logger().info('Waiting for MAVROS connection...')
        while rclpy.ok() and not self.state.connected:
            self._spin(0.1)

        # Pre-stream
        self.get_logger().info('Pre-streaming setpoints...')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 2.0:
            self._publish(0.0, 0.0, TEST_ALT)
            self._spin()

        if not self._set_mode('OFFBOARD'):
            self.get_logger().error('OFFBOARD failed')
            return
        if not self._arm(True):
            self.get_logger().error('Arm failed')
            return

        # Climb
        self.get_logger().info(f'Climbing to {TEST_ALT} m...')
        while rclpy.ok() and abs(self.pose.pose.position.z - TEST_ALT) > 0.5:
            self._publish(0.0, 0.0, TEST_ALT)
            self._spin()

        # Drive setpoint forward into expected obstacle
        self.get_logger().info(
            f'Commanding setpoint to (X={TARGET_X}, Y=0, Z={TEST_ALT}). '
            f'CP should halt drone before TARGET_X.'
        )
        max_x = self.pose.pose.position.x
        start = time.time()
        last_log = start

        while rclpy.ok() and time.time() - start < TEST_DURATION:
            self._publish(TARGET_X, 0.0, TEST_ALT)
            self._spin()
            x = self.pose.pose.position.x
            if x > max_x:
                max_x = x

            now = time.time()
            if now - last_log >= LOG_INTERVAL:
                if math.isfinite(self.nearest_obstacle):
                    nearest = f'{self.nearest_obstacle:.2f} m'
                else:
                    nearest = 'no obstacle'
                self.get_logger().info(
                    f't={now-start:4.1f}s  x={x:5.2f}  max_x={max_x:5.2f}  '
                    f'nearest_obstacle={nearest}'
                )
                last_log = now

        self.get_logger().info(
            f'Test complete. Max X reached: {max_x:.2f} m '
            f'(commanded {TARGET_X:.1f} m). '
            f'{"CP HALTED drone " if max_x < TARGET_X - 1.0 else "Drone reached target — CP not active?"}'
        )

        # Land
        self.get_logger().info('Landing...')
        self.land_client.call_async(CommandTOL.Request())
        while rclpy.ok() and self.pose.pose.position.z > 0.2:
            self._spin(0.1)
        self._arm(False)


def main():
    rclpy.init()
    node = AvoidanceTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
