#!/usr/bin/env python3
import sys
import tty
import termios
import select
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class OffboardTakeoffTeleop(Node):
    def __init__(self):
        super().__init__('offboard_takeoff_teleop')

        self.hover_alt = 5.0
        self.rate_hz = 20.0
        self.dt = 1.0 / self.rate_hz

        self.state = State()
        self.pose = PoseStamped()
        self.have_pose = False

        self.user_cmd = Twist()
        self.user_cmd_time = 0.0
        self.cmd_timeout = 9999.0

        self.mode = 'HOVER'  # HOVER or TELEOP
        self.GROUND_ALT_THRESHOLD = 0.5  # meters
        self._pending_action = None  # 'arm' or 'disarm', processed by main loop

        self.hover_x = 0.0
        self.hover_y = 0.0

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, qos_profile_sensor_data
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_profile_sensor_data
        )

        self.teleop_sub = self.create_subscription(
            Twist, '/cmd_vel', self._teleop_cb, 10
        )

        self.sp_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.sp_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self._stop = threading.Event()
        self._pub_thread = threading.Thread(target=self._publisher_loop, daemon=True)
        self._pub_thread.start()
        self._key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._key_thread.start()

    def destroy(self):
        self._stop.set()
        self._pub_thread.join(timeout=1.0)
        self._key_thread.join(timeout=1.0)
        self.destroy_node()

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self.have_pose = True

    def _teleop_cb(self, msg):
        self.user_cmd = msg
        self.user_cmd_time = time.time()

    def _keyboard_loop(self):
        """Read single keypresses from the terminal to listen for arm and disarm commands"""
        try:
            fd = sys.stdin.fileno()
        except Exception:
            self.get_logger().error("No terminal available for keyboard input")
            return
        old_settings = termios.tcgetattr(fd)
        try:
            # keep single-key reads without breaking terminal output formatting
            tty.setcbreak(fd)
            while not self._stop.is_set() and rclpy.ok():
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key == 'a':
                            self._pending_action = 'arm'
                        elif key == 'd':
                            self._pending_action = 'disarm'
                        elif key == '\x03':  # Ctrl-C
                            self._stop.set()
                            break
                except Exception:
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _arm_drone(self):
        """Arm the drone and take off to specified hover altitude"""
        if self.state.armed:
            self.get_logger().error("Arming Error: drone is already armed")
            return

        if self.have_pose and self.pose.pose.position.z > self.GROUND_ALT_THRESHOLD:
            self.get_logger().error("Arming Error: drone is not on the ground")
            return

        self.get_logger().info("Setting OFFBOARD mode...")
        if not self.set_mode('OFFBOARD'):
            self.get_logger().error("Arming Error: could not set OFFBOARD mode")
            return

        self.get_logger().info("Arming...")
        if not self.arm(True):
            self.get_logger().error("Arming Error: could not arm drone")
            return

        # Take off from current position — only change Z value
        self.hover_x = self.pose.pose.position.x
        self.hover_y = self.pose.pose.position.y
        self.get_logger().info(f"Arming Successful: Climbing to {self.hover_alt}m from ({self.hover_x:.2f}, {self.hover_y:.2f})...")
        timeout = 30.0
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.pose.pose.position.z - self.hover_alt) < 0.3:
                break

        self.get_logger().info("At altitude. Teleop enabled")
        self.mode = 'TELEOP'

    def _disarm_drone(self):
        """Land the drone at its current position using AUTO.LAND and then disarm"""
        if not self.state.armed:
            self.get_logger().error("Disarming Error: drone is already disarmed")
            return

        self.get_logger().info(f"Drone is at {self.pose.pose.position.z:.2f} m — switching to AUTO.LAND...")

        if not self.set_mode('AUTO.LAND'):
            self.get_logger().error("Landing Error: failed to set AUTO.LAND mode")
            return

        # Wait for the flight controller to detect landing
        timeout = 60.0
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.state.armed:
                # PX4 auto-disarmed after landing
                self.mode = 'HOVER'
                self.get_logger().info("Landing Successful: Drone landed and disarmed successfully")
                return

        self.get_logger().error("Landing Error: Landing timed out")

    def _call_service(self, client, req, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        return fut.result()

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        res = self._call_service(self.set_mode_client, req)
        return res is not None and res.mode_sent

    def arm(self, value: bool) -> bool:
        req = CommandBool.Request()
        req.value = bool(value)
        res = self._call_service(self.arming_client, req)
        return res is not None and res.success

    def _publish_hover_setpoint(self):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = float(self.hover_x)
        sp.pose.position.y = float(self.hover_y)
        sp.pose.position.z = float(self.hover_alt)
        sp.pose.orientation.w = 1.0
        self.sp_pos_pub.publish(sp)

    def _publish_vel_setpoint(self):
        out = Twist()
        if (time.time() - self.user_cmd_time) <= self.cmd_timeout:
            out.linear.x = float(self.user_cmd.linear.y) * -1.0
            out.linear.y = float(self.user_cmd.linear.x)
            out.linear.z = float(self.user_cmd.linear.z)
            out.angular.z = float(self.user_cmd.angular.z)
        self.sp_vel_pub.publish(out)

    def _publisher_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            if self.mode == 'TELEOP':
                self._publish_vel_setpoint()
            else:
                self._publish_hover_setpoint()
            time.sleep(self.dt)

    def run(self):
        self.get_logger().info("Waiting for MAVROS connection...")
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Connected.")

        self.get_logger().info("Waiting for pose...")
        while rclpy.ok() and not self.have_pose:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Prestreaming hover setpoints...")
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Ready. Press 'a' to arm and takeoff, 'd' to land and disarm.")

        while rclpy.ok() and not self._stop.is_set():
            if self._pending_action == 'arm':
                self._pending_action = None
                self._arm_drone()
            elif self._pending_action == 'disarm':
                self._pending_action = None
                self._disarm_drone()
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()