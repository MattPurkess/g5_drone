#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger
from std_msgs.msg import Float32


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

        # Add up/down subscribers and service
        self.move_up_sub = self.create_subscription(Float32, 'move_up', self._move_up_cb, 10)
        self.move_down_sub = self.create_subscription(Float32, 'move_down', self._move_down_cb, 10)
        self.set_vertical_velocity_sub = self.create_subscription(Float32, 'set_vertical_velocity', self._set_vertical_velocity_cb, 10)
        self.stop_vertical_srv = self.create_service(Trigger, 'stop_vertical', self._stop_vertical_cb)

        self._stop = threading.Event()
        self._pub_thread = threading.Thread(target=self._publisher_loop, daemon=True)
        self._pub_thread.start()

    def destroy(self):
        self._stop.set()
        self._pub_thread.join(timeout=1.0)
        self.destroy_node()

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self.have_pose = True

    def _teleop_cb(self, msg):
        self.user_cmd = msg
        self.user_cmd_time = time.time()

    def _move_up_cb(self, msg):
        """Move up by the specified distance (meters)"""
        distance = msg.data
        if distance <= 0:
            self.get_logger().warn("Distance must be positive")
            return
        
        target_alt = self.pose.pose.position.z + distance
        self.get_logger().info(f"Moving up to {target_alt:.2f} m")
        
        # Temporarily switch to position control for precise movement
        original_mode = self.mode
        self.mode = 'HOVER'
        
        # Update hover altitude
        self.hover_alt = target_alt
        
        # Wait for movement to complete
        timeout = 10.0  # seconds
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.pose.pose.position.z - target_alt) < 0.3:
                break
        
        self.mode = original_mode
        self.get_logger().info(f"At altitude: {self.pose.pose.position.z:.2f} m")

    def _move_down_cb(self, msg):
        """Move down by the specified distance (meters)"""
        distance = msg.data
        if distance <= 0:
            self.get_logger().warn("Distance must be positive")
            return
        
        target_alt = max(0.5, self.pose.pose.position.z - distance)  # Don't go below 0.5m
        self.get_logger().info(f"Moving down to {target_alt:.2f} m")
        
        # Temporarily switch to position control for precise movement
        original_mode = self.mode
        self.mode = 'HOVER'
        
        # Update hover altitude
        self.hover_alt = target_alt
        
        # Wait for movement to complete
        timeout = 10.0  # seconds
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.pose.pose.position.z - target_alt) < 0.3:
                break
        
        self.mode = original_mode
        self.get_logger().info(f"At altitude: {self.pose.pose.position.z:.2f} m")

    def _set_vertical_velocity_cb(self, msg):
        """Set vertical velocity (positive = up, negative = down)"""
        velocity = msg.data
        self.get_logger().info(f"Setting vertical velocity to {velocity:.2f} m/s")
        
        # Create a twist message with only vertical velocity
        vel_cmd = Twist()
        vel_cmd.linear.z = velocity
        
        # Publish immediately and set as user command
        self.user_cmd = vel_cmd
        self.user_cmd_time = time.time()
        self.sp_vel_pub.publish(vel_cmd)

    def _stop_vertical_cb(self, request, response):
        """Stop vertical movement"""
        self.get_logger().info("Stopping vertical movement")
        
        # Set vertical velocity to 0
        vel_cmd = Twist()
        vel_cmd.linear.x = self.user_cmd.linear.x
        vel_cmd.linear.y = self.user_cmd.linear.y
        vel_cmd.linear.z = 0.0  # Stop vertical movement
        vel_cmd.angular.z = self.user_cmd.angular.z
        
        self.user_cmd = vel_cmd
        self.user_cmd_time = time.time()
        self.sp_vel_pub.publish(vel_cmd)
        
        response.success = True
        response.message = "Vertical movement stopped"
        return response

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
        sp.pose.position.x = 0.0
        sp.pose.position.y = 0.0
        sp.pose.position.z = float(self.hover_alt)
        sp.pose.orientation.w = 1.0
        self.sp_pos_pub.publish(sp)

    def _publish_vel_setpoint(self):
        out = Twist()
        if (time.time() - self.user_cmd_time) <= self.cmd_timeout:
            out.linear.x = float(self.user_cmd.linear.x)
            out.linear.y = float(self.user_cmd.linear.y)
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

        self.get_logger().info("Setting OFFBOARD...")
        if not self.set_mode('OFFBOARD'):
            self.get_logger().error("OFFBOARD request failed.")
            return

        self.get_logger().info("Arming...")
        if not self.arm(True):
            self.get_logger().error("Arming failed.")
            return

        self.get_logger().info(f"Climbing to {self.hover_alt} m...")
        while rclpy.ok() and abs(self.pose.pose.position.z - self.hover_alt) > 0.3:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("At altitude. Teleop enabled")
        self.mode = 'TELEOP'

        while rclpy.ok():
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
