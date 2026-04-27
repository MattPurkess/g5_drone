#!/usr/bin/env python3
"""
AprilTag Search Node.
Flies an expanding square spiral at search altitude, subscribing to
/detections. Transitions to landing_controller topic once tag found.
"""
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
from drone_control.scripts.tag_transform import (
    detection_to_camera_pose, transform_to_world)


class AprilTagSearchNode(Node):
    def __init__(self):
        super().__init__('apriltag_search_node')

        self.declare_parameter('search_altitude',   12.0)
        self.declare_parameter('step_size',          8.0)
        self.declare_parameter('max_radius',        60.0)
        self.declare_parameter('target_tag_id',        0)
        self.declare_parameter('waypoint_dwell',     2.0)
        self.declare_parameter('accept_radius',      1.0)

        self.search_alt  = self.get_parameter('search_altitude').value
        self.step        = self.get_parameter('step_size').value
        self.max_r       = self.get_parameter('max_radius').value
        self.target_id   = self.get_parameter('target_tag_id').value
        self.dwell       = self.get_parameter('waypoint_dwell').value
        self.accept_r    = self.get_parameter('accept_radius').value

        self.current_state  = State()
        self.current_pose   = PoseStamped()
        self.tag_world_pose = None
        self.tag_found      = False
        self.world_frame    = 'odom'

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.det_sub   = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_cb, 10)

        self.setpoint_pub  = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.tag_found_pub = self.create_publisher(
            PoseStamped, '/apriltag/tag_world_pose', 10)

        self.arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def state_cb(self, msg):  self.current_state = msg
    def pose_cb(self,  msg):
        self.current_pose = msg

        # Bridge MAVROS local pose into TF so map -> base_link -> camera_down_link
        # is a connected tree for AprilTag transforms.
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        self.world_frame = tf_msg.header.frame_id
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = float(msg.pose.position.x)
        tf_msg.transform.translation.y = float(msg.pose.position.y)
        tf_msg.transform.translation.z = float(msg.pose.position.z)
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        camera_tf = TransformStamped()
        camera_tf.header.stamp = msg.header.stamp
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_down_link'
        camera_tf.transform.translation.x = 0.0
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = -0.05
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.7071
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 0.7071
        self.tf_broadcaster.sendTransform(camera_tf)

        camera_alias = TransformStamped()
        camera_alias.header.stamp = msg.header.stamp
        camera_alias.header.frame_id = 'camera_down_link'
        camera_alias.child_frame_id = 'camera_link'
        camera_alias.transform.translation.x = 0.0
        camera_alias.transform.translation.y = 0.0
        camera_alias.transform.translation.z = 0.0
        camera_alias.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(camera_alias)

    def detection_cb(self, msg: AprilTagDetectionArray):
        if self.tag_found:
            return
        for det in msg.detections:
            det_id = det.id
            if isinstance(det_id, (list, tuple)):
                match = self.target_id in det_id
            else:
                match = int(det_id) == int(self.target_id)

            if match:
                cam_pose   = detection_to_camera_pose(det)
                world_pose = transform_to_world(
                    self,
                    self.tf_buffer,
                    cam_pose,
                    world_frame=self.world_frame,
                )
                if world_pose is not None:
                    self.tag_world_pose = world_pose
                    self.tag_found      = True
                    self.get_logger().info(
                        f'TAG FOUND'
                    )
                    self.tag_found_pub.publish(world_pose)

    def pub_sp(self, x, y, z):
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = self.world_frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self.setpoint_pub.publish(ps)

    def dist_xy(self, tx, ty) -> float:
        dx = self.current_pose.pose.position.x - tx
        dy = self.current_pose.pose.position.y - ty
        return math.sqrt(dx*dx + dy*dy)

    def set_offboard_mode(self):
        """Request OFFBOARD mode from autopilot."""
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            return future.result().mode_sent
        return False

    def arm_drone(self):
        """Request arm command from autopilot."""
        from mavros_msgs.srv import CommandBool
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            return future.result().success
        return False

    def command_rtl(self):
        """Request Return-to-Launch mode."""
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = "AUTO.RTL"
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            return future.result().mode_sent
        return False

    def fly_to(self, x, y, z, timeout=45.0):
        start = time.time()
        while self.dist_xy(x, y) > self.accept_r or \
              abs(self.current_pose.pose.position.z - z) > 0.5:
            if self.tag_found:
                return
            if time.time() - start > timeout:
                self.get_logger().warn('fly_to timeout')
                return
            self.pub_sp(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.05)

    def generate_spiral(self):
        """Yield (x, y) waypoints for an expanding square spiral.

        Algorithm:
        Start at (0, 0). Move East by d, North by d, West by 2d, South by 2d,
        East by 3d, etc. Each direction side length increases by d every
        two turns. Stop when radius exceeds self.max_r.
        """
        x, y = 0, 0
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # E, N, W, S
        
        step_index = 0
        while True:
            # Side length: 1, 1, 2, 2, 3, 3, 4, 4, ...
            side_length = (step_index // 2) + 1
            
            # Current direction (cycles through E, N, W, S)
            dir_idx = step_index % 4
            dx, dy = directions[dir_idx]
            
            # Generate waypoints along this side
            for _ in range(side_length):
                x += dx * self.step
                y += dy * self.step
                
                # Stop if we exceed maximum search radius
                dist = math.sqrt(x*x + y*y)
                if dist > self.max_r:
                    return
                
                yield (x, y)
            
            step_index += 1

    def run(self):
        self.get_logger().info('Waiting for connection...')
        while not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)

        for _ in range(40):
            self.pub_sp(0.0, 0.0, self.search_alt)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Set OFFBOARD mode and arm
        self.get_logger().info("Setting OFFBOARD mode...")
        while not self.set_offboard_mode():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Arming...")
        while not self.arm_drone():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Climbing to search altitude {self.search_alt} m...')
        while abs(self.current_pose.pose.position.z - self.search_alt) > 0.5:
            self.pub_sp(0.0, 0.0, self.search_alt)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info('Beginning spiral search...')
        for (wx, wy) in self.generate_spiral():
            if self.tag_found:
                break
            self.get_logger().info(f'  Search WP -> ({wx:.1f}, {wy:.1f})')
            self.fly_to(wx, wy, self.search_alt)
            dwell_end = time.time() + self.dwell
            while time.time() < dwell_end and not self.tag_found:
                self.pub_sp(wx, wy, self.search_alt)
                rclpy.spin_once(self, timeout_sec=0.05)

        if not self.tag_found:
            self.get_logger().error('Search exhausted — tag not found! RTL.')
            self.get_logger().info('Commanding Return-to-Launch...')
            self.command_rtl()
        else:
            self.get_logger().info('Handover to landing controller.')


def main(args=None):
    rclpy.init(args=args)
    AprilTagSearchNode().run()
    rclpy.shutdown()