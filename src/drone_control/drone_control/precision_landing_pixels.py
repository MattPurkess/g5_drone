#!/usr/bin/env python3
"""
Precision Landing Controller.
Subscribes to /apriltag/tag_world_pose (published by the search node)
and /detections (live detections during descent).
Uses a P-controller to align above the tag, then descends in stages.
"""
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import CameraInfo
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from apriltag_msgs.msg import AprilTagDetectionArray
from enum import Enum, auto
from drone_control.scripts.tag_transform_pixels import detection_to_camera_pose, transform_to_world, lateral_error, euclidean_2d


class LandState(Enum):
    APPROACH        = auto()
    DESCEND_COARSE  = auto()
    DESCEND_FINE    = auto()
    FINAL_DESCENT   = auto()
    LANDED          = auto()


class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')

        self.declare_parameter('Kp',              0.5)
        self.declare_parameter('Kp_z',            1.0)
        self.declare_parameter('approach_tol',    0.30)
        self.declare_parameter('fine_tol',        0.15)
        self.declare_parameter('coarse_alt',      3.0)
        self.declare_parameter('fine_alt',        0.8)
        self.declare_parameter('target_tag_id',   0)
        self.declare_parameter('max_vel_xy',      1.5)
        self.declare_parameter('descent_vel',     0.4)
        self.declare_parameter('img_center_tol_px', 20.0)
        self.declare_parameter('Kp_img', 0.003)
        self.declare_parameter('Kp_img_coarse', 0.006)
        self.declare_parameter('detection_stale_sec', 2.0)

        self.Kp         = self.get_parameter('Kp').value
        self.Kp_z       = self.get_parameter('Kp_z').value
        self.app_tol    = self.get_parameter('approach_tol').value
        self.fine_tol   = self.get_parameter('fine_tol').value
        self.coarse_alt = self.get_parameter('coarse_alt').value
        self.fine_alt   = self.get_parameter('fine_alt').value
        self.target_id  = self.get_parameter('target_tag_id').value
        self.max_v_xy   = self.get_parameter('max_vel_xy').value
        self.desc_vel   = self.get_parameter('descent_vel').value
        self.img_center_tol = self.get_parameter('img_center_tol_px').value
        self.Kp_img = self.get_parameter('Kp_img').value
        self.Kp_img_coarse = self.get_parameter('Kp_img_coarse').value
        self.detection_stale_sec = self.get_parameter('detection_stale_sec').value

        self.fsm_state     = LandState.APPROACH
        self.current_state = State()
        self.current_pose  = PoseStamped()
        self.tag_world     = None
        self.tag_last_seen = 0.0
        self.last_u_err    = None
        self.last_v_err    = None
        self.cam_cx        = None
        self.cam_cy        = None
        self.start_time    = time.time()

        self.state_sub    = self.create_subscription(
            State, '/mavros/state', lambda m: setattr(self, 'current_state', m), qos_profile_sensor_data)
        self.pose_sub     = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            lambda m: setattr(self, 'current_pose', m), qos_profile_sensor_data)
        self.tag_init_sub = self.create_subscription(
            PoseStamped, '/apriltag/tag_world_pose',
            lambda m: setattr(self, 'tag_world', m), 1)
        self.det_sub      = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_cb, qos_profile_sensor_data)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/x500/down_camera/camera_info', self.camera_info_cb, qos_profile_sensor_data)

        self.sp_pub  = self.create_publisher(
            PoseStamped,  '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        self.arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def detection_cb(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            det_id = det.id
            if isinstance(det_id, (list, tuple)):
                match = self.target_id in det_id
            else:
                match = int(det_id) == int(self.target_id)

            if match:
                if self.cam_cx is not None and self.cam_cy is not None:
                    self.last_x_err = float(det.centre.x) - self.cam_cx
                    self.last_y_err = float(det.centre.y) - self.cam_cy
                    self.get_logger().debug(
                        f'Detection: centre=({det.centre.x:.1f}, {det.centre.y:.1f}) '
                        f'cam_centre=({self.cam_cx:.1f}, {self.cam_cy:.1f}) '
                        f'err=(x={self.last_x_err:.1f}, y={self.last_y_err:.1f})'
                    )
                self.tag_last_seen = time.time()

    def camera_info_cb(self, msg: CameraInfo):
        # Principal point in camera intrinsics matrix K.
        if len(msg.k) >= 6:
            self.cam_cx = float(msg.k[2])
            self.cam_cy = float(msg.k[5])

    def cmd_vel(self, vx, vy, vz):
        import numpy as np
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.current_pose.header.frame_id or 'odom'
        twist.twist.linear.x = float(np.clip(vx, -self.max_v_xy, self.max_v_xy))
        twist.twist.linear.y = float(np.clip(vy, -self.max_v_xy, self.max_v_xy))
        twist.twist.linear.z = float(np.clip(vz, -0.8, 0.0))
        self.vel_pub.publish(twist)

    def cmd_pos(self, x, y, z):
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = self.current_pose.header.frame_id or 'odom'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self.sp_pub.publish(ps)

    def step_fsm(self):
        alt     = self.current_pose.pose.position.z
        tag_age = time.time() - self.tag_last_seen

        if self.tag_world is None:
            self.get_logger().warn('Waiting for tag pose...')
            return

        ex, ey = lateral_error(self.current_pose, self.tag_world)
        err_2d = euclidean_2d(ex, ey)

        if self.fsm_state == LandState.APPROACH:
            if (self.last_x_err is not None and self.last_y_err is not None and tag_age < 1.0):
                # Tag is fresh: Use visual servoing
                vx = -self.Kp_img * self.last_y_err    # Tag Up (-y) -> Drone moves East (+vx)
                vy = -self.Kp_img * self.last_x_err    # Tag Right (+x) -> Drone moves South (-vy)
                
                # FIX 1: Clamp velocity to limit pitch/roll tilt during approach
                import numpy as np
                approach_max_v = 0.5 # Adjust this (m/s) so the drone doesn't tilt out of FOV
                vx = float(np.clip(vx, -approach_max_v, approach_max_v))
                vy = float(np.clip(vy, -approach_max_v, approach_max_v))

                self.cmd_vel(vx, vy, 0.0)

                centered = (abs(self.last_x_err) < self.img_center_tol and 
                            abs(self.last_y_err) < self.img_center_tol)
            else:
                # Tag lost (likely due to tilt): Use world coordinates to get close
                # Limit velocity here as well to prevent aggressive swinging
                import numpy as np
                approach_max_v = 0.5
                vx = float(np.clip(self.Kp * ex, -approach_max_v, approach_max_v))
                vy = float(np.clip(self.Kp * ey, -approach_max_v, approach_max_v))
                
                self.cmd_vel(vx, vy, 0.0)
                
                # FIX 2: Do NOT allow transition if the tag is not currently seen!
                # Wait until the camera levels out, sees the tag, and visually centers it.
                centered = False

            if centered:
                self.get_logger().info('FSM: APPROACH -> DESCEND_COARSE')
                self.fsm_state = LandState.DESCEND_COARSE

        elif self.fsm_state == LandState.DESCEND_COARSE:
            if (self.last_x_err is not None and self.last_y_err is not None and tag_age < self.detection_stale_sec):
                # APPLY FIXED MAPPING
                vx = -self.Kp_img_coarse * self.last_y_err
                vy = -self.Kp_img_coarse * self.last_x_err
            else:
                vx, vy = self.Kp * ex, self.Kp * ey
                
            vz = -self.Kp_z * self.desc_vel
            self.cmd_vel(vx, vy, vz)

            if alt < self.coarse_alt + 0.5:
                self.get_logger().info('FSM: DESCEND_COARSE -> DESCEND_FINE')
                self.fsm_state = LandState.DESCEND_FINE

        elif self.fsm_state == LandState.DESCEND_FINE:
            if tag_age > 2.0:
                self.cmd_vel(0.0, 0.0, 0.0)
            elif (self.last_x_err is not None and self.last_y_err is not None and tag_age < self.detection_stale_sec):
                # APPLY FIXED MAPPING
                vx = -self.Kp_img * self.last_y_err
                vy = -self.Kp_img * self.last_x_err
                vz = -self.Kp_z * self.desc_vel
                self.cmd_vel(vx, vy, vz)
            else:
                self.cmd_vel(0.0, 0.0, 0.0)

            if alt < self.fine_alt + 0.2:
                self.get_logger().info('FSM: DESCEND_FINE -> FINAL_DESCENT')
                self.fsm_state = LandState.FINAL_DESCENT

        elif self.fsm_state == LandState.FINAL_DESCENT:
            # 1. Send land service call to trigger AUTOLAND mode EXACTLY ONCE.
            if not getattr(self, 'land_cmd_sent', False):
                req = SetMode.Request()
                req.base_mode = 0
                req.custom_mode = 'AUTO.LAND'
                self.set_mode_client.call_async(req)
                self.get_logger().info('AUTO.LAND command sent, waiting for touchdown...')
                self.land_cmd_sent = True  # Prevent spamming the service

            # 2. Transition to LANDED when altitude < 0.15 m.
            if alt < 0.15:
                self.get_logger().info('Touchdown altitude reached.')
                self.fsm_state = LandState.LANDED
            else:
                self.get_logger().debug(f'Descending to touchdown: alt={alt:.2f} m')

        elif self.fsm_state == LandState.LANDED:
            # 1. Disarm the motors
            req = CommandBool.Request()
            req.value = False
            self.arm_client.call_async(req)
            self.get_logger().info('Disarm command sent.')

            # 2. Publish the landing report
            flight_time = time.time() - self.start_time
            
            # Safely calculate final pixel error in case the tag was lost at the very last second
            final_px_err = 0.0
            if self.last_x_err is not None and self.last_y_err is not None:
                final_px_err = euclidean_2d(self.last_x_err, self.last_y_err)

            self.get_logger().info(
                f'LANDED. Final pixel error: {final_px_err:.3f} px  '
                f'Flight time: {flight_time:.1f} s'
            )
            
            # 3. Mark as fully landed by breaking out of FSM loop
            self.fsm_state = None  # Signal to exit main loop

    def run(self):
        rate_hz = 20
        dt      = 1.0 / rate_hz
        self.get_logger().info('Precision landing controller ready.')
        self.get_logger().info('Waiting for tag_world_pose from search node...')
        while self.tag_world is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Tag pose received. Entering landing FSM.')
        self.start_time = time.time()

        while self.fsm_state is not None:
            self.step_fsm()
            rclpy.spin_once(self, timeout_sec=dt)

        self.get_logger().info('Landing complete.')


def main(args=None):
    rclpy.init(args=args)
    PrecisionLandingNode().run()
    rclpy.shutdown()
