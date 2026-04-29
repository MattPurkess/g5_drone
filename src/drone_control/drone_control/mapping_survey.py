#!/usr/bin/env python3
import rclpy
import math
import time
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType
from tf_transformations import quaternion_from_euler


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0
    dwell: float = 0.5
    name: str = ''


def generate_lawnmower(x_min, x_max, y_min, y_max, strip_spacing, altitude):
    waypoints = []
    y = y_min
    direction = 1
    strip_idx = 0

    while y <= y_max:
        x_start = x_min if direction == 1 else x_max
        x_end = x_max if direction == 1 else x_min
        yaw = 0.0 if direction == 1 else math.pi

        waypoints.append(Waypoint(
            x=x_start, y=y, z=altitude, yaw=yaw,
            dwell=0.5, name=f'Strip{strip_idx}_start'
        ))
        waypoints.append(Waypoint(
            x=x_end, y=y, z=altitude, yaw=yaw,
            dwell=1.0, name=f'Strip{strip_idx}_end'
        ))

        y += strip_spacing
        direction *= -1
        strip_idx += 1

    return waypoints


class MappingSurveyNode(Node):
    def __init__(self):
        super().__init__('mapping_survey_node')

        self.declare_parameter('x_min', -30.0)
        self.declare_parameter('x_max', 30.0)
        self.declare_parameter('y_min', -30.0)
        self.declare_parameter('y_max', 30.0)
        self.declare_parameter('strip_spacing', 8.0)
        self.declare_parameter('altitude', 17.0)
        self.declare_parameter('acceptance_radius', 1.5)

        self.declare_parameter('cruise_speed', 2.0)
        self.declare_parameter('max_horiz_speed', 2.0)
        self.declare_parameter('max_climb_speed', 2.0)
        self.declare_parameter('max_descent_speed', 2.0)

        # Yaw control
        self.declare_parameter('yaw_rate_auto', 20.0)   # deg/s, auto-mode cap
        self.declare_parameter('yaw_rate_max', 50.0)    # deg/s, rate-controller hard ceiling
        self.declare_parameter('turn_duration', 4.0)    # seconds for a 180° turn
        self.declare_parameter('yaw_threshold', 0.1)    # rad; trigger ramp if delta exceeds this

        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.strip_spacing = self.get_parameter('strip_spacing').value
        self.altitude = self.get_parameter('altitude').value
        self.accept_r = self.get_parameter('acceptance_radius').value
        self.cruise = self.get_parameter('cruise_speed').value
        self.v_max = self.get_parameter('max_horiz_speed').value
        self.v_up = self.get_parameter('max_climb_speed').value
        self.v_down = self.get_parameter('max_descent_speed').value
        self.yaw_rate_auto = self.get_parameter('yaw_rate_auto').value
        self.yaw_rate_max = self.get_parameter('yaw_rate_max').value
        self.turn_duration = self.get_parameter('turn_duration').value
        self.yaw_threshold = self.get_parameter('yaw_threshold').value

        self.mission = generate_lawnmower(
            self.x_min, self.x_max, self.y_min, self.y_max,
            self.strip_spacing, self.altitude
        )
        self.get_logger().info(
            f'Generated survey: {len(self.mission)} waypoints, '
            f'{(len(self.mission)//2)} strips, altitude {self.altitude} m'
        )

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_received = False

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_cb, sensor_qos)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.param_client = self.create_client(ParamSetV2, '/mavros/param/set')

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg
        self.pose_received = True

    def dist_to(self, wp):
        dx = self.current_pose.pose.position.x - wp.x
        dy = self.current_pose.pose.position.y - wp.y
        dz = self.current_pose.pose.position.z - wp.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def publish_setpoint(self, wp):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        pose.pose.position.z = wp.z
        q = quaternion_from_euler(0.0, 0.0, wp.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.setpoint_pub.publish(pose)

    def make_vertical_takeoff_wp(self):
        return Waypoint(
            x=self.current_pose.pose.position.x,
            y=self.current_pose.pose.position.y,
            z=self.altitude,
            yaw=0.0,
            dwell=2.0,
            name='Takeoff_Up'
        )

    def set_px4_param(self, name, value, retries=10):
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Param service unavailable, skipping {name}')
            return False

        for _ in range(retries):
            req = ParamSetV2.Request()
            req.force_set = True
            req.param_id = name
            req.value = ParameterValue()
            req.value.type = ParameterType.PARAMETER_DOUBLE
            req.value.double_value = float(value)

            future = self.param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            res = future.result()
            if res is not None and res.success:
                self.get_logger().info(f'Set {name} = {value}')
                return True
            time.sleep(0.5)

        self.get_logger().warn(f'Failed to set {name} after {retries} retries')
        return False

    def apply_speed_limits(self):
        self.get_logger().info('Applying speed caps...')
        self.set_px4_param('MPC_XY_CRUISE', self.cruise)
        self.set_px4_param('MPC_XY_VEL_MAX', self.v_max)
        self.set_px4_param('MPC_Z_VEL_MAX_UP', self.v_up)
        self.set_px4_param('MPC_Z_VEL_MAX_DN', self.v_down)
        # Auto-mode yaw cap (covers AUTO.* modes)
        self.set_px4_param('MPC_YAWRAUTO_MAX', self.yaw_rate_auto)
        # Rate-controller cap — actually bites in OFFBOARD position mode
        self.set_px4_param('MC_YAWRATE_MAX', self.yaw_rate_max)

    def set_offboard(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() and future.result().mode_sent:
            self.get_logger().info('Offboard mode set')
            return True
        self.get_logger().warn('Failed to set Offboard mode')
        return False

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() and future.result().success:
            self.get_logger().info('Armed')
            return True
        self.get_logger().warn('Failed to arm')
        return False

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() and future.result().success:
            self.get_logger().info('Disarmed')
        else:
            self.get_logger().warn('Disarm failed (may be auto-disarmed by PX4)')

    def turn_in_place(self, from_yaw, to_yaw, x, y, z, duration=None):
        """Smoothly rotate from from_yaw to to_yaw while holding position."""
        if duration is None:
            duration = self.turn_duration

        # shortest-path angular distance
        delta = math.atan2(math.sin(to_yaw - from_yaw),
                           math.cos(to_yaw - from_yaw))

        # scale duration by how far we actually need to turn (full duration = 180°)
        scaled = duration * (abs(delta) / math.pi)
        scaled = max(scaled, 0.5)  # floor for tiny turns

        steps = max(int(scaled / 0.1), 1)  # 10 Hz update
        self.get_logger().info(
            f'  Turning {math.degrees(delta):+.0f}° over {scaled:.1f}s '
            f'({math.degrees(abs(delta)) / scaled:.0f}°/s)'
        )

        for i in range(steps + 1):
            if not rclpy.ok():
                break
            t = i / steps
            yaw = from_yaw + delta * t
            self.publish_setpoint(Waypoint(
                x=x, y=y, z=z, yaw=yaw, name='Turning'
            ))
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def go_to_waypoint(self, wp):
        self.get_logger().info(
            f'→ {wp.name}  ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})'
        )

        while rclpy.ok() and self.dist_to(wp) > self.accept_r:
            self.publish_setpoint(wp)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        dwell_start = time.time()
        while rclpy.ok() and (time.time() - dwell_start) < wp.dwell:
            self.publish_setpoint(wp)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def run(self):
        self.get_logger().info('Waiting for MAVROS connection...')
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        self.get_logger().info('MAVROS connected')

        self.get_logger().info('Waiting for local position...')
        while rclpy.ok() and not self.pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.apply_speed_limits()

        takeoff_wp = self.make_vertical_takeoff_wp()

        self.get_logger().info('Pre-streaming vertical takeoff setpoints...')
        for _ in range(40):
            self.publish_setpoint(takeoff_wp)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        if not self.set_offboard():
            return
        if not self.arm():
            return

        self.get_logger().info('Climbing vertically to survey altitude...')
        self.go_to_waypoint(takeoff_wp)

        start = time.time()
        self.get_logger().info(f'Beginning mapping survey — {len(self.mission)} waypoints')

        prev_yaw = takeoff_wp.yaw
        for wp in self.mission:
            if not rclpy.ok():
                break

            # If yaw needs to change significantly, ramp it before flying to the new waypoint
            if abs(math.atan2(math.sin(wp.yaw - prev_yaw),
                              math.cos(wp.yaw - prev_yaw))) > self.yaw_threshold:
                self.turn_in_place(
                    from_yaw=prev_yaw,
                    to_yaw=wp.yaw,
                    x=self.current_pose.pose.position.x,
                    y=self.current_pose.pose.position.y,
                    z=self.altitude,
                )

            self.go_to_waypoint(wp)
            prev_yaw = wp.yaw

        elapsed = time.time() - start
        self.get_logger().info(f'Survey complete in {elapsed:.1f} s')

        # Return home, ramping yaw back to 0 if needed
        if abs(math.atan2(math.sin(0.0 - prev_yaw),
                          math.cos(0.0 - prev_yaw))) > self.yaw_threshold:
            self.turn_in_place(
                from_yaw=prev_yaw,
                to_yaw=0.0,
                x=self.current_pose.pose.position.x,
                y=self.current_pose.pose.position.y,
                z=self.altitude,
            )

        home = Waypoint(0.0, 0.0, self.altitude, 0.0, 2.0, 'Home')
        self.go_to_waypoint(home)

        self.get_logger().info('Landing...')
        self.land_client.call_async(CommandTOL.Request())
        while rclpy.ok() and self.current_pose.pose.position.z > 0.15:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.disarm()
        self.get_logger().info('Mission complete')


def main(args=None):
    rclpy.init(args=args)
    node = MappingSurveyNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
