#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time, math

class ClosedLoopHoverNode(Node):
    def __init__(self):
        super().__init__('hover_node_velocity')
        self.declare_parameter('hover_altitude', 5.0)
        self.declare_parameter('hover_duration', 30.0)
        self.declare_parameter('hover_x', 0.0)
        self.declare_parameter('hover_y', 0.0)
        
        self.hover_alt = self.get_parameter('hover_altitude').value
        self.hover_dur = self.get_parameter('hover_duration').value
        self.hover_x = self.get_parameter('hover_x').value
        self.hover_y = self.get_parameter('hover_y').value

        # Proportional controller gain
        self.kp = 0.8
        # Maximum velocity limit (m/s) for safety
        self.max_vel = 2.0

        self.current_state = State()
        self.current_pose = PoseStamped()

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, sensor_qos)
        
        # CHANGED: Now publishing velocity commands instead of position setpoints
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def distance_to_target(self, tx, ty, tz):
        dx = self.current_pose.pose.position.x - tx
        dy = self.current_pose.pose.position.y - ty
        dz = self.current_pose.pose.position.z - tz
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def publish_velocity_command(self):
        """Calculates position error and publishes a corrective velocity command."""
        # 1. Calculate error (Target - Current)
        err_x = self.hover_x - self.current_pose.pose.position.x
        err_y = self.hover_y - self.current_pose.pose.position.y
        err_z = self.hover_alt - self.current_pose.pose.position.z

        # 2. Apply Proportional gain (v = Kp * error)
        vx = self.kp * err_x
        vy = self.kp * err_y
        vz = self.kp * err_z

        # 3. Clamp velocities to safety limits
        vx = max(-self.max_vel, min(self.max_vel, vx))
        vy = max(-self.max_vel, min(self.max_vel, vy))
        vz = max(-self.max_vel, min(self.max_vel, vz))

        # 4. Publish Twist message
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        # Keeping yaw rate at 0 for simplicity
        twist.angular.z = 0.0 
        
        self.vel_pub.publish(twist)

    def run(self):
        self.get_logger().info('Waiting for MAVROS2 connection...')
        while not self.current_state.connected:
            rclpy.spin_once(self)
            time.sleep(0.1)
        
        # You must begin publishing setpoints BEFORE requesting the mode switch [cite: 36]
        self.get_logger().info('Pre-streaming velocity setpoints...')
        for _ in range(40):
            self.publish_velocity_command()
            rclpy.spin_once(self)
            time.sleep(0.05)

        # Switch to OFFBOARD
        req_mode = SetMode.Request()
        req_mode.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(req_mode)
        time.sleep(1)

        # Arm the drone
        req_arm = CommandBool.Request()
        req_arm.value = True
        self.arming_client.call_async(req_arm)
        time.sleep(1)

        self.get_logger().info(f'Climbing to hover altitude {self.hover_alt}m using velocity control...')
        while self.distance_to_target(self.hover_x, self.hover_y, self.hover_alt) > 0.3:
            self.publish_velocity_command()
            rclpy.spin_once(self)
            time.sleep(0.05)
        
        self.get_logger().info(f'Hovering for {self.hover_dur} seconds...')
        hover_start = time.time()
        last_print = time.time()
        
        # Closed-loop hold
        while time.time() - hover_start < self.hover_dur:
            self.publish_velocity_command()
            rclpy.spin_once(self)
            if time.time() - last_print > 5.0:
                dist = self.distance_to_target(self.hover_x, self.hover_y, self.hover_alt)
                self.get_logger().info(f'Position error: {dist:.2f} m')
                last_print = time.time()
            time.sleep(0.05)

        self.get_logger().info('Initiating landing...')
        req_land = CommandTOL.Request()
        self.land_client.call_async(req_land)

        while self.current_pose.pose.position.z > 0.15:
            rclpy.spin_once(self)
            time.sleep(0.1)

        req_disarm = CommandBool.Request()
        req_disarm.value = False
        self.arming_client.call_async(req_disarm)
        self.get_logger().info('Mission complete - disarmed.')

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopHoverNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()