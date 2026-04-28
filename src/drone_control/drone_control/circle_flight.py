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


@dataclass
class MissionConfig:
    center_x: float
    center_y: float
    altitude: float
    radius: float
    linear_speed: float = 2.0
    circle_laps: float = 1.0


# Tuning
ACCEPT_RADIUS = 0.1
SETPOINT_HZ = 20
DT = 1.0 / SETPOINT_HZ
LAND_TIMEOUT = 60
FLY_TIMEOUT = 60


def ask_float(prompt, default=None):
    while True:
        text = input(prompt).strip()

        if text == '' and default is not None:
            return float(default)

        try:
            return float(text)
        except ValueError:
            print('Please enter a valid number.')


class WaypointNavNode(Node):

    def __init__(self, config: MissionConfig):
        super().__init__('waypoint_nav_node')

        self.config = config

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
        # Position-driven lookahead: the setpoint always sits a fixed angle ahead
        # of where the drone actually is, so it can never out-run the drone and
        # induce catch-up loops. Total progress is tracked by integrating the
        # signed change in drone-angle (handling the ±π wrap), so completion is
        # tied to physical motion rather than wall-clock iterations.
        cfg = self.config
        final = 2.0 * math.pi * cfg.circle_laps

        # ~0.5 s of motion ahead, clamped to a sensible range.
        lookahead_rad = max(0.05, min(0.5, (cfg.linear_speed / cfg.radius) * 0.5))

        ideal_duration = cfg.radius * final / cfg.linear_speed
        timeout = max(60.0, ideal_duration * 3.0)

        self.get_logger().info(
            f'Circling: r={cfg.radius:.1f} m, v={cfg.linear_speed:.1f} m/s, '
            f'laps={cfg.circle_laps:.1f}, '
            f'lookahead={math.degrees(lookahead_rad):.0f} deg'
        )

        last_theta = 0.0
        progress = 0.0
        first_iter = True
        start = time.time()
        last_log = start

        while rclpy.ok():
            dx = self.current_pose.pose.position.x - cfg.center_x
            dy = self.current_pose.pose.position.y - cfg.center_y
            drone_theta = math.atan2(dy, dx)

            if first_iter:
                last_theta = drone_theta
                first_iter = False

            delta = drone_theta - last_theta
            if delta > math.pi:
                delta -= 2.0 * math.pi
            elif delta < -math.pi:
                delta += 2.0 * math.pi

            progress += delta
            last_theta = drone_theta

            if progress >= final:
                self.get_logger().info('Circle complete.')
                break

            if time.time() - start > timeout:
                self.get_logger().warn(
                    f'Circle timeout at {math.degrees(progress):.0f} / '
                    f'{math.degrees(final):.0f} deg'
                )
                break

            target_theta = drone_theta + lookahead_rad
            x = cfg.center_x + cfg.radius * math.cos(target_theta)
            y = cfg.center_y + cfg.radius * math.sin(target_theta)
            z = cfg.altitude
            yaw = target_theta + math.pi  # face centre

            self._publish(x, y, z, yaw)
            self._spin()

            now = time.time()
            if now - last_log >= 5.0:
                pct = (progress / final) * 100.0
                self.get_logger().info(
                    f'Circle: {pct:.0f}% complete '
                    f'({math.degrees(progress):.0f} / {math.degrees(final):.0f} deg)'
                )
                last_log = now

    def run(self):
        cfg = self.config

        while not self.mavros_state.connected:
            self._spin(0.1)

        while not self.have_pose:
            self._spin(0.1)

        # Pre-stream at the current x/y at altitude so OFFBOARD-on-arm climbs
        # vertically rather than darting diagonally toward (cx, cy).
        climb_wp = Waypoint(
            float(self.current_pose.pose.position.x),
            float(self.current_pose.pose.position.y),
            float(cfg.altitude),
            name='Climb'
        )

        t0 = time.time()
        while time.time() - t0 < 2:
            self._publish(climb_wp.x, climb_wp.y, climb_wp.z)
            self._spin()

        self._set_mode('OFFBOARD')
        self._arm(True)

        self._fly_to(climb_wp)

        # centre
        centre_wp = Waypoint(
            cfg.center_x,
            cfg.center_y,
            cfg.altitude,
            name='Centre'
        )

        self._fly_to(centre_wp)
        self._dwell(centre_wp)

        # move to circle start
        start_wp = Waypoint(
            cfg.center_x + cfg.radius,
            cfg.center_y,
            cfg.altitude,
            name='Circle start'
        )

        self._fly_to(start_wp)

        # circle
        self._fly_circle()

        # land
        self._set_mode('AUTO.LAND')


def main():
    print('\nCircular Flight Setup')
    print('Press Enter to use the default value shown in brackets.\n')

    center_x = ask_float('Centre X coordinate [0.0]: ', 0.0)
    center_y = ask_float('Centre Y coordinate [0.0]: ', 0.0)
    altitude = ask_float('Altitude in metres [8.0]: ', 8.0)
    radius = ask_float('Circle radius in metres [10.0]: ', 10.0)
    linear_speed = ask_float('Linear speed in m/s [2.0]: ', 2.0)
    circle_laps = ask_float('Number of laps [1.0]: ', 1.0)

    if radius <= 0.0:
        print('Radius must be greater than 0. Exiting.')
        return

    if altitude <= 0.0:
        print('Altitude must be greater than 0. Exiting.')
        return

    if linear_speed <= 0.0:
        print('Linear speed must be greater than 0. Exiting.')
        return

    if circle_laps <= 0.0:
        print('Number of laps must be greater than 0. Exiting.')
        return

    config = MissionConfig(
        center_x=center_x,
        center_y=center_y,
        altitude=altitude,
        radius=radius,
        linear_speed=linear_speed,
        circle_laps=circle_laps
    )

    print(
        f'\nMission configured:\n'
        f'  Centre:   ({config.center_x:.2f}, {config.center_y:.2f})\n'
        f'  Altitude: {config.altitude:.2f} m\n'
        f'  Radius:   {config.radius:.2f} m\n'
        f'  Speed:    {config.linear_speed:.2f} m/s\n'
        f'  Laps:     {config.circle_laps:.1f}\n'
    )

    rclpy.init()

    node = WaypointNavNode(config)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
