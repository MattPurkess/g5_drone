#!/usr/bin/env python3
"""
lidar_to_obstacle.py — Convert the drone's 3D lidar pointcloud into a flat
sensor_msgs/LaserScan and publish to /mavros/obstacle/send.

On startup, also enables PX4 collision prevention via /mavros/param/set:
    CP_DIST  = 3.0   safety distance from obstacles (m); positive enables CP
    CP_DELAY = 0.4   sensor + autopilot response delay (s)

MAVROS forwards the LaserScan as a MAVLink OBSTACLE_DISTANCE message; PX4 then
slows or stops when any active flight node commands setpoints into obstacles.

Run alongside the regular sim:
    ros2 run drone_control lidar_to_obstacle
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from sensor_msgs.msg import PointCloud2, LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSetV2


# 72 sectors of 5 deg each — the format PX4 expects.
NUM_SECTORS = 72
SECTOR_WIDTH = 2.0 * math.pi / NUM_SECTORS

# Sensor range
MIN_RANGE = 0.5
MAX_RANGE = 50.0

# Only consider lidar points within this vertical band of the lidar's own
# origin — i.e. only horizontal-ish obstacles around the drone, not floor/ceiling.
ALTITUDE_BAND_M = 0.3

CP_PARAMS = [
    ('CP_DIST', 3.0, ParameterType.PARAMETER_DOUBLE),
    ('CP_DELAY', 0.2, ParameterType.PARAMETER_DOUBLE),
]


def _read_xyz(msg: PointCloud2):
    """
    Minimal PointCloud2 iterator that pulls x/y/z out of a packed buffer.
    Avoids the sensor_msgs_py dependency.
    """
    fmt_lookup = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = (f.offset, fmt_lookup[f.datatype])

    if not all(k in offsets for k in ('x', 'y', 'z')):
        return

    data = msg.data
    point_step = msg.point_step
    n_points = msg.width * msg.height

    for i in range(n_points):
        base = i * point_step
        x = struct.unpack_from('<' + offsets['x'][1], data, base + offsets['x'][0])[0]
        y = struct.unpack_from('<' + offsets['y'][1], data, base + offsets['y'][0])[0]
        z = struct.unpack_from('<' + offsets['z'][1], data, base + offsets['z'][0])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield x, y, z


class LidarToObstacle(Node):

    def __init__(self):
        super().__init__('lidar_to_obstacle')

        self.state = State()

        self.create_subscription(
            PointCloud2, '/x500/scan/points',
            self._cb, qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state',
            lambda m: setattr(self, 'state', m), 10)

        self.pub = self.create_publisher(
            LaserScan, '/mavros/obstacle/send', 10)

        self.get_logger().info(
            f'Lidar -> obstacle distance bridge: {NUM_SECTORS} sectors, '
            f'range {MIN_RANGE:.1f}-{MAX_RANGE:.1f} m, '
            f'altitude band ±{ALTITUDE_BAND_M:.1f} m'
        )

        self.get_logger().info('Waiting for MAVROS connection to PX4...')
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec=0.2)

        self._set_obstacle_frame_body_frd()
        self._enable_collision_prevention()

    def _set_obstacle_frame_body_frd(self):
        client = self.create_client(SetParameters, '/mavros/obstacle/set_parameters')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                '/mavros/obstacle/set_parameters not available — '
                'mav_frame will remain at default (GLOBAL).'
            )
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'mav_frame'
        p.value.type = ParameterType.PARAMETER_STRING
        p.value.string_value = 'BODY_FRD'
        req.parameters = [p]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()
        ok = res is not None and all(r.successful for r in res.results)
        if ok:
            self.get_logger().info('Set /mavros/obstacle mav_frame = BODY_FRD')
        else:
            self.get_logger().error('Failed to set mav_frame to BODY_FRD')

    def _enable_collision_prevention(self):
        client = self.create_client(ParamSetV2, '/mavros/param/set')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                '/mavros/param/set not available — skipping CP param setup.'
            )
            return

        self.get_logger().info('Enabling PX4 collision prevention:')
        for name, value, type_ in CP_PARAMS:
            for _ in range(15):
                req = ParamSetV2.Request()
                req.param_id = name
                req.value = ParameterValue()
                req.value.type = type_
                if type_ == ParameterType.PARAMETER_INTEGER:
                    req.value.integer_value = int(value)
                else:
                    req.value.double_value = float(value)

                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                res = future.result()
                if res is not None and res.success:
                    self.get_logger().info(f'  {name} = {value}')
                    break
                time.sleep(1.0)
            else:
                self.get_logger().error(f'  failed to set {name} after retries')

    def _cb(self, msg: PointCloud2):
        # Initialize all sectors to "no obstacle" (= max range).
        ranges = [float('inf')] * NUM_SECTORS

        # mavros plugin sends OBSTACLE_DISTANCE with frame=BODY_FRD; PX4 expects
        # angles CW from forward (FRD). Lidar gives CCW from forward (ENU body),
        # so negate atan2 to flip CCW → CW.
        for x, y, z in _read_xyz(msg):
            # In lidar_link frame: x forward, y left, z up. Filter by vertical band.
            if abs(z) > ALTITUDE_BAND_M:
                continue
            d = math.hypot(x, y)
            if d < MIN_RANGE or d > MAX_RANGE:
                continue
            angle = -math.atan2(y, x)              # CW from forward (FRD)
            sector = int((angle + math.pi) / SECTOR_WIDTH) % NUM_SECTORS
            if d < ranges[sector]:
                ranges[sector] = d

        # NaN means "no measurement" per LaserScan convention.
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - SECTOR_WIDTH
        scan.angle_increment = SECTOR_WIDTH
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = MIN_RANGE
        scan.range_max = MAX_RANGE
        scan.ranges = [r if math.isfinite(r) else float('nan') for r in ranges]
        scan.intensities = []
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = LidarToObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
