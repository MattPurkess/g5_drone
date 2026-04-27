#!/usr/bin/env python3
"""
Utility functions for getting AprilTag world position from TF
and computing lateral error for landing guidance.
"""
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
import numpy as np

TAG_FRAME    = 'tag36h11:0'   # confirm with: ros2 run tf2_tools view_frames
CAMERA_FRAME = 'camera_down_link'
MAX_TAG_AGE_S = 0.5           # reject transforms older than this


def get_tag_world_pose(node: Node,
                       tf_buffer: tf2_ros.Buffer,
                       tag_frame: str = TAG_FRAME,
                       world_frame: str = 'map') -> PoseStamped | None:
    """
    Look up the tag pose in world_frame directly from the TF tree.
    apriltag_ros publishes camera_down_link -> tag36h11:0 to /tf,
    so once base_link -> camera_down_link is also in the tree,
    TF can chain all the way from map to the tag automatically.
    Returns None if the transform is unavailable or too stale.
    """
    try:
        t = tf_buffer.lookup_transform(
            world_frame,
            tag_frame,
            rclpy.time.Time(),  # latest available
            timeout=rclpy.duration.Duration(seconds=0.1)
        )

        # Reject stale transforms (tag has left FOV)
        age = (node.get_clock().now()
               - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > MAX_TAG_AGE_S:
            node.get_logger().warn(f'Tag transform is {age:.2f}s old, ignoring')
            return None

        pose = PoseStamped()
        pose.header.stamp    = t.header.stamp
        pose.header.frame_id = world_frame
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation
        return pose

    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        node.get_logger().warn(f'Tag TF lookup failed: {e}')
        return None


def lateral_error(drone_pose: PoseStamped,
                  tag_world:  PoseStamped) -> tuple:
    """
    Compute (ex, ey) lateral error in metres (ENU world frame).
    Positive ex: tag is East  of drone.
    Positive ey: tag is North of drone.
    """
    ex = tag_world.pose.position.x - drone_pose.pose.position.x
    ey = tag_world.pose.position.y - drone_pose.pose.position.y
    return ex, ey


def euclidean_2d(ex: float, ey: float) -> float:
    return float(np.sqrt(ex**2 + ey**2))