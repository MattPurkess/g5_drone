#!/usr/bin/env python3
"""
Utility functions for converting AprilTag detections from the
camera optical frame to world ENU coordinates using tf2.
"""
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PointStamped
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np


def detection_to_camera_pose(detection) -> PoseStamped:
    """Convert a single AprilTagDetection into a PoseStamped in the camera frame.

    The Jazzy apriltag_msgs AprilTagDetection message provides a 2D centre
    point instead of a full pose, so we map the detection centre into a
    zero-height pose for downstream TF handling.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'camera_down_link'
    pose_stamped.pose.position.x = float(detection.centre.x)
    pose_stamped.pose.position.y = float(detection.centre.y)
    pose_stamped.pose.position.z = 0.0
    pose_stamped.pose.orientation.w = 1.0
    return pose_stamped


def transform_to_world(node: Node,
                        tf_buffer: tf2_ros.Buffer,
                        pose_cam: PoseStamped,
                        world_frame: str = 'map') -> PoseStamped:
    """Transform pose_cam (in camera frame) to world_frame using tf2. Returns None if unavailable."""
    try:
        pose_world = tf_buffer.transform(
            pose_cam, world_frame,
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
        return pose_world
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        node.get_logger().warn(f'TF transform failed: {e}')
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
