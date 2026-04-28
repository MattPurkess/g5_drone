#!/usr/bin/env python3
"""
Utility functions for converting AprilTag detections from the
camera optical frame to world ENU coordinates using tf2.
(Configured for IBVS - Image Based Visual Servoing)
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import numpy as np

def detection_to_camera_pose(detection) -> PoseStamped:
    """
    Convert a single AprilTagDetection into a PoseStamped in the camera frame.
    Maps the 2D detection centre into a zero-height pose for IBVS TF handling.
    """
    pose_stamped = PoseStamped()
    
    # We explicitly set this to the downward camera link
    pose_stamped.header.frame_id = 'camera_link' 
    
    # Extract the 2D pixel coordinates from the detection instead of a 3D pose
    pose_stamped.pose.position.x = float(detection.centre.x)
    pose_stamped.pose.position.y = float(detection.centre.y)
    
    # Set height and orientation to dummy values
    pose_stamped.pose.position.z = 0.0
    pose_stamped.pose.orientation.w = 1.0
    
    return pose_stamped

def transform_to_world(node: Node,
                       tf_buffer: tf2_ros.Buffer,
                       pose_cam: PoseStamped,
                       world_frame: str = 'map') -> PoseStamped:
    """Transform pose_cam (in camera frame) to world_frame using tf2."""
    try:
        pose_world = tf_buffer.transform(
            pose_cam, 
            world_frame,
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
        return pose_world
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        node.get_logger().warn(f'TF transform failed: {e}')
        return None

def lateral_error(drone_pose: PoseStamped, tag_world: PoseStamped) -> tuple:
    """
    Compute (ex, ey) lateral error. 
    NOTE: In IBVS mode, this error is a hybrid of Pixels translated into the world frame.
    """
    ex = tag_world.pose.position.x - drone_pose.pose.position.x
    ey = tag_world.pose.position.y - drone_pose.pose.position.y
    return ex, ey

def euclidean_2d(ex: float, ey: float) -> float:
    return float(np.sqrt(ex**2 + ey**2))