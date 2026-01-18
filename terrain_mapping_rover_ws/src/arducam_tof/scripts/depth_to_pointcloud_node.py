#!/usr/bin/env python3
"""
Depth to PointCloud Node - Converts depth images to point clouds. 

This is a standalone node that subscribes to depth images and publishes
point clouds.  Useful when running ToF driver separately.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import message_filters


class DepthToPointCloudNode(Node):
    """Converts depth images to point clouds."""
    
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')
        
        self. declare_parameter('depth_topic', '/tof/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/tof/camera_info')
        self.declare_parameter('points_topic', '/tof/points')
        self.declare_parameter('max_range_m', 3.0)
        self.declare_parameter('min_range_m', 0.1)
        self.declare_parameter('depth_scale', 0.001)
        
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        points_topic = self. get_parameter('points_topic').value
        self.max_range = self.get_parameter('max_range_m').value
        self.min_range = self.get_parameter('min_range_m').value
        self.depth_scale = self.get_parameter('depth_scale').value
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.pixel_coords = None
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._info_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, sensor_qos)
        self.points_pub = self.create_publisher(PointCloud2, points_topic, sensor_qos)
        
        self.get_logger().info('Depth to PointCloud node started')
    
    def _info_callback(self, msg):
        """Store camera info and precompute pixel coordinates."""
        if self.camera_info is not None:
            return
        
        self.camera_info = msg
        
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        
        u = np.arange(msg. width)
        v = np.arange(msg.height)
        u, v = np.meshgrid(u, v)
        
        self.pixel_coords = np.stack([
            (u - cx) / fx,
            (v - cy) / fy,
            np.ones_like(u)
        ], axis=-1).astype(np.float32)
        
        self.get_logger().info(f'Camera info received: {msg.width}x{msg.height}')
    
    def _depth_callback(self, msg):
        """Convert depth image to point cloud."""
        if self.pixel_coords is None:
            return
        
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return
        
        depth_m = depth_image. astype(np.float32) * self.depth_scale
        valid_mask = (depth_m > self.min_range) & (depth_m < self.max_range)
        points = self.pixel_coords * depth_m[: , :, np.newaxis]
        valid_points = points[valid_mask]
        
        pc_msg = PointCloud2()
        pc_msg.header = msg.header
        pc_msg.height = 1
        pc_msg.width = len(valid_points)
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.is_dense = True
        pc_msg.data = valid_points.astype(np.float32).tobytes()
        
        self. points_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
