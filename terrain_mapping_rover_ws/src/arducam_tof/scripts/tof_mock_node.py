#!/usr/bin/env python3
"""
ToF Mock Node - Simulates ToF camera for testing without hardware. 
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct


class ToFMockNode(Node):
    """Mock ToF node for testing."""
    
    def __init__(self):
        super().__init__('tof_mock_node')
        
        self.declare_parameter('width', 240)
        self.declare_parameter('height', 180)
        self.declare_parameter('update_rate', 15.0)
        self.declare_parameter('optical_frame_id', 'tof_optical_frame')
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('optical_frame_id').value
        update_rate = self.get_parameter('update_rate').value
        
        self.bridge = CvBridge()
        
        self.depth_pub = self.create_publisher(Image, '/tof/depth/image_raw', 10)
        self.confidence_pub = self. create_publisher(Image, '/tof/confidence/image_raw', 10)
        self.amplitude_pub = self.create_publisher(Image, '/tof/amplitude/image_raw', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/tof/points', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/tof/camera_info', 10)
        
        self.timer = self.create_timer(1.0 / update_rate, self._timer_callback)
        self.frame_count = 0
        
        self.get_logger().info('ToF Mock Node started (SIMULATION MODE)')
        self.get_logger().warn('=' * 50)
        self.get_logger().warn('  THIS IS A MOCK NODE - NO REAL HARDWARE')
        self.get_logger().warn('=' * 50)
    
    def _timer_callback(self):
        now = self.get_clock().now()
        
        depth_image = self._generate_depth_image()
        confidence_image = np.full((self.height, self.width), 200, dtype=np.uint8)
        amplitude_image = np.full((self.height, self.width), 1000, dtype=np.uint16)
        
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        depth_msg.header.stamp = now.to_msg()
        depth_msg.header.frame_id = self.frame_id
        self.depth_pub.publish(depth_msg)
        
        conf_msg = self.bridge.cv2_to_imgmsg(confidence_image, encoding='8UC1')
        conf_msg.header = depth_msg.header
        self.confidence_pub.publish(conf_msg)
        
        amp_msg = self.bridge.cv2_to_imgmsg(amplitude_image, encoding='16UC1')
        amp_msg.header = depth_msg.header
        self.amplitude_pub.publish(amp_msg)
        
        self._publish_camera_info(depth_msg.header)
        self._publish_pointcloud(depth_image, depth_msg.header)
        
        self.frame_count += 1
    
    def _generate_depth_image(self):
        """Generate simulated depth image with objects."""
        depth = np.full((self.height, self.width), 2000, dtype=np.uint16)
        
        center_y, center_x = self.height // 2, self.width // 2
        y, x = np.ogrid[: self.height, :self.width]
        
        obstacle_x = center_x + int(30 * np.sin(self.frame_count * 0.05))
        mask = (x - obstacle_x)**2 + (y - center_y)**2 < 400
        depth[mask] = 800
        
        return depth
    
    def _publish_camera_info(self, header):
        """Publish camera info."""
        msg = CameraInfo()
        msg.header = header
        msg. width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'
        msg. d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [240.0, 0.0, 120.0, 0.0, 180.0, 90.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [240.0, 0.0, 120.0, 0.0, 0.0, 180.0, 90.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self. info_pub.publish(msg)
    
    def _publish_pointcloud(self, depth_image, header):
        """Generate and publish point cloud."""
        fx, fy = 240.0, 180.0
        cx, cy = 120.0, 90.0
        depth_scale = 0.001
        
        u = np.arange(self.width)
        v = np.arange(self.height)
        u, v = np.meshgrid(u, v)
        
        z = depth_image. astype(np.float32) * depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        valid = z > 0.1
        points = np.stack([x[valid], y[valid], z[valid]], axis=-1)
        
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField. FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        
        self. points_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToFMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
