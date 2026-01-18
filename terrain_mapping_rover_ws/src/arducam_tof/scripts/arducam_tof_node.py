#!/usr/bin/env python3
"""
Arducam ToF Camera Node - ROS 2 driver. 

Publishes:
- /tof/depth/image_raw (sensor_msgs/Image) - 16-bit depth image
- /tof/confidence/image_raw (sensor_msgs/Image) - 8-bit confidence
- /tof/amplitude/image_raw (sensor_msgs/Image) - 16-bit IR amplitude
- /tof/points (sensor_msgs/PointCloud2) - 3D point cloud
- /tof/camera_info (sensor_msgs/CameraInfo)

Author: Rover Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import numpy as np

from arducam_tof import ArducamToFDriver, PointCloudGenerator
from arducam_tof.tof_driver import ToFConfig
from arducam_tof. pointcloud_generator import Intrinsics


class ArducamToFNode(Node):
    """ROS 2 node for Arducam ToF camera."""
    
    def __init__(self):
        super().__init__('arducam_tof_node')
        
        self._declare_parameters()
        config = self._get_config()
        
        self.driver = ArducamToFDriver(config)
        
        if not self. driver.initialize():
            self.get_logger().error('Failed to initialize Arducam ToF!')
            raise RuntimeError('ToF initialization failed')
        
        self.get_logger().info('Arducam ToF initialized')
        
        self._setup_intrinsics()
        
        self. pc_generator = PointCloudGenerator()
        self._setup_pointcloud_generator()
        
        self.bridge = CvBridge()
        
        self. frame_id = self.get_parameter('frame_id').value
        self.optical_frame_id = self.get_parameter('optical_frame_id').value
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.depth_pub = self.create_publisher(Image, '/tof/depth/image_raw', sensor_qos)
        self.confidence_pub = self. create_publisher(Image, '/tof/confidence/image_raw', sensor_qos)
        self.amplitude_pub = self. create_publisher(Image, '/tof/amplitude/image_raw', sensor_qos)
        self.points_pub = self.create_publisher(PointCloud2, '/tof/points', sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/tof/camera_info', sensor_qos)
        
        update_rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0 / update_rate, self._capture_callback)
        
        self.get_logger().info(f'Publishing ToF data at {update_rate} Hz')
    
    def _declare_parameters(self):
        """Declare all node parameters."""
        self.declare_parameter('update_rate', 15.0)
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('optical_frame_id', 'tof_optical_frame')
        self.declare_parameter('max_range_m', 3.0)
        self.declare_parameter('min_range_m', 0.1)
        self.declare_parameter('confidence_threshold', 30)
        self.declare_parameter('use_yaml_intrinsics', False)
        self.declare_parameter('intrinsics_yaml_path', '')
        self.declare_parameter('tof_fx', 240.0)
        self.declare_parameter('tof_fy', 180.0)
        self.declare_parameter('tof_cx', 120.0)
        self.declare_parameter('tof_cy', 90.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('publish_pointcloud', True)
    
    def _get_config(self):
        """Build ToF config from parameters."""
        return ToFConfig(
            max_range_m=self.get_parameter('max_range_m').value,
            confidence_threshold=self.get_parameter('confidence_threshold').value
        )
    
    def _setup_intrinsics(self):
        """Set up camera intrinsics."""
        fx = self.get_parameter('tof_fx').value
        fy = self.get_parameter('tof_fy').value
        cx = self.get_parameter('tof_cx').value
        cy = self.get_parameter('tof_cy').value
        depth_scale = self.get_parameter('depth_scale').value
        self.driver.set_intrinsics(fx, fy, cx, cy, depth_scale)
        self.get_logger().info(f'ToF intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}')
    
    def _setup_pointcloud_generator(self):
        """Set up point cloud generator with intrinsics."""
        intr = self.driver.intrinsics
        self.pc_generator.set_intrinsics(Intrinsics(
            fx=intr.fx, fy=intr.fy, cx=intr.cx, cy=intr.cy,
            width=intr.width, height=intr.height, depth_scale=intr.depth_scale
        ))
    
    def _capture_callback(self):
        """Capture and publish ToF data."""
        frame = self.driver.capture()
        if frame is None:
            return
        
        now = self. get_clock().now()
        
        depth_msg = self.bridge.cv2_to_imgmsg(frame.depth_image, encoding='16UC1')
        depth_msg.header.stamp = now. to_msg()
        depth_msg.header.frame_id = self.optical_frame_id
        self.depth_pub.publish(depth_msg)
        
        conf_msg = self.bridge.cv2_to_imgmsg(frame.confidence_image, encoding='8UC1')
        conf_msg.header = depth_msg. header
        self.confidence_pub.publish(conf_msg)
        
        amp_msg = self.bridge.cv2_to_imgmsg(frame.amplitude_image, encoding='16UC1')
        amp_msg.header = depth_msg.header
        self.amplitude_pub.publish(amp_msg)
        
        self._publish_camera_info(depth_msg.header)
        
        if self.get_parameter('publish_pointcloud').value:
            max_range = self.get_parameter('max_range_m').value
            min_range = self.get_parameter('min_range_m').value
            pc_msg = self.pc_generator.to_ros_pointcloud2(
                frame.depth_image, depth_msg.header,
                max_depth_m=max_range, min_depth_m=min_range
            )
            self.points_pub. publish(pc_msg)
    
    def _publish_camera_info(self, header):
        """Publish camera info message."""
        intr = self.driver. intrinsics
        msg = CameraInfo()
        msg.header = header
        msg. width = intr.width
        msg.height = intr.height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [intr.fx, 0.0, intr. cx, 0.0, 0.0, intr. fy, intr.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(msg)
    
    def destroy_node(self):
        """Clean shutdown."""
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArducamToFNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Failed to start ToF node: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
