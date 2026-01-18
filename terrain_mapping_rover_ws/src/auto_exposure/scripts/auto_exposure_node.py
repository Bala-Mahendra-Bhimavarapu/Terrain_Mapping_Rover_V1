#!/usr/bin/env python3
"""
Auto-Exposure Controller Node.

Subscribes to camera images and IMU data to automatically
adjust camera exposure for optimal SLAM performance in
challenging lunar lighting conditions. 

Author: Rover Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import numpy as np

from rover_interfaces.msg import (
    CameraParams, ExposureState, 
    Classification, ClassificationArray
)

from auto_exposure import ExposureController, BrightnessAnalyzer
from auto_exposure.exposure_controller import ExposureLimits


class AutoExposureNode(Node):
    """Auto-exposure controller node."""
    
    def __init__(self):
        super().__init__('auto_exposure_node')
        
        self._declare_parameters()
        
        # Initialize components
        limits = ExposureLimits(
            min_exposure_us=self. get_parameter('min_exposure_time_ms').value * 1000,
            max_exposure_us=self.get_parameter('max_exposure_time_ms').value * 1000,
            min_gain=self. get_parameter('min_gain').value,
            max_gain=self.get_parameter('max_gain').value
        )
        
        self. controller = ExposureController(
            target_brightness=self.get_parameter('target_brightness').value,
            tolerance=self.get_parameter('brightness_tolerance').value,
            limits=limits
        )
        
        roi_fraction = self.get_parameter('roi_fraction').value
        self. roi = (
            (1 - roi_fraction) / 2,
            (1 - roi_fraction) / 2,
            roi_fraction,
            roi_fraction
        )
        
        self.analyzer = BrightnessAnalyzer()
        self.bridge = CvBridge()
        
        # Motion detection state
        self. motion_threshold = self.get_parameter('motion_threshold').value
        self.lock_during_motion = self.get_parameter('lock_during_motion').value
        self.motion_detected = False
        self.last_angular_velocity = 0.0
        
        # Classification feedback
        self.use_classification = self.get_parameter('use_classification_feedback').value
        self.min_classification_conf = self.get_parameter('min_classification_confidence').value
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, sensor_qos)
        self.imu_sub = self. create_subscription(
            Imu, '/imu/data', self._imu_callback, sensor_qos)
        self.classification_sub = self.create_subscription(
            ClassificationArray, '/ai_camera/classification',
            self._classification_callback, 10)
        
        # Publishers
        self.params_pub = self.create_publisher(CameraParams, '/camera/set_params', 10)
        self.state_pub = self.create_publisher(ExposureState, '/auto_exposure/state', 10)
        
        # Timer for periodic updates
        rate = self.get_parameter('adjustment_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._control_callback)
        
        # State
        self.latest_image = None
        self. latest_brightness = 128.0
        
        self.get_logger().info('Auto-exposure controller started')
    
    def _declare_parameters(self):
        """Declare all parameters."""
        self.declare_parameter('target_brightness', 128.0)
        self.declare_parameter('brightness_tolerance', 15.0)
        self.declare_parameter('max_exposure_time_ms', 50. 0)
        self.declare_parameter('min_exposure_time_ms', 0.1)
        self.declare_parameter('max_gain', 8.0)
        self.declare_parameter('min_gain', 1.0)
        self.declare_parameter('roi_fraction', 0.5)
        self.declare_parameter('adjustment_rate_hz', 5.0)
        self.declare_parameter('lock_during_motion', True)
        self.declare_parameter('motion_threshold', 0.5)
        self.declare_parameter('use_classification_feedback', False)
        self.declare_parameter('min_classification_confidence', 0.7)
    
    def _image_callback(self, msg:  Image):
        """Handle incoming image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
    
    def _imu_callback(self, msg:  Imu):
        """Handle IMU data for motion detection."""
        angular_vel = np.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity. y**2 +
            msg.angular_velocity.z**2
        )
        
        self.motion_detected = angular_vel > self.motion_threshold
        self.last_angular_velocity = angular_vel
        
        if self.lock_during_motion and self.motion_detected:
            self.controller.lock()
        elif self.lock_during_motion and not self.motion_detected:
            self.controller.unlock()
    
    def _classification_callback(self, msg: ClassificationArray):
        """Handle classification feedback."""
        if not self.use_classification:
            return
        
        # Adjust target brightness based on classification confidence
        for cls in msg.classifications:
            if cls.confidence < self.min_classification_conf:
                # Low confidence might mean poor exposure
                # Slightly adjust target
                pass
    
    def _control_callback(self):
        """Periodic control update."""
        if self.latest_image is None:
            return
        
        # Analyze brightness
        result = self.analyzer.analyze(self.latest_image, self.roi)
        self.latest_brightness = result.recommended_brightness
        
        # Update controller
        new_settings, changed = self.controller.update(self.latest_brightness)
        
        # Publish new settings if changed
        if changed:
            params = CameraParams()
            params.header.stamp = self.get_clock().now().to_msg()
            params.ae_enabled = False
            params.exposure_time_us = float(new_settings.exposure_time_us)
            params.analogue_gain = new_settings.analogue_gain
            params.digital_gain = new_settings.digital_gain
            params.target_brightness = self.controller.target_brightness
            self.params_pub.publish(params)
        
        # Publish state
        state = ExposureState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.current_brightness = self.latest_brightness
        state.target_brightness = self.controller.target_brightness
        state.exposure_time_us = float(new_settings.exposure_time_us)
        state.analogue_gain = new_settings.analogue_gain
        state.digital_gain = new_settings.digital_gain
        state.ae_locked = self.controller.is_locked
        state.motion_detected = self.motion_detected
        state.adjustment_state = 'LOCKED' if self.controller.is_locked else 'ADJUSTING'
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExposureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
