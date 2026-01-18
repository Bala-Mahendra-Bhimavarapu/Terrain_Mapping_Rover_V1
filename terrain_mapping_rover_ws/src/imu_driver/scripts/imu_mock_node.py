#!/usr/bin/env python3
"""
Mock IMU Node for testing without hardware.

Publishes simulated IMU data. 
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import random


class IMUMockNode(Node):
    """Mock IMU node for testing."""
    
    def __init__(self):
        super().__init__('imu_mock_node')
        
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('noise_level', 0.01)
        
        self.frame_id = self.get_parameter('frame_id').value
        update_rate = self.get_parameter('update_rate').value
        self.noise = self.get_parameter('noise_level').value
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(1.0 / update_rate, self._timer_callback)
        
        self.get_logger().info('IMU Mock Node started (SIMULATION MODE)')
        self.get_logger().warn('=' * 50)
        self.get_logger().warn('  THIS IS A MOCK NODE - NO REAL HARDWARE')
        self.get_logger().warn('=' * 50)
    
    def _timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Simulated stationary IMU with noise
        msg.linear_acceleration.x = random.gauss(0, self.noise)
        msg.linear_acceleration. y = random.gauss(0, self.noise)
        msg.linear_acceleration.z = 9.81 + random.gauss(0, self.noise)
        
        msg.angular_velocity.x = random.gauss(0, self.noise * 0.1)
        msg.angular_velocity.y = random.gauss(0, self. noise * 0.1)
        msg.angular_velocity.z = random.gauss(0, self.noise * 0.1)
        
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Set covariances
        msg.orientation_covariance = [0.01, 0, 0, 0, 0. 01, 0, 0, 0, 0.01]
        msg.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
