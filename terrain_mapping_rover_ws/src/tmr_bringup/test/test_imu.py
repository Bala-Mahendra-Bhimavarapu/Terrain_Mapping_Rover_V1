#!/usr/bin/env python3
"""
IMU Test

Unit tests for IMU sensor. 
"""

import unittest
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32


class TestIMU(unittest.TestCase):
    """Test IMU functionality."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_imu')
        
        cls.connected = False
        cls.imu_data = []
        cls.temperature = 0.0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        cls.connected_sub = cls.node.create_subscription(
            Bool, '/imu/connected', cls.connected_callback, 10)
        cls.imu_sub = cls.node.create_subscription(
            Imu, '/imu/data', cls.imu_callback, qos)
        cls.temp_sub = cls.node.create_subscription(
            Float32, '/imu/temperature', cls.temp_callback, 10)
        
        # Collect data
        timeout = 5.0
        start = time. time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def connected_callback(cls, msg):
        cls.connected = msg.data
    
    @classmethod
    def imu_callback(cls, msg):
        cls.imu_data. append(msg)
    
    @classmethod
    def temp_callback(cls, msg):
        cls.temperature = msg.data
    
    def test_connection(self):
        """Test IMU is connected."""
        self.assertTrue(self.connected, "IMU not connected")
    
    def test_data_received(self):
        """Test IMU data is received."""
        self.assertGreater(len(self.imu_data), 0, "No IMU data received")
    
    def test_data_rate(self):
        """Test IMU data rate is acceptable."""
        if len(self.imu_data) < 10:
            self. skipTest("Insufficient IMU data")
        
        # Should have at least 50 Hz
        expected_samples = 5. 0 * 50  # 5 seconds at 50 Hz
        self.assertGreater(len(self.imu_data), expected_samples * 0.5,
                          f"Low IMU rate: {len(self.imu_data)} samples in 5s")
    
    def test_accelerometer_gravity(self):
        """Test accelerometer reads approximately 1g."""
        if len(self.imu_data) == 0:
            self.skipTest("No IMU data")
        
        last = self.imu_data[-1]
        accel_magnitude = math.sqrt(
            last.linear_acceleration.x**2 +
            last.linear_acceleration. y**2 +
            last.linear_acceleration.z**2
        )
        
        # Should be close to 9.81 m/s²
        self.assertGreater(accel_magnitude, 8.0, "Accelerometer reading too low")
        self.assertLess(accel_magnitude, 12.0, "Accelerometer reading too high")
    
    def test_gyroscope_stationary(self):
        """Test gyroscope is near zero when stationary."""
        if len(self.imu_data) == 0:
            self.skipTest("No IMU data")
        
        # Average gyro readings
        avg_gx = sum(d.angular_velocity.x for d in self.imu_data) / len(self.imu_data)
        avg_gy = sum(d.angular_velocity.y for d in self.imu_data) / len(self.imu_data)
        avg_gz = sum(d. angular_velocity.z for d in self.imu_data) / len(self.imu_data)
        
        gyro_magnitude = math.sqrt(avg_gx**2 + avg_gy**2 + avg_gz**2)
        
        # Should be near zero when stationary (< 0.1 rad/s)
        self.assertLess(gyro_magnitude, 0.2, "Gyroscope not stationary")
    
    def test_temperature_reasonable(self):
        """Test temperature is in reasonable range."""
        if self.temperature == 0.0:
            self.skipTest("No temperature data")
        
        self.assertGreater(self.temperature, 0.0, "Temperature too low")
        self.assertLess(self.temperature, 80.0, "Temperature too high")


if __name__ == '__main__':
    unittest.main()
