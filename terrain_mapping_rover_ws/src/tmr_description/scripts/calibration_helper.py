#!/usr/bin/env python3
"""
Calibration Helper Node

Interactive tool for calibrating sensor positions.
Provides real-time feedback and saves calibration data.

Usage:
    ros2 run tmr_description calibration_helper.py
"""

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image, Imu, PointCloud2
from geometry_msgs.msg import TransformStamped

import yaml
import os
import time
from datetime import datetime
from typing import Optional


class CalibrationHelperNode(Node):
    """Interactive calibration helper."""
    
    def __init__(self):
        super().__init__('calibration_helper')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Sensor data tracking
        self.imu_data: Optional[Imu] = None
        self. camera_data_received = False
        self.tof_data_received = False
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self. camera_callback, 10)
        self.tof_sub = self.create_subscription(
            PointCloud2, '/tof/points', self.tof_callback, 10)
        
        # Timer for status
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info("Calibration Helper started")
        self.get_logger().info("Press Ctrl+C to save and exit")
    
    def imu_callback(self, msg:  Imu):
        self.imu_data = msg
    
    def camera_callback(self, msg: Image):
        self.camera_data_received = True
    
    def tof_callback(self, msg: PointCloud2):
        self.tof_data_received = True
    
    def print_status(self):
        """Print current calibration status."""
        print("\033[2J\033[H", end="")
        
        print("=" * 60)
        print("  CALIBRATION HELPER")
        print("=" * 60)
        print()
        
        # Sensor connectivity
        print("Sensor Status:")
        print(f"  IMU:     {'✅ Receiving' if self.imu_data else '❌ No data'}")
        print(f"  Camera: {'✅ Receiving' if self.camera_data_received else '❌ No data'}")
        print(f"  ToF:    {'✅ Receiving' if self.tof_data_received else '❌ No data'}")
        print()
        
        # IMU orientation (for checking mounting)
        if self.imu_data:
            ax = self.imu_data.linear_acceleration.x
            ay = self.imu_data.linear_acceleration.y
            az = self.imu_data.linear_acceleration.z
            
            print("IMU Accelerometer (m/s²):")
            print(f"  X: {ax: +7.3f}  (forward)")
            print(f"  Y: {ay:+7.3f}  (left)")
            print(f"  Z:  {az:+7.3f}  (up, expected ~9.81)")
            print()
            
            # Check orientation
            if abs(az - 9.81) < 1. 0 and abs(ax) < 1.0 and abs(ay) < 1.0:
                print("  ✅ IMU appears correctly oriented (level)")
            elif abs(ax - 9.81) < 1.0:
                print("  ⚠️  IMU may be tilted forward 90°")
            elif abs(ay - 9.81) < 1.0:
                print("  ⚠️  IMU may be tilted left 90°")
            else:
                print("  ⚠️  IMU orientation unusual - check mounting")
            print()
        
        # Current transform values
        print("Current Transforms (from URDF):")
        
        transforms_to_show = [
            ('base_link', 'camera_link'),
            ('base_link', 'tof_link'),
            ('base_link', 'imu_link'),
        ]
        
        for parent, child in transforms_to_show: 
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                t = tf.transform.translation
                print(f"  {parent} -> {child}:")
                print(f"    Position: ({t.x:+. 4f}, {t.y:+.4f}, {t. z:+.4f}) m")
            except Exception: 
                print(f"  {parent} -> {child}: ❌ Not available")
        
        print()
        print("=" * 60)
        print("Adjust positions via launch arguments:")
        print("  ros2 launch tmr_description calibration. launch.py \\")
        print("    camera_x: =0.14 camera_pitch:=-0.28")
        print("=" * 60)
        print("\nPress Ctrl+C to exit")
        
        # Reset flags
        self.camera_data_received = False
        self.tof_data_received = False
    
    def save_calibration(self):
        """Save current calibration to file."""
        calibration = {
            'timestamp': datetime. now().isoformat(),
            'imu':  {
                'position': {'x': 0.0, 'y':  0.0, 'z': 0.046},
                'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            },
            'camera': {
                'position': {'x': 0.13, 'y': -0.0275, 'z': 0.078},
                'orientation': {'roll': 0.0, 'pitch': -0.2751, 'yaw': 0.0}
            },
            'tof': {
                'position': {'x': 0.13, 'y': 0.014, 'z': 0.078},
                'orientation': {'roll': 0.0, 'pitch': -0.2751, 'yaw': 0.0}
            }
        }
        
        filename = f"calibration_{datetime. now().strftime('%Y%m%d_%H%M%S')}.yaml"
        
        with open(filename, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False)
        
        self.get_logger().info(f"Calibration saved to {filename}")


def main():
    rclpy.init()
    
    node = CalibrationHelperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        node.save_calibration()
    finally:
        node. destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
