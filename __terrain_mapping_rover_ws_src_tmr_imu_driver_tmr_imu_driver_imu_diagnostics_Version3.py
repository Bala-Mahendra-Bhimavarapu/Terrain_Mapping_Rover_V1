#!/usr/bin/env python3
"""
IMU Diagnostics Tool

Real-time monitoring and visualization of IMU data. 

Usage:
    ros2 run tmr_imu_driver imu_diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool

import math
import time
import sys
from collections import deque


class IMUDiagnosticsNode(Node):
    """
    Diagnostic node for monitoring IMU data quality.
    """
    
    def __init__(self):
        super().__init__('imu_diagnostics')
        
        # Data buffers for statistics
        self.accel_x_buffer = deque(maxlen=100)
        self.accel_y_buffer = deque(maxlen=100)
        self.accel_z_buffer = deque(maxlen=100)
        self.gyro_x_buffer = deque(maxlen=100)
        self.gyro_y_buffer = deque(maxlen=100)
        self.gyro_z_buffer = deque(maxlen=100)
        
        # Rate tracking
        self.msg_count = 0
        self.last_rate_time = time.time()
        self.measured_rate = 0.0
        self.last_msg_time = None
        
        # Status
        self.connected = False
        self.temperature = 0.0
        
        # Create QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.temp_sub = self.create_subscription(
            Float32, '/imu/temperature', self.temp_callback, 10)
        self.conn_sub = self.create_subscription(
            Bool, '/imu/connected', self.connected_callback, 10)
        
        # Display timer
        self.display_timer = self.create_timer(0.5, self.display_callback)
        
        self.get_logger().info('IMU Diagnostics started')
        self.get_logger().info('Monitoring /imu/data.. .')
    
    def imu_callback(self, msg:  Imu):
        """Process IMU message"""
        # Store data
        self.accel_x_buffer.append(msg.linear_acceleration.x)
        self.accel_y_buffer. append(msg.linear_acceleration. y)
        self.accel_z_buffer.append(msg.linear_acceleration.z)
        self.gyro_x_buffer.append(msg.angular_velocity.x)
        self.gyro_y_buffer.append(msg.angular_velocity.y)
        self.gyro_z_buffer.append(msg. angular_velocity.z)
        
        # Update rate tracking
        self.msg_count += 1
        self.last_msg_time = time.time()
    
    def temp_callback(self, msg: Float32):
        """Process temperature message"""
        self.temperature = msg.data
    
    def connected_callback(self, msg: Bool):
        """Process connection status"""
        self. connected = msg.data
    
    def calc_stats(self, buffer):
        """Calculate mean and std of buffer"""
        if len(buffer) == 0:
            return 0.0, 0.0
        
        mean = sum(buffer) / len(buffer)
        variance = sum((x - mean)**2 for x in buffer) / len(buffer)
        std = math.sqrt(variance)
        
        return mean, std
    
    def display_callback(self):
        """Display diagnostics"""
        # Calculate rate
        current_time = time.time()
        elapsed = current_time - self.last_rate_time
        
        if elapsed > 0:
            self.measured_rate = self.msg_count / elapsed
        
        self.msg_count = 0
        self.last_rate_time = current_time
        
        # Calculate statistics
        accel_x_mean, accel_x_std = self.calc_stats(self.accel_x_buffer)
        accel_y_mean, accel_y_std = self.calc_stats(self. accel_y_buffer)
        accel_z_mean, accel_z_std = self.calc_stats(self.accel_z_buffer)
        gyro_x_mean, gyro_x_std = self. calc_stats(self.gyro_x_buffer)
        gyro_y_mean, gyro_y_std = self.calc_stats(self.gyro_y_buffer)
        gyro_z_mean, gyro_z_std = self.calc_stats(self.gyro_z_buffer)
        
        # Calculate total acceleration magnitude
        accel_magnitude = math.sqrt(accel_x_mean**2 + accel_y_mean**2 + accel_z_mean**2)
        
        # Clear screen and display
        print('\033[2J\033[H', end='')  # Clear screen
        
        print('=' * 60)
        print('           IMU DIAGNOSTICS')
        print('=' * 60)
        
        # Connection status
        status_str = '\033[92mCONNECTED\033[0m' if self. connected else '\033[91mDISCONNECTED\033[0m'
        print(f'\nStatus: {status_str}')
        print(f'Data Rate: {self.measured_rate:.1f} Hz')
        print(f'Temperature: {self.temperature:.1f} °C')
        
        print('\n' + '-' * 60)
        print('ACCELEROMETER (m/s²)')
        print('-' * 60)
        print(f'  X: {accel_x_mean: +8.4f}  (σ = {accel_x_std:. 4f})')
        print(f'  Y: {accel_y_mean:+8.4f}  (σ = {accel_y_std:. 4f})')
        print(f'  Z: {accel_z_mean:+8.4f}  (σ = {accel_z_std:. 4f})')
        print(f'  Magnitude: {accel_magnitude:. 4f} (expected: ~9.81)')
        
        print('\n' + '-' * 60)
        print('GYROSCOPE (rad/s)')
        print('-' * 60)
        print(f'  X: {gyro_x_mean:+8.6f}  (σ = {gyro_x_std:.6f})')
        print(f'  Y: {gyro_y_mean:+8.6f}  (σ = {gyro_y_std:.6f})')
        print(f'  Z: {gyro_z_mean:+8.6f}  (σ = {gyro_z_std:.6f})')
        
        # Visual indicators
        print('\n' + '-' * 60)
        print('QUALITY INDICATORS')
        print('-' * 60)
        
        # Check if stationary (low gyro, accel near 1g)
        gyro_magnitude = math.sqrt(gyro_x_mean**2 + gyro_y_mean**2 + gyro_z_mean**2)
        
        if gyro_magnitude < 0.01: 
            print('  Rotation: \033[92m● Stationary\033[0m')
        elif gyro_magnitude < 0.1:
            print('  Rotation: \033[93m● Slow movement\033[0m')
        else:
            print('  Rotation: \033[91m● Fast movement\033[0m')
        
        if 9.5 < accel_magnitude < 10.1:
            print('  Gravity: \033[92m● Normal (1g)\033[0m')
        else:
            print(f'  Gravity: \033[93m● Abnormal ({accel_magnitude/9.81:.2f}g)\033[0m')
        
        if self.temperature < 50:
            print('  Temperature:  \033[92m● Normal\033[0m')
        elif self.temperature < 60:
            print('  Temperature:  \033[93m● Warm\033[0m')
        else:
            print('  Temperature: \033[91m● Hot!\033[0m')
        
        if self.measured_rate > 90:
            print('  Data Rate: \033[92m● Good\033[0m')
        elif self.measured_rate > 50:
            print('  Data Rate: \033[93m● Low\033[0m')
        else:
            print('  Data Rate: \033[91m● Very Low\033[0m')
        
        print('\n' + '=' * 60)
        print('Press Ctrl+C to exit')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = IMUDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()