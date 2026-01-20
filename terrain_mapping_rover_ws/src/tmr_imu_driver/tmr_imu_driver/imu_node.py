#!/usr/bin/env python3
"""
IMU ROS 2 Node

Publishes IMU data from MPU6050 sensor. 

Published Topics:
    - /imu/data (sensor_msgs/Imu): Filtered IMU data with orientation
    - /imu/data_raw (sensor_msgs/Imu): Raw IMU data without orientation
    - /imu/temperature (std_msgs/Float32): Temperature reading
    - /imu/connected (std_msgs/Bool): Connection status

Services:
    - /imu/calibrate (std_srvs/Trigger): Trigger calibration
    - /imu/reset_orientation (std_srvs/Trigger): Reset orientation estimate

Usage:
    ros2 run tmr_imu_driver imu_node
    ros2 run tmr_imu_driver imu_node --ros-args -p publish_rate_hz:=100
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import time
import threading
import math
from typing import Optional, List

from .mpu6050 import MPU6050, IMUData
from .complementary_filter import ComplementaryFilter, Vector3Filter


class IMUNode(Node):
    """
    ROS 2 node for MPU6050 IMU. 
    """
    
    def __init__(self):
        super().__init__('imu_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Initialize IMU
        self.imu: Optional[MPU6050] = None
        self.connected = False
        
        # Initialize filters
        self.comp_filter = ComplementaryFilter(alpha=self. comp_alpha)
        self.accel_filter = Vector3Filter(window_size=self.ma_window)
        self.gyro_filter = Vector3Filter(window_size=self.ma_window)
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.imu_raw_pub = self. create_publisher(Imu, 'imu/data_raw', sensor_qos)
        self.temp_pub = self. create_publisher(Float32, 'imu/temperature', 10)
        self.connected_pub = self.create_publisher(Bool, 'imu/connected', 10)
        
        if self.publish_diagnostics:
            self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Create services
        self.calibrate_srv = self.create_service(
            Trigger, 'imu/calibrate', self. calibrate_callback)
        self.reset_orientation_srv = self.create_service(
            Trigger, 'imu/reset_orientation', self.reset_orientation_callback)
        
        # Statistics
        self.sample_count = 0
        self.last_rate_check_time = time.time()
        self.measured_rate = 0.0
        self.last_temp = 0.0
        
        # Lock for thread safety
        self. lock = threading.Lock()
        
        # Connect to IMU
        self._connect()
        
        # Create timers
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_callback)
        
        if self.publish_diagnostics:
            self.diag_timer = self.create_timer(
                1.0 / self.diag_rate, self.diagnostics_callback)
        
        self.get_logger().info('IMU Node initialized')
        self.get_logger().info(f'  I2C Bus: {self.i2c_bus}')
        self.get_logger().info(f'  I2C Address: 0x{self.i2c_address:02X}')
        self.get_logger().info(f'  Publish Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  Accel Range: ±{self.accel_range}g')
        self.get_logger().info(f'  Gyro Range: ±{self. gyro_range}°/s')
    
    def _declare_parameters(self):
        """Declare all node parameters"""
        # I2C configuration
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        
        # Publishing configuration
        self.declare_parameter('publish_rate_hz', 100. 0)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Sensor configuration
        self.declare_parameter('accel_range_g', 2)
        self.declare_parameter('gyro_range_dps', 250)
        self.declare_parameter('dlpf_mode', 3)
        
        # Calibration
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calibration_samples', 500)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)
        self.declare_parameter('gyro_offset_x', 0.0)
        self.declare_parameter('gyro_offset_y', 0.0)
        self.declare_parameter('gyro_offset_z', 0.0)
        
        # Filtering
        self.declare_parameter('use_complementary_filter', True)
        self.declare_parameter('complementary_alpha', 0.98)
        self.declare_parameter('use_moving_average', False)
        self.declare_parameter('moving_average_window', 5)
        
        # Covariance
        self.declare_parameter('orientation_covariance', 
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01])
        self.declare_parameter('angular_velocity_covariance',
            [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001])
        self.declare_parameter('linear_acceleration_covariance',
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01])
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('diagnostics_rate_hz', 1.0)
        self.declare_parameter('temperature_warning_c', 60.0)
        self.declare_parameter('rate_warning_threshold', 0.9)
    
    def _get_parameters(self):
        """Get all parameter values"""
        # I2C configuration
        self. i2c_bus = self. get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        
        # Handle hex string for address
        if isinstance(self.i2c_address, str):
            self.i2c_address = int(self.i2c_address, 16)
        
        # Publishing configuration
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Sensor configuration
        self.accel_range = self.get_parameter('accel_range_g').value
        self.gyro_range = self.get_parameter('gyro_range_dps').value
        self.dlpf_mode = self.get_parameter('dlpf_mode').value
        
        # Calibration
        self.auto_calibrate = self. get_parameter('auto_calibrate').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.accel_offset = [
            self.get_parameter('accel_offset_x').value,
            self.get_parameter('accel_offset_y').value,
            self.get_parameter('accel_offset_z').value,
        ]
        self.gyro_offset = [
            self.get_parameter('gyro_offset_x').value,
            self.get_parameter('gyro_offset_y').value,
            self.get_parameter('gyro_offset_z').value,
        ]
        
        # Filtering
        self.use_comp_filter = self.get_parameter('use_complementary_filter').value
        self.comp_alpha = self.get_parameter('complementary_alpha').value
        self.use_ma_filter = self.get_parameter('use_moving_average').value
        self.ma_window = self.get_parameter('moving_average_window').value
        
        # Covariance
        self.orientation_cov = list(self.get_parameter('orientation_covariance').value)
        self.angular_vel_cov = list(self. get_parameter('angular_velocity_covariance').value)
        self.linear_accel_cov = list(self.get_parameter('linear_acceleration_covariance').value)
        
        # Diagnostics
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self. diag_rate = self.get_parameter('diagnostics_rate_hz').value
        self.temp_warning = self.get_parameter('temperature_warning_c').value
        self.rate_warning = self.get_parameter('rate_warning_threshold').value
    
    def _connect(self):
        """Connect to the IMU"""
        self.get_logger().info('Connecting to MPU6050...')
        
        try:
            self.imu = MPU6050(
                bus=self.i2c_bus,
                address=self.i2c_address,
                accel_range_g=self.accel_range,
                gyro_range_dps=self.gyro_range,
                dlpf_mode=self.dlpf_mode
            )
            
            if self.imu. connect():
                self.connected = True
                self.get_logger().info('Connected to MPU6050 successfully')
                
                # Perform self-test
                if self.imu.self_test():
                    self.get_logger().info('IMU self-test passed')
                else:
                    self.get_logger().warn('IMU self-test failed')
                
                # Apply or perform calibration
                if self.auto_calibrate:
                    self._perform_calibration()
                else:
                    self._apply_stored_calibration()
            else:
                self.connected = False
                self.get_logger().error('Failed to connect to MPU6050')
                
        except Exception as e:
            self.connected = False
            self.get_logger().error(f'Exception connecting to MPU6050: {e}')
    
    def _perform_calibration(self):
        """Perform IMU calibration"""
        self.get_logger().info('Performing IMU calibration...')
        self.get_logger().info('Keep the robot stationary!')
        
        try:
            accel_offset, gyro_offset = self. imu.calibrate(
                samples=self.calibration_samples,
                delay=0.005
            )
            
            self. get_logger().info('Calibration complete: ')
            self.get_logger().info(f'  Accel offset: ({accel_offset[0]:. 4f}, {accel_offset[1]:.4f}, {accel_offset[2]:.4f})')
            self.get_logger().info(f'  Gyro offset: ({gyro_offset[0]:.6f}, {gyro_offset[1]:.6f}, {gyro_offset[2]:.6f})')
            
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
    
    def _apply_stored_calibration(self):
        """Apply pre-stored calibration values"""
        self. get_logger().info('Applying stored calibration values')
        
        self.imu.set_calibration(
            tuple(self.accel_offset),
            tuple(self.gyro_offset)
        )
        
        self.get_logger().info(f'  Accel offset: ({self.accel_offset[0]:.4f}, {self.accel_offset[1]:.4f}, {self.accel_offset[2]:.4f})')
        self.get_logger().info(f'  Gyro offset:  ({self.gyro_offset[0]:.6f}, {self.gyro_offset[1]:.6f}, {self.gyro_offset[2]:.6f})')
    
    def publish_callback(self):
        """Timer callback to publish IMU data"""
        # Publish connection status
        conn_msg = Bool()
        conn_msg.data = self.connected
        self.connected_pub.publish(conn_msg)
        
        if not self.connected or self.imu is None:
            # Try to reconnect periodically
            if not self.connected:
                self._connect()
            return
        
        try:
            with self.lock:
                # Read IMU data
                data = self.imu.read()
            
            # Update statistics
            self.sample_count += 1
            self.last_temp = data.temperature
            
            # Apply moving average filter if enabled
            if self.use_ma_filter:
                accel_x, accel_y, accel_z = self. accel_filter.update(
                    data.accel_x, data.accel_y, data.accel_z)
                gyro_x, gyro_y, gyro_z = self.gyro_filter.update(
                    data.gyro_x, data. gyro_y, data.gyro_z)
            else:
                accel_x, accel_y, accel_z = data.accel_x, data. accel_y, data.accel_z
                gyro_x, gyro_y, gyro_z = data.gyro_x, data.gyro_y, data.gyro_z
            
            # Get current timestamp
            now = self.get_clock().now()
            
            # Create raw IMU message (no orientation)
            raw_msg = self._create_imu_message(
                accel_x, accel_y, accel_z,
                gyro_x, gyro_y, gyro_z,
                now,
                include_orientation=False
            )
            self.imu_raw_pub. publish(raw_msg)
            
            # Update complementary filter and create filtered message
            if self.use_comp_filter:
                orientation = self.comp_filter.update(
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                    data.timestamp
                )
                
                filtered_msg = self._create_imu_message(
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                    now,
                    include_orientation=True,
                    quaternion=self.comp_filter.get_quaternion()
                )
            else:
                filtered_msg = raw_msg
            
            self.imu_pub.publish(filtered_msg)
            
            # Publish temperature
            temp_msg = Float32()
            temp_msg.data = data.temperature
            self.temp_pub.publish(temp_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU:  {e}')
            self.connected = False
    
    def _create_imu_message(
        self,
        accel_x: float, accel_y: float, accel_z: float,
        gyro_x: float, gyro_y: float, gyro_z: float,
        timestamp,
        include_orientation: bool = False,
        quaternion: tuple = None
    ) -> Imu:
        """
        Create sensor_msgs/Imu message. 
        
        Args:
            accel_*: Accelerometer readings in m/s^2
            gyro_*: Gyroscope readings in rad/s
            timestamp:  ROS timestamp
            include_orientation:  Whether to include orientation
            quaternion: (x, y, z, w) quaternion for orientation
        
        Returns:
            Imu message
        """
        msg = Imu()
        msg.header.stamp = timestamp. to_msg()
        msg.header.frame_id = self. frame_id
        
        # Orientation
        if include_orientation and quaternion is not None:
            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]
            msg.orientation_covariance = self.orientation_cov
        else:
            # Set first element to -1 to indicate orientation is not available
            msg.orientation_covariance = [-1.0] + [0.0] * 8
        
        # Angular velocity (rad/s)
        msg.angular_velocity. x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg. angular_velocity.z = gyro_z
        msg.angular_velocity_covariance = self. angular_vel_cov
        
        # Linear acceleration (m/s^2)
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration. z = accel_z
        msg.linear_acceleration_covariance = self.linear_accel_cov
        
        return msg
    
    def calibrate_callback(self, request, response):
        """Service callback to trigger calibration"""
        self.get_logger().info('Calibration requested via service')
        
        if not self.connected or self.imu is None:
            response.success = False
            response. message = 'IMU not connected'
            return response
        
        try:
            with self. lock:
                self._perform_calibration()
            
            # Reset complementary filter
            self.comp_filter.reset()
            
            response.success = True
            response. message = 'Calibration completed successfully'
            
        except Exception as e:
            response.success = False
            response.message = f'Calibration failed: {e}'
        
        return response
    
    def reset_orientation_callback(self, request, response):
        """Service callback to reset orientation estimate"""
        self.get_logger().info('Orientation reset requested')
        
        try:
            self.comp_filter.reset()
            
            response.success = True
            response.message = 'Orientation reset successfully'
            
        except Exception as e:
            response.success = False
            response.message = f'Reset failed: {e}'
        
        return response
    
    def diagnostics_callback(self):
        """Timer callback to publish diagnostics"""
        if not self.publish_diagnostics:
            return
        
        # Calculate actual publish rate
        current_time = time.time()
        elapsed = current_time - self.last_rate_check_time
        
        if elapsed > 0: 
            self.measured_rate = self.sample_count / elapsed
        
        self.sample_count = 0
        self.last_rate_check_time = current_time
        
        # Create diagnostic message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'IMU/MPU6050'
        status. hardware_id = f'i2c_{self.i2c_bus}_0x{self.i2c_address:02X}'
        
        # Determine overall status
        if not self.connected:
            status. level = DiagnosticStatus. ERROR
            status.message = 'IMU disconnected'
        elif self.last_temp > self.temp_warning:
            status.level = DiagnosticStatus. WARN
            status.message = f'High temperature: {self.last_temp:.1f}°C'
        elif self.measured_rate < self. publish_rate * self.rate_warning:
            status.level = DiagnosticStatus.WARN
            status. message = f'Low data rate: {self.measured_rate:.1f} Hz'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        
        # Add key-value pairs
        status.values = [
            KeyValue(key='connected', value=str(self.connected)),
            KeyValue(key='temperature_c', value=f'{self.last_temp:.1f}'),
            KeyValue(key='data_rate_hz', value=f'{self. measured_rate:.1f}'),
            KeyValue(key='target_rate_hz', value=f'{self.publish_rate:.1f}'),
            KeyValue(key='i2c_bus', value=str(self.i2c_bus)),
            KeyValue(key='i2c_address', value=f'0x{self.i2c_address:02X}'),
            KeyValue(key='accel_range_g', value=str(self.accel_range)),
            KeyValue(key='gyro_range_dps', value=str(self.gyro_range)),
        ]
        
        diag_msg.status. append(status)
        self.diag_pub.publish(diag_msg)
    
    def destroy_node(self):
        """Clean up on node shutdown"""
        self.get_logger().info('Shutting down IMU node...')
        
        if self.imu is not None:
            self.imu.disconnect()
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = IMUNode()
    
    try:
        rclpy. spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
