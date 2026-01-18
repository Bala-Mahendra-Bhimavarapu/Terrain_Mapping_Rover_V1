#!/usr/bin/env python3
"""
MPU6050 IMU ROS 2 Node

Publishes sensor_msgs/Imu messages from MPU6050.

Author: Rover Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion

import math
from imu_driver import MPU6050Driver, ComplementaryFilter


class MPU6050Node(Node):
    """ROS 2 node for MPU6050 IMU."""
    
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('update_rate', 100.0)  # Hz
        self.declare_parameter('accel_range', 2)  # ±2g
        self.declare_parameter('gyro_range', 250)  # ±250 deg/s
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('calibration_samples', 100)
        self.declare_parameter('use_complementary_filter', True)
        self.declare_parameter('filter_alpha', 0.98)
        
        # Covariance parameters (diagonal elements)
        self.declare_parameter('orientation_covariance', [0.01, 0.01, 0.01])
        self.declare_parameter('angular_velocity_covariance', [0.001, 0.001, 0.001])
        self.declare_parameter('linear_acceleration_covariance', [0.01, 0.01, 0.01])
        
        # Get parameters
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        update_rate = self.get_parameter('update_rate').value
        accel_range = self.get_parameter('accel_range').value
        gyro_range = self.get_parameter('gyro_range').value
        calibrate = self.get_parameter('calibrate_on_start').value
        cal_samples = self.get_parameter('calibration_samples').value
        use_filter = self.get_parameter('use_complementary_filter').value
        filter_alpha = self.get_parameter('filter_alpha').value
        
        # Initialize driver
        self.driver = MPU6050Driver(
            bus_number=i2c_bus,
            address=i2c_address,
            accel_range=accel_range,
            gyro_range=gyro_range
        )
        
        if not self.driver.initialize():
            self.get_logger().error('Failed to initialize MPU6050!')
            raise RuntimeError('MPU6050 initialization failed')
        
        self.get_logger().info('MPU6050 initialized successfully')
        
        # Calibrate if requested
        if calibrate:
            self.get_logger().info('Starting calibration...')
            if self.driver.calibrate(num_samples=cal_samples):
                self.get_logger().info('Calibration complete')
            else:
                self. get_logger().warn('Calibration failed, using defaults')
        
        # Initialize complementary filter
        self.use_filter = use_filter
        if use_filter:
            self.filter = ComplementaryFilter(alpha=filter_alpha)
        else:
            self.filter = None
        
        # Build covariance matrices
        self._build_covariances()
        
        # Create publisher
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.imu_pub = self.create_publisher(Imu, '/imu/data', imu_qos)
        
        # Create timer for publishing
        period = 1.0 / update_rate
        self. timer = self.create_timer(period, self._timer_callback)
        
        self.get_logger().info(f'Publishing IMU data at {update_rate} Hz on /imu/data')
    
    def _build_covariances(self):
        """Build covariance matrices from parameters."""
        orient_cov = self.get_parameter('orientation_covariance').value
        ang_vel_cov = self. get_parameter('angular_velocity_covariance').value
        lin_acc_cov = self. get_parameter('linear_acceleration_covariance').value
        
        self.orientation_covariance = self._diagonal_to_matrix(orient_cov)
        self.angular_velocity_covariance = self._diagonal_to_matrix(ang_vel_cov)
        self.linear_acceleration_covariance = self._diagonal_to_matrix(lin_acc_cov)
    
    @staticmethod
    def _diagonal_to_matrix(diagonal: list) -> list:
        """Convert 3-element diagonal to 9-element row-major covariance."""
        cov = [0.0] * 9
        cov[0] = diagonal[0]  # (0,0)
        cov[4] = diagonal[1]  # (1,1)
        cov[8] = diagonal[2]  # (2,2)
        return cov
    
    def _timer_callback(self):
        """Read IMU and publish."""
        data = self.driver.read()
        if data is None:
            return
        
        # Create IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Linear acceleration
        msg.linear_acceleration.x = data.accel_x
        msg.linear_acceleration.y = data.accel_y
        msg.linear_acceleration.z = data.accel_z
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        
        # Angular velocity
        msg.angular_velocity.x = data.gyro_x
        msg.angular_velocity.y = data.gyro_y
        msg.angular_velocity.z = data.gyro_z
        msg.angular_velocity_covariance = self.angular_velocity_covariance
        
        # Orientation (from complementary filter or set to unknown)
        if self.use_filter and self.filter:
            roll, pitch = self.filter.update(
                data.accel_x, data.accel_y, data.accel_z,
                data.gyro_x, data.gyro_y,
                data.timestamp
            )
            # Convert roll, pitch to quaternion (yaw = 0, no magnetometer)
            msg.orientation = self._euler_to_quaternion(roll, pitch, 0.0)
            msg.orientation_covariance = self.orientation_covariance
        else:
            # Orientation unknown - set covariance[0] to -1
            msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            cov = [0.0] * 9
            cov[0] = -1.0
            msg.orientation_covariance = cov
        
        self.imu_pub.publish(msg)
    
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q
    
    def destroy_node(self):
        """Clean shutdown."""
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MPU6050Node()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Failed to start MPU6050 node: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
