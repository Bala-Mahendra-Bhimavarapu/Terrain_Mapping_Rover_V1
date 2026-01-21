#!/usr/bin/env python3
"""
Phase 1 Calibration Script

Performs calibration for all Phase 1 sensors: 
1. IMU calibration (gyro and accelerometer offsets)
2. Odometry calibration (wheel radius and track width)
3. Camera calibration guidance
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import time
import math
import yaml
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class CalibrationResult:
    name: str
    success: bool
    values: dict
    message: str


class Phase1Calibration(Node):
    """Phase 1 calibration node."""
    
    def __init__(self):
        super().__init__('phase1_calibration')
        
        # IMU data
        self.imu_data:  List[Imu] = []
        
        # Odometry data
        self.current_odom: Optional[Odometry] = None
        
        # Results
        self.results: List[CalibrationResult] = []
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=100
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, sensor_qos)
        self.odom_sub = self. create_subscription(
            Odometry, '/odom', self. odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # IMU calibration service
        self. imu_calibrate_client = self.create_client(Trigger, '/imu/calibrate')
        
        self.get_logger().info("Phase 1 Calibration initialized")
    
    def imu_callback(self, msg: Imu):
        """IMU callback."""
        self.imu_data.append(msg)
    
    def odom_callback(self, msg: Odometry):
        """Odometry callback."""
        self.current_odom = msg
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def calibrate_imu(self, samples: int = 500) -> CalibrationResult:
        """
        Calibrate IMU by computing bias offsets. 
        Robot must be stationary! 
        """
        print("\n" + "="*50)
        print("  IMU CALIBRATION")
        print("="*50)
        print("\n⚠️  IMPORTANT: Robot must be STATIONARY and LEVEL!")
        print("   Do not move the robot during calibration.\n")
        
        input("Press Enter when ready to start IMU calibration...")
        
        # Try service-based calibration first
        if self.imu_calibrate_client.wait_for_service(timeout_sec=2.0):
            print("Using IMU driver calibration service...")
            future = self.imu_calibrate_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() and future.result().success:
                return CalibrationResult(
                    name="IMU Calibration",
                    success=True,
                    values={},
                    message="Calibration completed via service"
                )
        
        # Manual calibration
        print(f"Collecting {samples} IMU samples...")
        
        self.imu_data.clear()
        
        start_time = time.time()
        while len(self.imu_data) < samples and time.time() - start_time < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            progress = len(self.imu_data) / samples * 100
            print(f"\rProgress: {progress:.1f}%", end='', flush=True)
        
        print("\n")
        
        if len(self.imu_data) < samples * 0.9:
            return CalibrationResult(
                name="IMU Calibration",
                success=False,
                values={},
                message=f"Insufficient data: {len(self.imu_data)}/{samples} samples"
            )
        
        # Calculate offsets
        accel_x_sum = sum(d.linear_acceleration.x for d in self. imu_data)
        accel_y_sum = sum(d.linear_acceleration.y for d in self.imu_data)
        accel_z_sum = sum(d.linear_acceleration.z for d in self. imu_data)
        
        gyro_x_sum = sum(d.angular_velocity. x for d in self.imu_data)
        gyro_y_sum = sum(d. angular_velocity.y for d in self.imu_data)
        gyro_z_sum = sum(d.angular_velocity. z for d in self.imu_data)
        
        n = len(self.imu_data)
        
        # Accelerometer:  expect (0, 0, 9.81) when level
        accel_offset_x = accel_x_sum / n
        accel_offset_y = accel_y_sum / n
        accel_offset_z = accel_z_sum / n - 9.80665
        
        # Gyroscope: expect (0, 0, 0) when stationary
        gyro_offset_x = gyro_x_sum / n
        gyro_offset_y = gyro_y_sum / n
        gyro_offset_z = gyro_z_sum / n
        
        values = {
            'accel_offset_x': accel_offset_x,
            'accel_offset_y': accel_offset_y,
            'accel_offset_z': accel_offset_z,
            'gyro_offset_x': gyro_offset_x,
            'gyro_offset_y':  gyro_offset_y,
            'gyro_offset_z': gyro_offset_z,
        }
        
        print("IMU Calibration Results:")
        print(f"  Accelerometer Offsets (m/s²):")
        print(f"    X: {accel_offset_x: +.6f}")
        print(f"    Y: {accel_offset_y:+.6f}")
        print(f"    Z: {accel_offset_z:+.6f}")
        print(f"  Gyroscope Offsets (rad/s):")
        print(f"    X: {gyro_offset_x:+.6f}")
        print(f"    Y: {gyro_offset_y:+.6f}")
        print(f"    Z: {gyro_offset_z:+.6f}")
        
        return CalibrationResult(
            name="IMU Calibration",
            success=True,
            values=values,
            message="Calibration completed successfully"
        )
    
    def calibrate_odometry(self) -> CalibrationResult:
        """
        Guide user through odometry calibration.
        Tests wheel radius and track width.
        """
        print("\n" + "="*50)
        print("  ODOMETRY CALIBRATION")
        print("="*50)
        
        print("\nThis will help verify odometry parameters.")
        print("You will need a tape measure.\n")
        
        results = {}
        
        # Test 1: Linear distance
        print("TEST 1: Linear Distance")
        print("-" * 30)
        print("The robot will drive forward.  Measure the actual distance traveled.")
        input("Press Enter when ready...")
        
        # Get start position
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.current_odom is None:
            return CalibrationResult(
                name="Odometry Calibration",
                success=False,
                values={},
                message="No odometry data available"
            )
        
        start_x = self.current_odom. pose.pose.position.x
        start_y = self.current_odom.pose.pose.position.y
        
        # Drive forward
        print("Driving forward for 3 seconds...")
        twist = Twist()
        twist.linear.x = 0.1  # 0.1 m/s
        
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop_robot()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # Get end position
        end_x = self.current_odom.pose. pose.position.x
        end_y = self.current_odom.pose.pose.position. y
        
        odom_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        
        print(f"\nOdometry reported distance: {odom_distance:. 4f} m")
        
        try:
            actual_distance = float(input("Enter actual measured distance (m): "))
            
            if actual_distance > 0:
                linear_scale = actual_distance / odom_distance
                results['linear_scale'] = linear_scale
                print(f"Linear scale factor: {linear_scale:.4f}")
                
                # Calculate corrected wheel radius
                current_radius = 0.0508  # Default 4" wheels
                corrected_radius = current_radius * linear_scale
                results['corrected_wheel_radius'] = corrected_radius
                print(f"Corrected wheel radius: {corrected_radius:.5f} m")
        except ValueError:
            print("Invalid input, skipping linear calibration")
        
        # Test 2: Angular (rotation)
        print("\n\nTEST 2: Rotation")
        print("-" * 30)
        print("The robot will rotate in place. Count the number of full rotations.")
        input("Press Enter when ready...")
        
        # Get start angle
        rclpy.spin_once(self, timeout_sec=0.5)
        q = self.current_odom. pose.pose.orientation
        start_yaw = math. atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Rotate
        print("Rotating for 10 seconds (should be ~1-2 rotations)...")
        twist = Twist()
        twist.angular.z = 0.5  # 0.5 rad/s
        
        total_rotation = 0.0
        last_yaw = start_yaw
        
        start_time = time. time()
        while time.time() - start_time < 10.0:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if self.current_odom:
                q = self.current_odom.pose.pose.orientation
                current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                delta = current_yaw - last_yaw
                if delta > math.pi:
                    delta -= 2 * math.pi
                elif delta < -math.pi:
                    delta += 2 * math.pi
                
                total_rotation += delta
                last_yaw = current_yaw
        
        self.stop_robot()
        
        odom_rotations = total_rotation / (2 * math.pi)
        print(f"\nOdometry reported rotations: {odom_rotations:.3f}")
        
        try:
            actual_rotations = float(input("Enter actual number of rotations: "))
            
            if actual_rotations > 0:
                angular_scale = actual_rotations / odom_rotations
                results['angular_scale'] = angular_scale
                print(f"Angular scale factor: {angular_scale:.4f}")
                
                # Calculate corrected track width
                current_track = 0.295  # Default track width
                corrected_track = current_track / angular_scale
                results['corrected_track_width'] = corrected_track
                print(f"Corrected track width: {corrected_track:.5f} m")
        except ValueError: 
            print("Invalid input, skipping angular calibration")
        
        return CalibrationResult(
            name="Odometry Calibration",
            success=len(results) > 0,
            values=results,
            message="Calibration values computed" if results else "No calibration performed"
        )
    
    def save_calibration(self, filename:  str = "phase1_calibration. yaml"):
        """Save calibration results to file."""
        data = {
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'results': {}
        }
        
        for result in self.results:
            data['results'][result.name] = {
                'success': result.success,
                'values': result. values,
                'message': result.message
            }
        
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        print(f"\nCalibration saved to:  {filename}")
    
    def run_calibration(self):
        """Run all calibrations."""
        print("\n" + "="*60)
        print("  PHASE 1 CALIBRATION")
        print("="*60)
        
        print("\nThis script will guide you through sensor calibration.")
        print("Follow the prompts carefully.\n")
        
        self.results = []
        
        # IMU calibration
        imu_result = self.calibrate_imu()
        self.results.append(imu_result)
        
        # Odometry calibration
        odom_result = self.calibrate_odometry()
        self.results.append(odom_result)
        
        # Print summary
        print("\n" + "="*60)
        print("  CALIBRATION SUMMARY")
        print("="*60 + "\n")
        
        for result in self.results:
            status = "✅" if result.success else "❌"
            print(f"{status} {result.name}:  {result.message}")
            
            if result.values:
                for key, value in result.values. items():
                    if isinstance(value, float):
                        print(f"   {key}: {value:.6f}")
                    else:
                        print(f"   {key}:  {value}")
        
        # Offer to save
        save = input("\nSave calibration to file? (y/n): ").lower()
        if save == 'y':
            self. save_calibration()
        
        return all(r.success for r in self.results)


def main():
    rclpy.init()
    
    node = Phase1Calibration()
    
    try:
        success = node.run_calibration()
    except KeyboardInterrupt:
        print("\n\nCalibration aborted")
        node.stop_robot()
        success = False
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
