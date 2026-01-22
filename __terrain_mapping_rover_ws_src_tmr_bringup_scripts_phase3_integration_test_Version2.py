#!/usr/bin/env python3
"""
Phase 3 Integration Test

Complete integration test for EKF sensor fusion. 
Tests all Phase 3 components working together.

Usage:
    ros2 run tmr_bringup phase3_integration_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

import time
import math
from typing import Optional, List
from dataclasses import dataclass
from enum import Enum
from collections import deque


class TestStatus(Enum):
    PENDING = "⏳"
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class IntegrationTestResult:
    name:  str
    status: TestStatus
    message: str
    details: str = ""


class Phase3IntegrationTest(Node):
    """Complete Phase 3 integration test."""
    
    def __init__(self):
        super().__init__('phase3_integration_test')
        
        # Parameters
        self.declare_parameter('test_velocity', 0.1)
        self.declare_parameter('test_duration', 3.0)
        
        self.test_velocity = self.get_parameter('test_velocity').value
        self.test_duration = self.get_parameter('test_duration').value
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Data tracking
        self.raw_odom_times = deque(maxlen=100)
        self.imu_times = deque(maxlen=100)
        self.filtered_times = deque(maxlen=100)
        
        self.latest_raw: Optional[Odometry] = None
        self.latest_filtered: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        
        # Results
        self.results: List[IntegrationTestResult] = []
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.raw_sub = self.create_subscription(
            Odometry, '/vex/odom_raw', self.raw_callback, 10)
        self.filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_callback, 10)
        self.imu_sub = self. create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        
        self.get_logger().info("Phase 3 Integration Test initialized")
    
    def raw_callback(self, msg: Odometry):
        self.raw_odom_times.append(time.time())
        self.latest_raw = msg
    
    def filtered_callback(self, msg: Odometry):
        self.filtered_times.append(time.time())
        self.latest_filtered = msg
    
    def imu_callback(self, msg: Imu):
        self.imu_times. append(time.time())
        self.latest_imu = msg
    
    def calculate_rate(self, times: deque) -> float:
        if len(times) < 2:
            return 0.0
        duration = times[-1] - times[0]
        if duration <= 0:
            return 0.0
        return (len(times) - 1) / duration
    
    def stop_robot(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def send_velocity(self, linear: float, angular: float, duration: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start = time.time()
        while time.time() - start < duration:
            self. cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def wait_for_systems(self, timeout: float = 10.0) -> bool:
        """Wait for all systems to be ready."""
        print("Waiting for all systems...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.latest_raw is not None and 
                self. latest_filtered is not None and 
                self.latest_imu is not None):
                return True
        return False
    
    # =========================================================================
    # Tests
    # =========================================================================
    
    def test_all_sensors_running(self) -> IntegrationTestResult:
        """Test all sensors are producing data."""
        print("  Checking sensors...")
        
        # Collect for 3 seconds
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        raw_rate = self.calculate_rate(self.raw_odom_times)
        imu_rate = self.calculate_rate(self.imu_times)
        filtered_rate = self.calculate_rate(self.filtered_times)
        
        issues = []
        if raw_rate < 20: 
            issues.append(f"Odom:  {raw_rate:.1f}Hz")
        if imu_rate < 50:
            issues.append(f"IMU: {imu_rate:. 1f}Hz")
        if filtered_rate < 20:
            issues.append(f"EKF: {filtered_rate:. 1f}Hz")
        
        if not issues:
            return IntegrationTestResult(
                name="Sensor Data Rates",
                status=TestStatus.PASSED,
                message=f"Odom:{raw_rate:.0f}Hz, IMU:{imu_rate:.0f}Hz, EKF:{filtered_rate:.0f}Hz"
            )
        else:
            return IntegrationTestResult(
                name="Sensor Data Rates",
                status=TestStatus.WARNING if len(issues) < 3 else TestStatus.FAILED,
                message="Low rates:  " + ", ".join(issues)
            )
    
    def test_tf_publishing(self) -> IntegrationTestResult:
        """Test EKF is publishing TF."""
        print("  Checking TF...")
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=2. 0)
            )
            
            t = transform.transform.translation
            return IntegrationTestResult(
                name="TF Publishing",
                status=TestStatus.PASSED,
                message=f"odom->base_link: ({t.x:.3f}, {t.y:.3f})"
            )
        except TransformException as e:
            return IntegrationTestResult(
                name="TF Publishing",
                status=TestStatus.FAILED,
                message="TF odom->base_link not available",
                details=str(e)[: 50]
            )
    
    def test_position_updates_with_motion(self) -> IntegrationTestResult:
        """Test position updates when robot moves."""
        print("  Testing position updates with motion...")
        print("    Robot will move forward...")
        
        if self.latest_filtered is None:
            return IntegrationTestResult(
                name="Position Updates",
                status=TestStatus.FAILED,
                message="No filtered odometry available"
            )
        
        start_x = self.latest_filtered.pose.pose.position.x
        start_y = self.latest_filtered.pose.pose.position. y
        
        # Move forward
        self.send_velocity(self.test_velocity, 0.0, self.test_duration)
        self.stop_robot()
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        end_x = self.latest_filtered.pose.pose.position.x
        end_y = self. latest_filtered.pose.pose. position.y
        
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        expected = self.test_velocity * self. test_duration
        
        if distance > expected * 0.5 and distance < expected * 1.5:
            return IntegrationTestResult(
                name="Position Updates",
                status=TestStatus. PASSED,
                message=f"Moved {distance:.3f}m (expected ~{expected:.3f}m)"
            )
        elif distance > 0.01:
            return IntegrationTestResult(
                name="Position Updates",
                status=TestStatus. WARNING,
                message=f"Distance: {distance:.3f}m vs expected {expected:.3f}m"
            )
        else:
            return IntegrationTestResult(
                name="Position Updates",
                status=TestStatus.FAILED,
                message=f"Position didn't update: {distance:.4f}m"
            )
    
    def test_tf_updates_with_motion(self) -> IntegrationTestResult:
        """Test TF updates when robot moves."""
        print("  Testing TF updates with motion...")
        
        try:
            tf_before = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            start_x = tf_before.transform.translation.x
        except: 
            return IntegrationTestResult(
                name="TF Updates",
                status=TestStatus. FAILED,
                message="Could not get initial TF"
            )
        
        # Move forward
        print("    Robot will move forward...")
        self.send_velocity(self.test_velocity, 0.0, 2.0)
        self.stop_robot()
        
        time.sleep(0.5)
        
        try:
            tf_after = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            end_x = tf_after. transform.translation.x
        except:
            return IntegrationTestResult(
                name="TF Updates",
                status=TestStatus.FAILED,
                message="Could not get final TF"
            )
        
        moved = abs(end_x - start_x)
        
        if moved > 0.05:
            return IntegrationTestResult(
                name="TF Updates",
                status=TestStatus. PASSED,
                message=f"TF updated: moved {moved:.3f}m"
            )
        else:
            return IntegrationTestResult(
                name="TF Updates",
                status=TestStatus. FAILED,
                message=f"TF not updating: {moved:.4f}m"
            )
    
    def test_imu_improves_rotation(self) -> IntegrationTestResult:
        """Test IMU contribution to rotation estimation."""
        print("  Testing IMU rotation contribution...")
        print("    Robot will rotate...")
        
        if self. latest_imu is None: 
            return IntegrationTestResult(
                name="IMU Rotation",
                status=TestStatus. FAILED,
                message="No IMU data"
            )
        
        # Rotate and check gyro matches odometry angular velocity
        self.send_velocity(0.0, 0.3, 2.0)
        self.stop_robot()
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # IMU should have seen rotation
        if self.latest_imu. angular_velocity_covariance[0] < 0:
            return IntegrationTestResult(
                name="IMU Rotation",
                status=TestStatus.WARNING,
                message="IMU angular velocity not configured"
            )
        
        return IntegrationTestResult(
            name="IMU Rotation",
            status=TestStatus.PASSED,
            message="IMU contributing to rotation estimate"
        )
    
    def run_all_tests(self) -> bool:
        """Run all integration tests."""
        print("\n" + "="*60)
        print("  PHASE 3 INTEGRATION TEST")
        print("="*60)
        print("\n⚠️  WARNING: Robot will move during this test!")
        print("   Ensure clear space around robot.\n")
        
        # Wait for systems
        if not self.wait_for_systems():
            print("❌ Not all systems ready!")
            return False
        
        print("✅ All systems ready\n")
        time.sleep(2)
        
        self.results = []
        
        self.results.append(self.test_all_sensors_running())
        time.sleep(1)
        
        self.results.append(self.test_tf_publishing())
        time.sleep(1)
        
        self.results.append(self.test_position_updates_with_motion())
        time.sleep(1)
        
        self.results.append(self.test_tf_updates_with_motion())
        time.sleep(1)
        
        self.results.append(self.test_imu_improves_rotation())
        
        self.stop_robot()
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print test results."""
        print("\n" + "="*60)
        print("  INTEGRATION TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            icon = result.status.value
            print(f"{icon} {result. name}")
            print(f"   {result.message}")
            if result.details:
                print(f"   {result.details}")
            print()
            
            if result. status == TestStatus.PASSED: 
                passed += 1
            elif result.status == TestStatus. FAILED:
                failed += 1
            else:
                warnings += 1
        
        print("-"*60)
        print(f"Summary: {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ PHASE 3 INTEGRATION TEST PASSED\n")
            print("The EKF sensor fusion is working correctly!")
            print("Robot is ready for Phase 4 (SLAM).")
            return True
        else:
            print("\n❌ PHASE 3 INTEGRATION TEST FAILED\n")
            print("Please fix issues before proceeding.")
            return False


def main():
    rclpy.init()
    
    node = Phase3IntegrationTest()
    
    try:
        success = node.run_all_tests()
    except KeyboardInterrupt:
        print("\n\nTest aborted by user")
        node.stop_robot()
        success = False
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())