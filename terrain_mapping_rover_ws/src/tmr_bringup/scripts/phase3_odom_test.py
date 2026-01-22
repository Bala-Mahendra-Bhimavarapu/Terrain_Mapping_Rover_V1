#!/usr/bin/env python3
"""
Phase 3 Odometry Test

Compares raw and filtered odometry during robot motion.

Usage:
    ros2 run tmr_bringup phase3_odom_test. py
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import time
import math
from typing import Optional, List
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class OdomTestResult:
    name: str
    status: TestStatus
    raw_value: str
    filtered_value: str
    message: str


class Phase3OdomTest(Node):
    """Odometry comparison during motion."""
    
    def __init__(self):
        super().__init__('phase3_odom_test')
        
        # Parameters
        self.declare_parameter('test_velocity', 0.1)
        self.declare_parameter('test_duration', 2.0)
        
        self.test_velocity = self.get_parameter('test_velocity').value
        self.test_duration = self. get_parameter('test_duration').value
        
        # State
        self.latest_raw: Optional[Odometry] = None
        self.latest_filtered: Optional[Odometry] = None
        
        # Start positions
        self.start_raw_x = 0.0
        self. start_raw_y = 0.0
        self.start_filtered_x = 0.0
        self.start_filtered_y = 0.0
        
        # Results
        self. results: List[OdomTestResult] = []
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.raw_sub = self.create_subscription(
            Odometry, '/vex/odom_raw', self. raw_callback, 10)
        self.filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_callback, 10)
        
        self.get_logger().info("Phase 3 Odom Test initialized")
    
    def raw_callback(self, msg: Odometry):
        self.latest_raw = msg
    
    def filtered_callback(self, msg: Odometry):
        self.latest_filtered = msg
    
    def _get_yaw(self, orientation) -> float:
        """Extract yaw from quaternion."""
        q = orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def wait_for_data(self, timeout:  float = 5.0) -> bool:
        """Wait for odometry data."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_raw and self.latest_filtered:
                return True
        return False
    
    def send_velocity(self, linear: float, angular: float, duration: float):
        """Send velocity command for duration."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub. publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def record_start_positions(self):
        """Record starting positions."""
        if self.latest_raw:
            self.start_raw_x = self.latest_raw. pose.pose.position.x
            self.start_raw_y = self.latest_raw.pose.pose.position.y
        if self.latest_filtered:
            self.start_filtered_x = self.latest_filtered.pose.pose.position.x
            self.start_filtered_y = self.latest_filtered.pose. pose.position.y
    
    def test_forward_motion(self) -> OdomTestResult:
        """Test forward motion odometry."""
        print("\n  Testing forward motion...")
        
        self.record_start_positions()
        
        # Move forward
        expected_distance = self.test_velocity * self.test_duration
        self.send_velocity(self.test_velocity, 0.0, self.test_duration)
        self.stop_robot()
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calculate distances
        raw_dist = math. sqrt(
            (self.latest_raw.pose.pose.position.x - self.start_raw_x)**2 +
            (self.latest_raw.pose.pose.position.y - self.start_raw_y)**2
        )
        
        filtered_dist = math.sqrt(
            (self.latest_filtered.pose.pose.position. x - self.start_filtered_x)**2 +
            (self.latest_filtered.pose. pose.position.y - self. start_filtered_y)**2
        )
        
        # Compare
        diff = abs(raw_dist - filtered_dist)
        
        if diff < 0.05:  # Within 5cm
            status = TestStatus.PASSED
            message = "Raw and filtered agree"
        elif diff < 0.15: 
            status = TestStatus.WARNING
            message = f"Difference: {diff*100:.1f}cm"
        else:
            status = TestStatus.FAILED
            message = f"Large difference: {diff*100:.1f}cm"
        
        return OdomTestResult(
            name="Forward Motion",
            status=status,
            raw_value=f"{raw_dist:.4f}m",
            filtered_value=f"{filtered_dist:.4f}m",
            message=f"{message} (expected ~{expected_distance:.3f}m)"
        )
    
    def test_rotation(self) -> OdomTestResult:
        """Test rotation odometry."""
        print("\n  Testing rotation...")
        
        # Record start yaw
        start_raw_yaw = self._get_yaw(self.latest_raw.pose.pose.orientation)
        start_filtered_yaw = self._get_yaw(self.latest_filtered.pose.pose.orientation)
        
        # Rotate
        angular_vel = 0.3
        self.send_velocity(0.0, angular_vel, self.test_duration)
        self.stop_robot()
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calculate rotations
        end_raw_yaw = self._get_yaw(self.latest_raw.pose.pose.orientation)
        end_filtered_yaw = self._get_yaw(self.latest_filtered.pose.pose.orientation)
        
        raw_rotation = end_raw_yaw - start_raw_yaw
        filtered_rotation = end_filtered_yaw - start_filtered_yaw
        
        # Normalize
        while raw_rotation > math.pi: raw_rotation -= 2*math.pi
        while raw_rotation < -math.pi: raw_rotation += 2*math.pi
        while filtered_rotation > math. pi: filtered_rotation -= 2*math.pi
        while filtered_rotation < -math.pi: filtered_rotation += 2*math.pi
        
        diff = abs(raw_rotation - filtered_rotation)
        expected = angular_vel * self.test_duration
        
        if diff < 0.1:  # Within ~6 degrees
            status = TestStatus.PASSED
            message = "Rotations agree"
        elif diff < 0.3:
            status = TestStatus. WARNING
            message = f"Difference: {math.degrees(diff):.1f}°"
        else:
            status = TestStatus.FAILED
            message = f"Large difference: {math.degrees(diff):.1f}°"
        
        return OdomTestResult(
            name="Rotation",
            status=status,
            raw_value=f"{math.degrees(raw_rotation):.1f}°",
            filtered_value=f"{math.degrees(filtered_rotation):.1f}°",
            message=f"{message} (expected ~{math.degrees(expected):.1f}°)"
        )
    
    def run_tests(self) -> bool:
        """Run odometry tests."""
        print("\n" + "="*60)
        print("  PHASE 3 ODOMETRY TEST")
        print("="*60)
        print("\n⚠️  WARNING: Robot will move during this test!")
        print("   Ensure clear space around robot.")
        
        # Wait for data
        print("\nWaiting for odometry data...")
        if not self.wait_for_data():
            print("❌ No odometry data received!")
            return False
        
        time.sleep(2)  # Give user time to prepare
        
        self.results = []
        
        self.results.append(self.test_forward_motion())
        time.sleep(1)
        
        self.results.append(self.test_rotation())
        
        self.stop_robot()
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print results."""
        print("\n" + "="*60)
        print("  ODOMETRY TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            icon = result.status.value
            print(f"{icon} {result.name}")
            print(f"   Raw:       {result.raw_value}")
            print(f"   Filtered: {result.filtered_value}")
            print(f"   {result.message}")
            print()
            
            if result.status == TestStatus.PASSED: 
                passed += 1
            elif result.status == TestStatus. FAILED:
                failed += 1
            else:
                warnings += 1
        
        print("-"*60)
        print(f"Summary: {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ ODOMETRY TEST PASSED\n")
            return True
        else:
            print("\n❌ ODOMETRY TEST FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase3OdomTest()
    
    try:
        success = node.run_tests()
    except KeyboardInterrupt:
        print("\n\nTest aborted")
        node.stop_robot()
        success = False
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
