#!/usr/bin/env python3
"""
Phase 1 Motor Test

Tests motor control and encoder feedback.

Tests:
1. Forward motion
2. Backward motion
3. Turn left
4. Turn right
5. Encoder response verification
6. Emergency stop
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import time
import math
from typing import Optional
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PENDING = "⏳"
    RUNNING = "🔄"
    PASSED = "✅"
    FAILED = "❌"


@dataclass
class MotorTestResult:
    name: str
    status: TestStatus
    expected:  str
    actual: str
    message: str


class Phase1MotorTest(Node):
    """Phase 1 motor test node."""
    
    def __init__(self):
        super().__init__('phase1_motor_test')
        
        # Parameters
        self.declare_parameter('test_velocity', 0.1)
        self.declare_parameter('test_duration', 2.0)
        self.declare_parameter('settle_time', 1.0)
        
        self.test_velocity = self.get_parameter('test_velocity').value
        self.test_duration = self. get_parameter('test_duration').value
        self.settle_time = self.get_parameter('settle_time').value
        
        # State
        self.current_odom:  Optional[Odometry] = None
        self.vex_connected = False
        self.odom_received = False
        
        # Results
        self.results = []
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self. odom_callback, 10)
        self.connected_sub = self.create_subscription(
            Bool, '/vex/connected', self.connected_callback, 10)
        
        # Service client
        self.estop_client = self.create_client(Trigger, '/vex/emergency_stop')
        
        self.get_logger().info("Phase 1 Motor Test initialized")
    
    def odom_callback(self, msg: Odometry):
        """Odometry callback."""
        self.current_odom = msg
        self.odom_received = True
    
    def connected_callback(self, msg: Bool):
        """Connection status callback."""
        self.vex_connected = msg.data
    
    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        """Wait for VEX connection."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.vex_connected and self.odom_received:
                return True
        return False
    
    def send_velocity(self, linear: float, angular: float, duration: float):
        """Send velocity command for specified duration."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = time.time()
        rate = 20  # Hz
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=1.0/rate)
    
    def stop_robot(self):
        """Send stop command."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        self.cmd_vel_pub.publish(twist)
    
    def get_position(self) -> tuple:
        """Get current position from odometry."""
        if self.current_odom is None:
            return (0.0, 0.0, 0.0)
        
        x = self.current_odom. pose.pose.position.x
        y = self.current_odom.pose.pose.position. y
        
        # Extract yaw from quaternion
        q = self.current_odom. pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (x, y, yaw)
    
    def test_forward(self) -> MotorTestResult: 
        """Test forward motion."""
        print("  Testing forward motion...")
        
        # Record start position
        start_x, start_y, _ = self.get_position()
        
        # Send forward command
        self.send_velocity(self.test_velocity, 0.0, self.test_duration)
        self.stop_robot()
        
        # Wait for settle
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Record end position
        end_x, end_y, _ = self. get_position()
        
        # Calculate distance
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        expected_distance = self.test_velocity * self.test_duration
        
        # Check result
        tolerance = 0.1  # 10cm tolerance
        passed = abs(distance - expected_distance) < tolerance and distance > 0.01
        
        return MotorTestResult(
            name="Forward Motion",
            status=TestStatus. PASSED if passed else TestStatus.FAILED,
            expected=f"{expected_distance:.3f} m",
            actual=f"{distance:.3f} m",
            message="Robot moved forward" if passed else "Forward motion failed"
        )
    
    def test_backward(self) -> MotorTestResult: 
        """Test backward motion."""
        print("  Testing backward motion...")
        
        start_x, start_y, _ = self.get_position()
        
        self.send_velocity(-self.test_velocity, 0.0, self.test_duration)
        self.stop_robot()
        
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        end_x, end_y, _ = self.get_position()
        
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        expected_distance = self.test_velocity * self.test_duration
        
        tolerance = 0.1
        passed = abs(distance - expected_distance) < tolerance and distance > 0.01
        
        return MotorTestResult(
            name="Backward Motion",
            status=TestStatus.PASSED if passed else TestStatus.FAILED,
            expected=f"{expected_distance:.3f} m",
            actual=f"{distance:. 3f} m",
            message="Robot moved backward" if passed else "Backward motion failed"
        )
    
    def test_turn_left(self) -> MotorTestResult:
        """Test left turn."""
        print("  Testing left turn...")
        
        _, _, start_yaw = self.get_position()
        
        angular_vel = 0.3  # rad/s
        self.send_velocity(0.0, angular_vel, self.test_duration)
        self.stop_robot()
        
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        _, _, end_yaw = self.get_position()
        
        # Calculate angle change (handle wraparound)
        angle_change = end_yaw - start_yaw
        if angle_change > math.pi:
            angle_change -= 2 * math.pi
        elif angle_change < -math.pi:
            angle_change += 2 * math.pi
        
        expected_angle = angular_vel * self.test_duration
        
        tolerance = 0.3  # ~17 degrees
        passed = abs(angle_change - expected_angle) < tolerance and angle_change > 0.05
        
        return MotorTestResult(
            name="Turn Left",
            status=TestStatus.PASSED if passed else TestStatus.FAILED,
            expected=f"{math.degrees(expected_angle):.1f}°",
            actual=f"{math.degrees(angle_change):.1f}°",
            message="Robot turned left" if passed else "Left turn failed"
        )
    
    def test_turn_right(self) -> MotorTestResult:
        """Test right turn."""
        print("  Testing right turn...")
        
        _, _, start_yaw = self.get_position()
        
        angular_vel = -0.3  # rad/s (negative for right)
        self.send_velocity(0.0, angular_vel, self.test_duration)
        self.stop_robot()
        
        time.sleep(self. settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        _, _, end_yaw = self.get_position()
        
        angle_change = end_yaw - start_yaw
        if angle_change > math.pi:
            angle_change -= 2 * math.pi
        elif angle_change < -math.pi:
            angle_change += 2 * math.pi
        
        expected_angle = angular_vel * self.test_duration
        
        tolerance = 0.3
        passed = abs(angle_change - expected_angle) < tolerance and angle_change < -0.05
        
        return MotorTestResult(
            name="Turn Right",
            status=TestStatus.PASSED if passed else TestStatus.FAILED,
            expected=f"{math. degrees(expected_angle):.1f}°",
            actual=f"{math.degrees(angle_change):.1f}°",
            message="Robot turned right" if passed else "Right turn failed"
        )
    
    def test_emergency_stop(self) -> MotorTestResult:
        """Test emergency stop."""
        print("  Testing emergency stop...")
        
        # Start moving
        self.send_velocity(self.test_velocity, 0.0, 0.5)
        
        # Record position
        start_x, start_y, _ = self.get_position()
        
        # Trigger emergency stop
        if self.estop_client.wait_for_service(timeout_sec=1.0):
            future = self.estop_client. call_async(Trigger. Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        # Wait and check
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        end_x, end_y, _ = self.get_position()
        distance_after_stop = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        
        # Should stop quickly (< 5cm after e-stop)
        passed = distance_after_stop < 0.05
        
        return MotorTestResult(
            name="Emergency Stop",
            status=TestStatus.PASSED if passed else TestStatus.FAILED,
            expected="< 0.05 m after stop",
            actual=f"{distance_after_stop:.3f} m",
            message="E-stop working" if passed else "E-stop may be slow"
        )
    
    def run_all_tests(self):
        """Run all motor tests."""
        print("\n" + "="*60)
        print("  PHASE 1 MOTOR TEST")
        print("="*60 + "\n")
        
        # Wait for connection
        print("Waiting for VEX connection...")
        if not self.wait_for_connection():
            print("❌ Failed to connect to VEX!")
            return False
        print("✅ Connected to VEX\n")
        
        # Safety warning
        print("⚠️  WARNING: Robot will move during this test!")
        print("   Ensure robot is on the ground with clear space around it.")
        print("   Press Ctrl+C to abort.\n")
        time.sleep(3)
        
        # Run tests
        self.results = []
        
        self.results.append(self.test_forward())
        time.sleep(1)
        
        self.results.append(self.test_backward())
        time.sleep(1)
        
        self.results.append(self.test_turn_left())
        time.sleep(1)
        
        self.results.append(self.test_turn_right())
        time.sleep(1)
        
        self.results.append(self.test_emergency_stop())
        
        # Ensure stopped
        self.stop_robot()
        
        return True
    
    def print_results(self):
        """Print test results."""
        print("\n" + "="*60)
        print("  MOTOR TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        
        for result in self.results:
            status_icon = result.status.value
            print(f"{status_icon} {result.name}")
            print(f"   Expected: {result.expected}")
            print(f"   Actual:   {result.actual}")
            print(f"   {result.message}\n")
            
            if result.status == TestStatus.PASSED:
                passed += 1
            else:
                failed += 1
        
        print("-"*60)
        print(f"Summary: {passed} passed, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ ALL MOTOR TESTS PASSED\n")
            return True
        else:
            print("\n❌ SOME MOTOR TESTS FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase1MotorTest()
    
    try:
        if node.run_all_tests():
            success = node.print_results()
        else:
            success = False
            
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
