#!/usr/bin/env python3
"""
Phase 1 Integration Test

Tests the complete system integration: 
1. Teleop -> Motor response
2. Motor movement -> Odometry update
3. Odometry -> TF broadcast
4. All sensors during motion
5. Emergency stop functionality
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

import time
import math
from typing import Optional, List
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PENDING = "⏳"
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class IntegrationTestResult:
    name: str
    status: TestStatus
    message: str
    details: str = ""


class Phase1IntegrationTest(Node):
    """Phase 1 integration test node."""
    
    def __init__(self):
        super().__init__('phase1_integration_test')
        
        # Parameters
        self.declare_parameter('test_velocity', 0.1)
        self.declare_parameter('test_duration', 3.0)
        
        self.test_velocity = self.get_parameter('test_velocity').value
        self.test_duration = self.get_parameter('test_duration').value
        
        # State tracking
        self.current_odom:  Optional[Odometry] = None
        self.current_imu: Optional[Imu] = None
        self.vex_connected = False
        self.imu_connected = False
        
        self.odom_count = 0
        self.imu_count = 0
        self.camera_count = 0
        self.tof_count = 0
        
        # Results
        self.results: List[IntegrationTestResult] = []
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self. create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, sensor_qos)
        self.tof_sub = self.create_subscription(
            Image, '/tof/depth/image_raw', self.tof_callback, sensor_qos)
        self.vex_connected_sub = self.create_subscription(
            Bool, '/vex/connected', self.vex_connected_callback, 10)
        self.imu_connected_sub = self.create_subscription(
            Bool, '/imu/connected', self.imu_connected_callback, 10)
        
        # Service client
        self.estop_client = self.create_client(Trigger, '/vex/emergency_stop')
        self.clear_estop_client = self.create_client(Trigger, '/vex/clear_emergency_stop')
        
        self.get_logger().info("Phase 1 Integration Test initialized")
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def odom_callback(self, msg: Odometry):
        self.current_odom = msg
        self.odom_count += 1
    
    def imu_callback(self, msg: Imu):
        self.current_imu = msg
        self.imu_count += 1
    
    def camera_callback(self, msg: Image):
        self.camera_count += 1
    
    def tof_callback(self, msg: Image):
        self.tof_count += 1
    
    def vex_connected_callback(self, msg: Bool):
        self.vex_connected = msg. data
    
    def imu_connected_callback(self, msg: Bool):
        self.imu_connected = msg.data
    
    # =========================================================================
    # Helpers
    # =========================================================================
    
    def send_velocity(self, linear: float, angular: float, duration: float):
        """Send velocity command for duration."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        self.cmd_vel_pub.publish(twist)
    
    def reset_counts(self):
        """Reset message counts."""
        self.odom_count = 0
        self.imu_count = 0
        self.camera_count = 0
        self.tof_count = 0
    
    def get_odom_position(self) -> tuple: 
        """Get position from odometry."""
        if self.current_odom is None:
            return (0.0, 0.0, 0.0)
        
        x = self.current_odom. pose.pose.position.x
        y = self.current_odom.pose.pose.position. y
        
        q = self.current_odom. pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        return (x, y, yaw)
    
    def wait_for_systems(self, timeout: float = 10.0) -> bool:
        """Wait for all systems to be ready."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.vex_connected and self.current_odom is not None:
                return True
        
        return False
    
    # =========================================================================
    # Tests
    # =========================================================================
    
    def test_teleop_to_motion(self) -> IntegrationTestResult:
        """Test teleop command results in motion."""
        print("  Testing teleop -> motor response...")
        
        # Get start position
        start_x, start_y, _ = self.get_odom_position()
        start_odom_count = self.odom_count
        
        # Send velocity command
        self.send_velocity(self.test_velocity, 0.0, self.test_duration)
        self.stop_robot()
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Check motion
        end_x, end_y, _ = self.get_odom_position()
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        
        odom_messages = self.odom_count - start_odom_count
        
        expected = self.test_velocity * self.test_duration
        
        if distance > expected * 0.5 and odom_messages > 10:
            return IntegrationTestResult(
                name="Teleop → Motion",
                status=TestStatus. PASSED,
                message=f"Motion detected: {distance:.3f}m",
                details=f"Expected ~{expected:.3f}m, received {odom_messages} odom messages"
            )
        else:
            return IntegrationTestResult(
                name="Teleop → Motion",
                status=TestStatus. FAILED,
                message=f"Motion insufficient: {distance:.3f}m",
                details=f"Expected ~{expected:.3f}m"
            )
    
    def test_odometry_to_tf(self) -> IntegrationTestResult: 
        """Test odometry updates TF."""
        print("  Testing odometry -> TF...")
        
        try:
            # Try to get transform
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2. 0)
            )
            
            # Check that transform matches odometry
            if self.current_odom is not None: 
                odom_x = self.current_odom.pose.pose.position.x
                odom_y = self.current_odom. pose.pose.position.y
                
                tf_x = transform.transform.translation.x
                tf_y = transform.transform.translation.y
                
                error = math.sqrt((odom_x - tf_x)**2 + (odom_y - tf_y)**2)
                
                if error < 0.01:  # Within 1cm
                    return IntegrationTestResult(
                        name="Odometry → TF",
                        status=TestStatus.PASSED,
                        message="TF matches odometry",
                        details=f"Position: ({tf_x:.3f}, {tf_y:.3f})"
                    )
                else: 
                    return IntegrationTestResult(
                        name="Odometry → TF",
                        status=TestStatus.WARNING,
                        message=f"TF/Odom mismatch: {error:. 3f}m",
                    )
            else:
                return IntegrationTestResult(
                    name="Odometry → TF",
                    status=TestStatus.PASSED,
                    message="TF available",
                    details=f"Transform:  odom -> base_link"
                )
                
        except Exception as e: 
            return IntegrationTestResult(
                name="Odometry → TF",
                status=TestStatus.FAILED,
                message=f"TF lookup failed:  {e}",
            )
    
    def test_sensors_during_motion(self) -> IntegrationTestResult:
        """Test all sensors work during motion."""
        print("  Testing sensors during motion...")
        
        self.reset_counts()
        
        # Move while collecting sensor data
        duration = 3.0
        self.send_velocity(self.test_velocity, 0.1, duration)
        self.stop_robot()
        
        # Check counts
        issues = []
        passed = True
        
        if self.odom_count < duration * 20:  # Expect 20+ Hz
            issues.append(f"Odometry:  {self.odom_count} messages (low)")
            passed = False
        else:
            issues.append(f"Odometry: {self. odom_count} messages ✓")
        
        if self. imu_count < duration * 50:  # Expect 50+ Hz
            issues.append(f"IMU: {self.imu_count} messages (low)")
            passed = False
        else:
            issues.append(f"IMU: {self.imu_count} messages ✓")
        
        if self. camera_count < duration * 5:  # Expect 5+ Hz
            issues.append(f"Camera: {self.camera_count} messages (low)")
        else:
            issues.append(f"Camera: {self.camera_count} messages ✓")
        
        if self.tof_count < duration * 5:  # Expect 5+ Hz
            issues.append(f"ToF: {self.tof_count} messages (low)")
        else:
            issues.append(f"ToF: {self. tof_count} messages ✓")
        
        return IntegrationTestResult(
            name="Sensors During Motion",
            status=TestStatus.PASSED if passed else TestStatus.WARNING,
            message="All critical sensors operational" if passed else "Some sensors underperforming",
            details="\n".join(issues)
        )
    
    def test_imu_motion_detection(self) -> IntegrationTestResult:
        """Test IMU detects motion."""
        print("  Testing IMU motion detection...")
        
        if self.current_imu is None:
            return IntegrationTestResult(
                name="IMU Motion Detection",
                status=TestStatus.FAILED,
                message="No IMU data available"
            )
        
        # Record gyro while stationary
        stationary_samples = []
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_imu:
                gyro_z = self.current_imu. angular_velocity.z
                stationary_samples.append(abs(gyro_z))
        
        stationary_gyro = sum(stationary_samples) / len(stationary_samples) if stationary_samples else 0
        
        # Now rotate and check gyro
        motion_samples = []
        start_time = time.time()
        
        while time.time() - start_time < 2.0:
            self.cmd_vel_pub.publish(Twist(angular=Twist().angular))  # Create twist with angular
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if self.current_imu:
                gyro_z = self.current_imu.angular_velocity.z
                motion_samples.append(abs(gyro_z))
        
        self.stop_robot()
        
        motion_gyro = sum(motion_samples) / len(motion_samples) if motion_samples else 0
        
        if motion_gyro > stationary_gyro + 0.1:
            return IntegrationTestResult(
                name="IMU Motion Detection",
                status=TestStatus.PASSED,
                message="IMU detects rotation",
                details=f"Stationary:  {stationary_gyro:.3f} rad/s, Motion: {motion_gyro:.3f} rad/s"
            )
        else:
            return IntegrationTestResult(
                name="IMU Motion Detection",
                status=TestStatus.WARNING,
                message="IMU may not be detecting motion properly",
                details=f"Stationary: {stationary_gyro:.3f}, Motion: {motion_gyro:. 3f}"
            )
    
    def test_emergency_stop_integration(self) -> IntegrationTestResult:
        """Test e-stop integration."""
        print("  Testing emergency stop integration...")
        
        # Start moving
        self.send_velocity(self.test_velocity, 0.0, 0.5)
        
        # Trigger e-stop
        if not self.estop_client.wait_for_service(timeout_sec=1.0):
            return IntegrationTestResult(
                name="Emergency Stop",
                status=TestStatus.FAILED,
                message="E-stop service not available"
            )
        
        future = self.estop_client. call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        # Check robot stopped
        time.sleep(0.3)
        
        # Try to send velocity (should be ignored)
        self.cmd_vel_pub.publish(Twist(linear=Twist().linear))
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)
        
        time.sleep(0.5)
        
        # Check velocity is zero
        if self.current_odom: 
            vel_x = self.current_odom.twist.twist.linear. x
            vel_z = self.current_odom. twist.twist.angular.z
            
            if abs(vel_x) < 0.01 and abs(vel_z) < 0.01:
                # Clear e-stop
                if self.clear_estop_client.wait_for_service(timeout_sec=1.0):
                    self.clear_estop_client.call_async(Trigger.Request())
                
                return IntegrationTestResult(
                    name="Emergency Stop",
                    status=TestStatus.PASSED,
                    message="E-stop working correctly",
                    details="Robot stopped and ignoring commands"
                )
        
        # Clear e-stop
        if self. clear_estop_client.wait_for_service(timeout_sec=1.0):
            self.clear_estop_client. call_async(Trigger.Request())
        
        return IntegrationTestResult(
            name="Emergency Stop",
            status=TestStatus.WARNING,
            message="E-stop may not be working correctly"
        )
    
    # =========================================================================
    # Main Test Runner
    # =========================================================================
    
    def run_all_tests(self) -> bool:
        """Run all integration tests."""
        print("\n" + "="*60)
        print("  PHASE 1 INTEGRATION TEST")
        print("="*60 + "\n")
        
        # Wait for systems
        print("Waiting for systems to be ready...")
        if not self.wait_for_systems():
            print("❌ Systems not ready!")
            return False
        print("✅ Systems ready\n")
        
        # Safety warning
        print("⚠️  WARNING: Robot will move during this test!")
        print("   Ensure clear space around robot.")
        print("   Press Ctrl+C to abort.\n")
        time.sleep(3)
        
        # Run tests
        self.results = []
        
        self.results.append(self.test_teleop_to_motion())
        time.sleep(1)
        
        self.results.append(self.test_odometry_to_tf())
        time.sleep(1)
        
        self. results.append(self.test_sensors_during_motion())
        time.sleep(1)
        
        self.results.append(self.test_imu_motion_detection())
        time.sleep(1)
        
        self.results.append(self.test_emergency_stop_integration())
        
        self.stop_robot()
        
        return True
    
    def print_results(self) -> bool:
        """Print test results."""
        print("\n" + "="*60)
        print("  INTEGRATION TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            status_icon = result.status.value
            print(f"{status_icon} {result.name}")
            print(f"   {result.message}")
            
            if result.details:
                for line in result.details.split('\n'):
                    print(f"   - {line}")
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
            print("\n✅ PHASE 1 INTEGRATION TEST PASSED\n")
            print("The robot is ready for Phase 2 development!")
            return True
        else:
            print("\n❌ PHASE 1 INTEGRATION TEST FAILED\n")
            print("Please fix the issues before proceeding.")
            return False


def main():
    rclpy.init()
    
    node = Phase1IntegrationTest()
    
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
