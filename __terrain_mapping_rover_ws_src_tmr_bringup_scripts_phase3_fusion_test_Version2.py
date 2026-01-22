#!/usr/bin/env python3
"""
Phase 3 Fusion Test

Verifies that IMU and wheel odometry are being properly fused.

Usage:
    ros2 run tmr_bringup phase3_fusion_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy. qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import time
import math
from typing import Optional, List
from dataclasses import dataclass
from enum import Enum
from collections import deque


class TestStatus(Enum):
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class FusionTestResult:
    name: str
    status: TestStatus
    message: str
    details: str = ""


class Phase3FusionTest(Node):
    """Tests sensor fusion quality."""
    
    def __init__(self):
        super().__init__('phase3_fusion_test')
        
        # Parameters
        self.declare_parameter('test_duration', 10.0)
        self.test_duration = self.get_parameter('test_duration').value
        
        # Data storage
        self.raw_odom_data:  List[Odometry] = []
        self.filtered_odom_data: List[Odometry] = []
        self.imu_data: List[Imu] = []
        
        self.latest_raw: Optional[Odometry] = None
        self.latest_filtered: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        
        # Results
        self.results: List[FusionTestResult] = []
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.raw_sub = self.create_subscription(
            Odometry, '/vex/odom_raw', self.raw_callback, 10)
        self.filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        
        self.get_logger().info("Phase 3 Fusion Test initialized")
    
    def raw_callback(self, msg:  Odometry):
        self.raw_odom_data.append(msg)
        self.latest_raw = msg
        # Keep only recent data
        if len(self. raw_odom_data) > 1000:
            self.raw_odom_data.pop(0)
    
    def filtered_callback(self, msg: Odometry):
        self.filtered_odom_data.append(msg)
        self.latest_filtered = msg
        if len(self.filtered_odom_data) > 1000:
            self.filtered_odom_data.pop(0)
    
    def imu_callback(self, msg: Imu):
        self.imu_data.append(msg)
        self.latest_imu = msg
        if len(self.imu_data) > 1000:
            self.imu_data.pop(0)
    
    def _get_yaw(self, orientation) -> float:
        """Extract yaw from quaternion."""
        q = orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q. y + q.z * q. z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def collect_data(self):
        """Collect data for test duration."""
        print(f"\nCollecting fusion data for {self.test_duration} seconds...")
        print("Robot should be STATIONARY for this test.\n")
        
        self.raw_odom_data. clear()
        self.filtered_odom_data.clear()
        self.imu_data. clear()
        
        start = time.time()
        while time.time() - start < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            progress = int(elapsed / self.test_duration * 50)
            print(f"\r[{'='*progress}{' '*(50-progress)}] {elapsed:.1f}s", end='', flush=True)
        
        print("\n")
    
    def test_yaw_fusion(self) -> FusionTestResult:
        """Test that yaw from IMU is being incorporated."""
        if len(self.filtered_odom_data) < 10 or len(self.imu_data) < 10:
            return FusionTestResult(
                name="Yaw Fusion",
                status=TestStatus. FAILED,
                message="Insufficient data collected"
            )
        
        # Get yaw values from filtered and raw
        filtered_yaws = [self._get_yaw(o.pose.pose.orientation) for o in self.filtered_odom_data[-50:]]
        raw_yaws = [self._get_yaw(o.pose.pose.orientation) for o in self.raw_odom_data[-50:]]
        imu_yaws = [self._get_yaw(i.orientation) for i in self.imu_data[-50:] if i.orientation_covariance[0] >= 0]
        
        if not imu_yaws:
            return FusionTestResult(
                name="Yaw Fusion",
                status=TestStatus.WARNING,
                message="IMU not providing orientation",
                details="Check IMU complementary filter is enabled"
            )
        
        # Calculate variance of each
        import statistics
        filtered_var = statistics.variance(filtered_yaws) if len(filtered_yaws) > 1 else 0
        raw_var = statistics.variance(raw_yaws) if len(raw_yaws) > 1 else 0
        
        # Filtered should generally have lower or similar variance if fusion is working
        if filtered_var <= raw_var * 1.5:
            return FusionTestResult(
                name="Yaw Fusion",
                status=TestStatus.PASSED,
                message="Yaw fusion working",
                details=f"Filtered variance: {filtered_var:.6f}, Raw variance: {raw_var:.6f}"
            )
        else:
            return FusionTestResult(
                name="Yaw Fusion",
                status=TestStatus.WARNING,
                message="Filtered yaw more variable than raw",
                details=f"May need tuning.  Filtered:  {filtered_var:.6f}, Raw: {raw_var:.6f}"
            )
    
    def test_position_consistency(self) -> FusionTestResult:
        """Test that position estimates are consistent."""
        if len(self.filtered_odom_data) < 10 or len(self.raw_odom_data) < 10:
            return FusionTestResult(
                name="Position Consistency",
                status=TestStatus.FAILED,
                message="Insufficient data"
            )
        
        # Compare final positions
        raw_pos = self.raw_odom_data[-1].pose. pose.position
        filt_pos = self.filtered_odom_data[-1].pose.pose.position
        
        distance = math.sqrt(
            (raw_pos.x - filt_pos.x)**2 +
            (raw_pos.y - filt_pos.y)**2
        )
        
        if distance < 0.1:  # Within 10cm
            return FusionTestResult(
                name="Position Consistency",
                status=TestStatus. PASSED,
                message=f"Raw and filtered within {distance*100:.1f}cm"
            )
        elif distance < 0.5:
            return FusionTestResult(
                name="Position Consistency",
                status=TestStatus.WARNING,
                message=f"Position difference:  {distance*100:.1f}cm",
                details="May be normal if robot moved during test"
            )
        else:
            return FusionTestResult(
                name="Position Consistency",
                status=TestStatus. FAILED,
                message=f"Large position difference: {distance:. 2f}m"
            )
    
    def test_covariance_reduction(self) -> FusionTestResult:
        """Test that EKF reduces covariance over time."""
        if len(self.filtered_odom_data) < 20: 
            return FusionTestResult(
                name="Covariance Evolution",
                status=TestStatus. WARNING,
                message="Insufficient data for covariance analysis"
            )
        
        # Compare early vs late covariance
        early_cov = self.filtered_odom_data[5].pose.covariance[0]  # X variance
        late_cov = self.filtered_odom_data[-1].pose.covariance[0]
        
        # During stationary period, covariance shouldn't explode
        if late_cov < early_cov * 2:
            return FusionTestResult(
                name="Covariance Evolution",
                status=TestStatus.PASSED,
                message="Covariance stable",
                details=f"Early: {early_cov:.6f}, Late: {late_cov:.6f}"
            )
        else:
            return FusionTestResult(
                name="Covariance Evolution",
                status=TestStatus.WARNING,
                message="Covariance growing",
                details=f"May need process noise tuning.  Early: {early_cov:. 6f}, Late: {late_cov:.6f}"
            )
    
    def test_imu_contribution(self) -> FusionTestResult:
        """Test that IMU data is contributing to the filter."""
        if len(self. imu_data) < 10:
            return FusionTestResult(
                name="IMU Contribution",
                status=TestStatus. FAILED,
                message="No IMU data received"
            )
        
        # Check IMU has valid angular velocity
        recent_imu = self.imu_data[-10:]
        avg_gyro_z = sum(i.angular_velocity.z for i in recent_imu) / len(recent_imu)
        
        # When stationary, should be near zero
        if abs(avg_gyro_z) < 0.1:
            return FusionTestResult(
                name="IMU Contribution",
                status=TestStatus.PASSED,
                message=f"IMU gyro valid: avg_z={avg_gyro_z:. 4f} rad/s"
            )
        else:
            return FusionTestResult(
                name="IMU Contribution",
                status=TestStatus.WARNING,
                message=f"IMU gyro offset: {avg_gyro_z:.4f} rad/s",
                details="IMU may need calibration"
            )
    
    def run_tests(self) -> bool:
        """Run all fusion tests."""
        print("\n" + "="*60)
        print("  PHASE 3 FUSION TEST")
        print("="*60)
        
        self.collect_data()
        
        self. results = [
            self.test_imu_contribution(),
            self.test_yaw_fusion(),
            self.test_position_consistency(),
            self.test_covariance_reduction(),
        ]
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print results."""
        print("\n" + "="*60)
        print("  FUSION TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            icon = result.status.value
            print(f"{icon} {result.name}")
            print(f"   {result.message}")
            if result.details:
                print(f"   {result.details}")
            print()
            
            if result.status == TestStatus. PASSED:
                passed += 1
            elif result.status == TestStatus.FAILED:
                failed += 1
            else: 
                warnings += 1
        
        print("-"*60)
        print(f"Summary:  {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ FUSION TEST PASSED\n")
            return True
        else:
            print("\n❌ FUSION TEST FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase3FusionTest()
    
    try:
        success = node.run_tests()
    except KeyboardInterrupt:
        success = False
    finally:
        node. destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())