#!/usr/bin/env python3
"""
Phase 3 EKF Check

Verifies that the EKF is running and producing valid output.

Usage:
    ros2 run tmr_bringup phase3_ekf_check. py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
class EKFCheckResult:
    name:  str
    status: TestStatus
    message: str
    details:  str = ""


class Phase3EKFCheck(Node):
    """Phase 3 EKF validation."""
    
    def __init__(self):
        super().__init__('phase3_ekf_check')
        
        # Parameters
        self.declare_parameter('test_duration', 5.0)
        self.test_duration = self.get_parameter('test_duration').value
        
        # Data tracking
        self.odom_raw_count = 0
        self.imu_count = 0
        self.odom_filtered_count = 0
        
        self. odom_raw_times = deque(maxlen=100)
        self.imu_times = deque(maxlen=100)
        self.odom_filtered_times = deque(maxlen=100)
        
        self.latest_raw: Optional[Odometry] = None
        self.latest_filtered: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        
        # Results
        self.results: List[EKFCheckResult] = []
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.raw_sub = self.create_subscription(
            Odometry, '/vex/odom_raw', self.raw_callback, 10)
        self.filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_callback, 10)
        self.imu_sub = self. create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        
        self.get_logger().info("Phase 3 EKF Check initialized")
    
    def raw_callback(self, msg:  Odometry):
        self.odom_raw_count += 1
        self.odom_raw_times.append(time. time())
        self.latest_raw = msg
    
    def filtered_callback(self, msg:  Odometry):
        self.odom_filtered_count += 1
        self. odom_filtered_times.append(time.time())
        self.latest_filtered = msg
    
    def imu_callback(self, msg: Imu):
        self.imu_count += 1
        self.imu_times.append(time. time())
        self.latest_imu = msg
    
    def calculate_rate(self, times: deque) -> float:
        """Calculate message rate from timestamps."""
        if len(times) < 2:
            return 0.0
        duration = times[-1] - times[0]
        if duration <= 0:
            return 0.0
        return (len(times) - 1) / duration
    
    def collect_data(self):
        """Collect sensor data for test duration."""
        print(f"\nCollecting data for {self. test_duration} seconds...")
        
        start = time.time()
        while time.time() - start < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            progress = int(elapsed / self. test_duration * 50)
            print(f"\r[{'='*progress}{' '*(50-progress)}] {elapsed:.1f}s", end='', flush=True)
        
        print("\n")
    
    def check_sensor_inputs(self) -> EKFCheckResult: 
        """Check that sensor inputs are being received."""
        raw_rate = self.calculate_rate(self. odom_raw_times)
        imu_rate = self. calculate_rate(self.imu_times)
        
        issues = []
        if raw_rate < 20:
            issues.append(f"Odom raw rate low: {raw_rate:.1f} Hz")
        if imu_rate < 50:
            issues. append(f"IMU rate low: {imu_rate:.1f} Hz")
        
        if self.odom_raw_count == 0:
            return EKFCheckResult(
                name="Sensor Inputs",
                status=TestStatus.FAILED,
                message="No wheel odometry received",
                details="Check vex_serial_node is running"
            )
        
        if self.imu_count == 0:
            return EKFCheckResult(
                name="Sensor Inputs",
                status=TestStatus.FAILED,
                message="No IMU data received",
                details="Check imu_node is running"
            )
        
        if issues: 
            return EKFCheckResult(
                name="Sensor Inputs",
                status=TestStatus. WARNING,
                message="Sensors running but rates low",
                details="; ".join(issues)
            )
        
        return EKFCheckResult(
            name="Sensor Inputs",
            status=TestStatus.PASSED,
            message=f"Odom:  {raw_rate:.1f} Hz, IMU: {imu_rate:.1f} Hz"
        )
    
    def check_ekf_output(self) -> EKFCheckResult:
        """Check that EKF is producing output."""
        filtered_rate = self.calculate_rate(self.odom_filtered_times)
        
        if self. odom_filtered_count == 0:
            return EKFCheckResult(
                name="EKF Output",
                status=TestStatus.FAILED,
                message="No filtered odometry received",
                details="Check EKF node is running"
            )
        
        if filtered_rate < 20:
            return EKFCheckResult(
                name="EKF Output",
                status=TestStatus.WARNING,
                message=f"EKF output rate low: {filtered_rate:. 1f} Hz"
            )
        
        return EKFCheckResult(
            name="EKF Output",
            status=TestStatus.PASSED,
            message=f"EKF producing output at {filtered_rate:.1f} Hz"
        )
    
    def check_output_valid(self) -> EKFCheckResult:
        """Check that EKF output is valid."""
        if self.latest_filtered is None:
            return EKFCheckResult(
                name="Output Validity",
                status=TestStatus.FAILED,
                message="No filtered odometry to check"
            )
        
        pos = self.latest_filtered.pose. pose.position
        ori = self.latest_filtered.pose.pose.orientation
        
        # Check for NaN
        if any(math.isnan(v) for v in [pos.x, pos.y, pos.z]):
            return EKFCheckResult(
                name="Output Validity",
                status=TestStatus. FAILED,
                message="Position contains NaN"
            )
        
        if any(math. isnan(v) for v in [ori.x, ori.y, ori.z, ori.w]):
            return EKFCheckResult(
                name="Output Validity",
                status=TestStatus.FAILED,
                message="Orientation contains NaN"
            )
        
        # Check quaternion normalized
        norm = math. sqrt(ori.x**2 + ori. y**2 + ori.z**2 + ori.w**2)
        if abs(norm - 1.0) > 0.01:
            return EKFCheckResult(
                name="Output Validity",
                status=TestStatus.WARNING,
                message=f"Quaternion not normalized: {norm:.4f}"
            )
        
        return EKFCheckResult(
            name="Output Validity",
            status=TestStatus.PASSED,
            message=f"Position:  ({pos.x:.3f}, {pos.y:.3f}), valid quaternion"
        )
    
    def check_covariance(self) -> EKFCheckResult:
        """Check that covariance is reasonable."""
        if self.latest_filtered is None:
            return EKFCheckResult(
                name="Covariance",
                status=TestStatus.FAILED,
                message="No data to check"
            )
        
        cov = self.latest_filtered.pose.covariance
        
        # Check diagonal elements are positive
        diag = [cov[0], cov[7], cov[14], cov[21], cov[28], cov[35]]
        
        if any(v < 0 for v in diag):
            return EKFCheckResult(
                name="Covariance",
                status=TestStatus.FAILED,
                message="Negative covariance detected"
            )
        
        if any(v > 10.0 for v in diag[: 2]):  # X, Y position
            return EKFCheckResult(
                name="Covariance",
                status=TestStatus.WARNING,
                message=f"High position covariance: X={diag[0]:.4f}, Y={diag[1]:.4f}"
            )
        
        return EKFCheckResult(
            name="Covariance",
            status=TestStatus.PASSED,
            message=f"Covariance reasonable: X={diag[0]:.4f}, Y={diag[1]:.4f}, Yaw={diag[5]:.4f}"
        )
    
    def check_frame_ids(self) -> EKFCheckResult:
        """Check frame IDs are correct."""
        if self.latest_filtered is None:
            return EKFCheckResult(
                name="Frame IDs",
                status=TestStatus.FAILED,
                message="No data to check"
            )
        
        header_frame = self.latest_filtered.header. frame_id
        child_frame = self.latest_filtered.child_frame_id
        
        issues = []
        if header_frame != "odom": 
            issues.append(f"header. frame_id={header_frame} (expected:  odom)")
        if child_frame != "base_link":
            issues.append(f"child_frame_id={child_frame} (expected:  base_link)")
        
        if issues:
            return EKFCheckResult(
                name="Frame IDs",
                status=TestStatus. FAILED,
                message="Incorrect frame IDs",
                details="; ".join(issues)
            )
        
        return EKFCheckResult(
            name="Frame IDs",
            status=TestStatus. PASSED,
            message="Frame IDs correct:  odom -> base_link"
        )
    
    def run_checks(self) -> bool:
        """Run all EKF checks."""
        print("\n" + "="*60)
        print("  PHASE 3 EKF CHECK")
        print("="*60)
        
        self.collect_data()
        
        self.results = [
            self.check_sensor_inputs(),
            self.check_ekf_output(),
            self.check_output_valid(),
            self.check_covariance(),
            self.check_frame_ids(),
        ]
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print results and return success status."""
        print("\n" + "="*60)
        print("  EKF CHECK RESULTS")
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
            print("\n✅ PHASE 3 EKF CHECK PASSED\n")
            return True
        else:
            print("\n❌ PHASE 3 EKF CHECK FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase3EKFCheck()
    
    try:
        success = node.run_checks()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())