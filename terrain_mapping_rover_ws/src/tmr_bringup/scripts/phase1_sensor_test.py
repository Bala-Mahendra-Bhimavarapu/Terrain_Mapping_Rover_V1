#!/usr/bin/env python3
"""
Phase 1 Sensor Test

Tests all sensor data streams. 

Tests:
1. IMU data validity and rate
2. Camera image quality and rate
3. ToF depth data and rate
4. Sensor fusion readiness
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, Imu, PointCloud2, CameraInfo
from std_msgs. msg import Float32, Bool

import time
import math
import struct
from typing import Optional, List, Dict
from dataclasses import dataclass
from enum import Enum
from collections import deque


class TestStatus(Enum):
    PENDING = "⏳"
    RUNNING = "🔄"
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class SensorTestResult:
    name:  str
    status: TestStatus
    rate_hz: float
    expected_rate_hz: float
    message: str
    details: str = ""


class Phase1SensorTest(Node):
    """Phase 1 sensor test node."""
    
    def __init__(self):
        super().__init__('phase1_sensor_test')
        
        # Parameters
        self.declare_parameter('test_duration', 5.0)
        self.test_duration = self.get_parameter('test_duration').value
        
        # Sensor data storage
        self.imu_data:  deque = deque(maxlen=1000)
        self.imu_timestamps: deque = deque(maxlen=1000)
        
        self.camera_timestamps: deque = deque(maxlen=100)
        self.camera_info: Optional[CameraInfo] = None
        
        self.tof_depth_timestamps: deque = deque(maxlen=100)
        self.tof_points_timestamps: deque = deque(maxlen=100)
        
        # Connection status
        self.imu_connected = False
        self.camera_connected = False
        self.tof_connected = False
        
        # Temperature
        self.imu_temperature = 0.0
        
        # Results
        self.results: List[SensorTestResult] = []
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # IMU subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self. imu_callback, sensor_qos)
        self.imu_temp_sub = self.create_subscription(
            Float32, '/imu/temperature', self.imu_temp_callback, 10)
        self.imu_connected_sub = self.create_subscription(
            Bool, '/imu/connected', self.imu_connected_callback, 10)
        
        # Camera subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, sensor_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, sensor_qos)
        self.camera_connected_sub = self.create_subscription(
            Bool, '/camera/connected', self.camera_connected_callback, 10)
        
        # ToF subscribers
        self.tof_depth_sub = self.create_subscription(
            Image, '/tof/depth/image_raw', self.tof_depth_callback, sensor_qos)
        self.tof_points_sub = self.create_subscription(
            PointCloud2, '/tof/points', self.tof_points_callback, sensor_qos)
        self.tof_connected_sub = self.create_subscription(
            Bool, '/tof/connected', self.tof_connected_callback, 10)
        
        self.get_logger().info("Phase 1 Sensor Test initialized")
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def imu_callback(self, msg: Imu):
        """IMU data callback."""
        self.imu_data.append(msg)
        self.imu_timestamps. append(time.time())
    
    def imu_temp_callback(self, msg: Float32):
        """IMU temperature callback."""
        self.imu_temperature = msg.data
    
    def imu_connected_callback(self, msg: Bool):
        """IMU connection callback."""
        self.imu_connected = msg.data
    
    def camera_callback(self, msg: Image):
        """Camera image callback."""
        self.camera_timestamps.append(time.time())
    
    def camera_info_callback(self, msg: CameraInfo):
        """Camera info callback."""
        self.camera_info = msg
    
    def camera_connected_callback(self, msg: Bool):
        """Camera connection callback."""
        self.camera_connected = msg.data
    
    def tof_depth_callback(self, msg: Image):
        """ToF depth callback."""
        self.tof_depth_timestamps.append(time. time())
    
    def tof_points_callback(self, msg: PointCloud2):
        """ToF point cloud callback."""
        self.tof_points_timestamps.append(time.time())
    
    def tof_connected_callback(self, msg: Bool):
        """ToF connection callback."""
        self.tof_connected = msg. data
    
    # =========================================================================
    # Helper Functions
    # =========================================================================
    
    def calculate_rate(self, timestamps: deque) -> float:
        """Calculate message rate from timestamps."""
        if len(timestamps) < 2:
            return 0.0
        
        duration = timestamps[-1] - timestamps[0]
        if duration <= 0:
            return 0.0
        
        return (len(timestamps) - 1) / duration
    
    def clear_data(self):
        """Clear all collected data."""
        self.imu_data.clear()
        self.imu_timestamps.clear()
        self.camera_timestamps. clear()
        self.tof_depth_timestamps.clear()
        self.tof_points_timestamps.clear()
    
    def collect_data(self, duration: float):
        """Collect sensor data for specified duration."""
        self.clear_data()
        
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
    
    # =========================================================================
    # Tests
    # =========================================================================
    
    def test_imu(self) -> SensorTestResult: 
        """Test IMU sensor."""
        print("  Testing IMU...")
        
        expected_rate = 100.0  # Hz
        min_rate = 80.0  # Minimum acceptable rate
        
        # Check connection
        if not self.imu_connected:
            return SensorTestResult(
                name="IMU (MPU6050)",
                status=TestStatus.FAILED,
                rate_hz=0.0,
                expected_rate_hz=expected_rate,
                message="IMU not connected"
            )
        
        # Calculate rate
        rate = self.calculate_rate(self.imu_timestamps)
        
        # Check data validity
        data_valid = True
        issues = []
        
        if len(self.imu_data) > 0:
            # Check for valid accelerometer data (should be near 1g when stationary)
            last_imu = self.imu_data[-1]
            accel_magnitude = math.sqrt(
                last_imu.linear_acceleration. x**2 +
                last_imu.linear_acceleration.y**2 +
                last_imu.linear_acceleration.z**2
            )
            
            if not (8.0 < accel_magnitude < 12.0):
                issues.append(f"Accel magnitude:  {accel_magnitude:.2f} m/s² (expected ~9. 81)")
                data_valid = False
            
            # Check for reasonable gyro values (should be near 0 when stationary)
            gyro_magnitude = math.sqrt(
                last_imu.angular_velocity.x**2 +
                last_imu.angular_velocity.y**2 +
                last_imu.angular_velocity.z**2
            )
            
            if gyro_magnitude > 0.5:
                issues.append(f"Gyro magnitude: {gyro_magnitude:.3f} rad/s (should be ~0 when stationary)")
        else:
            data_valid = False
            issues. append("No IMU data received")
        
        # Check temperature
        if self.imu_temperature > 60.0:
            issues.append(f"High temperature: {self.imu_temperature:.1f}°C")
        
        # Determine status
        if rate >= min_rate and data_valid:
            status = TestStatus.PASSED
            message = f"IMU working at {rate:.1f} Hz"
        elif rate >= min_rate * 0.5: 
            status = TestStatus.WARNING
            message = f"IMU rate low:  {rate:.1f} Hz"
        else:
            status = TestStatus.FAILED
            message = "IMU not functioning properly"
        
        return SensorTestResult(
            name="IMU (MPU6050)",
            status=status,
            rate_hz=rate,
            expected_rate_hz=expected_rate,
            message=message,
            details="\n".join(issues) if issues else f"Temp: {self.imu_temperature:.1f}°C"
        )
    
    def test_camera(self) -> SensorTestResult:
        """Test RGB camera."""
        print("  Testing Camera...")
        
        expected_rate = 30.0  # Hz (or as configured)
        min_rate = 10.0
        
        # Check connection
        if not self.camera_connected:
            return SensorTestResult(
                name="Camera (IMX500)",
                status=TestStatus. FAILED,
                rate_hz=0.0,
                expected_rate_hz=expected_rate,
                message="Camera not connected"
            )
        
        # Calculate rate
        rate = self.calculate_rate(self.camera_timestamps)
        
        # Check camera info
        issues = []
        
        if self.camera_info is not None:
            width = self.camera_info.width
            height = self.camera_info.height
            issues.append(f"Resolution: {width}x{height}")
        else:
            issues.append("No camera_info received")
        
        # Determine status
        if rate >= min_rate: 
            status = TestStatus. PASSED
            message = f"Camera working at {rate:.1f} Hz"
        elif rate > 0:
            status = TestStatus.WARNING
            message = f"Camera rate low: {rate:.1f} Hz"
        else:
            status = TestStatus.FAILED
            message = "Camera not producing images"
        
        return SensorTestResult(
            name="Camera (IMX500)",
            status=status,
            rate_hz=rate,
            expected_rate_hz=expected_rate,
            message=message,
            details="\n".join(issues)
        )
    
    def test_tof_camera(self) -> SensorTestResult:
        """Test ToF camera."""
        print("  Testing ToF Camera...")
        
        expected_rate = 15.0  # Hz
        min_rate = 10.0
        
        # Check connection
        if not self.tof_connected:
            return SensorTestResult(
                name="ToF Camera (B0410)",
                status=TestStatus. FAILED,
                rate_hz=0.0,
                expected_rate_hz=expected_rate,
                message="ToF camera not connected"
            )
        
        # Calculate rates
        depth_rate = self.calculate_rate(self.tof_depth_timestamps)
        points_rate = self.calculate_rate(self.tof_points_timestamps)
        
        issues = [
            f"Depth image rate: {depth_rate:.1f} Hz",
            f"Point cloud rate: {points_rate:. 1f} Hz"
        ]
        
        # Determine status
        rate = depth_rate
        if rate >= min_rate: 
            status = TestStatus. PASSED
            message = f"ToF working at {rate:.1f} Hz"
        elif rate > 0:
            status = TestStatus. WARNING
            message = f"ToF rate low: {rate:.1f} Hz"
        else: 
            status = TestStatus. FAILED
            message = "ToF not producing data"
        
        return SensorTestResult(
            name="ToF Camera (B0410)",
            status=status,
            rate_hz=rate,
            expected_rate_hz=expected_rate,
            message=message,
            details="\n".join(issues)
        )
    
    def test_sensor_synchronization(self) -> SensorTestResult:
        """Test sensor synchronization."""
        print("  Testing sensor synchronization...")
        
        # Check if all sensors have recent data
        current_time = time.time()
        max_age = 1.0  # 1 second
        
        sensors_synced = True
        issues = []
        
        if self.imu_timestamps:
            imu_age = current_time - self.imu_timestamps[-1]
            if imu_age > max_age:
                sensors_synced = False
                issues.append(f"IMU data stale: {imu_age:. 2f}s old")
        else:
            sensors_synced = False
            issues.append("No IMU data")
        
        if self.camera_timestamps:
            camera_age = current_time - self.camera_timestamps[-1]
            if camera_age > max_age: 
                sensors_synced = False
                issues.append(f"Camera data stale: {camera_age:.2f}s old")
        else:
            sensors_synced = False
            issues. append("No camera data")
        
        if self.tof_depth_timestamps:
            tof_age = current_time - self.tof_depth_timestamps[-1]
            if tof_age > max_age:
                sensors_synced = False
                issues.append(f"ToF data stale: {tof_age:.2f}s old")
        else:
            sensors_synced = False
            issues. append("No ToF data")
        
        if sensors_synced:
            return SensorTestResult(
                name="Sensor Sync",
                status=TestStatus. PASSED,
                rate_hz=0.0,
                expected_rate_hz=0.0,
                message="All sensors producing current data",
                details="Ready for sensor fusion"
            )
        else:
            return SensorTestResult(
                name="Sensor Sync",
                status=TestStatus.WARNING if len(issues) < 3 else TestStatus. FAILED,
                rate_hz=0.0,
                expected_rate_hz=0.0,
                message="Sensor synchronization issues",
                details="\n". join(issues)
            )
    
    # =========================================================================
    # Main Test Runner
    # =========================================================================
    
    def run_all_tests(self):
        """Run all sensor tests."""
        print("\n" + "="*60)
        print("  PHASE 1 SENSOR TEST")
        print("="*60 + "\n")
        
        print(f"Collecting sensor data for {self.test_duration} seconds.. .\n")
        self.collect_data(self.test_duration)
        
        self.results = []
        
        self.results.append(self.test_imu())
        self.results.append(self.test_camera())
        self.results. append(self.test_tof_camera())
        self.results.append(self.test_sensor_synchronization())
        
        return True
    
    def print_results(self):
        """Print test results."""
        print("\n" + "="*60)
        print("  SENSOR TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            status_icon = result.status.value
            print(f"{status_icon} {result.name}")
            
            if result.expected_rate_hz > 0:
                print(f"   Rate: {result.rate_hz:. 1f} Hz (expected: {result.expected_rate_hz:.1f} Hz)")
            
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
            print("\n✅ ALL SENSOR TESTS PASSED\n")
            return True
        else:
            print("\n❌ SOME SENSOR TESTS FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase1SensorTest()
    
    try:
        node.run_all_tests()
        success = node.print_results()
        
    except KeyboardInterrupt:
        print("\n\nTest aborted by user")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
