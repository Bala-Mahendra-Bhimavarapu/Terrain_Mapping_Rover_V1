#!/usr/bin/env python3
"""
EKF Diagnostics Node

Monitors EKF performance and publishes diagnostics.
Shows sensor input rates, covariance values, and fusion quality.

Usage:
    ros2 run tmr_localization ekf_diagnostics.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import time
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass
class SensorStats:
    """Statistics for a sensor input."""
    name: str
    topic: str
    msg_count: int = 0
    last_time: float = 0.0
    rate_hz: float = 0.0
    timestamps: deque = None
    
    def __post_init__(self):
        if self.timestamps is None:
            self.timestamps = deque(maxlen=100)


class EKFDiagnosticsNode(Node):
    """Monitors EKF health and performance."""
    
    def __init__(self):
        super().__init__('ekf_diagnostics')
        
        # Parameters
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('min_odom_rate', 20.0)
        self.declare_parameter('min_imu_rate', 50.0)
        self.declare_parameter('max_covariance', 1.0)
        
        self.update_rate = self.get_parameter('update_rate').value
        self.min_odom_rate = self.get_parameter('min_odom_rate').value
        self.min_imu_rate = self.get_parameter('min_imu_rate').value
        self.max_covariance = self.get_parameter('max_covariance').value
        
        # Sensor stats tracking
        self.sensors = {
            'wheel_odom': SensorStats(name='Wheel Odometry', topic='/vex/odom_raw'),
            'imu': SensorStats(name='IMU', topic='/imu/data'),
            'ekf_output': SensorStats(name='EKF Output', topic='/odometry/filtered'),
        }
        
        # Latest data
        self.latest_odom_raw:  Optional[Odometry] = None
        self.latest_odom_filtered: Optional[Odometry] = None
        self. latest_imu: Optional[Imu] = None
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.odom_raw_sub = self.create_subscription(
            Odometry, '/vex/odom_raw', self.odom_raw_callback, 10)
        self.odom_filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_filtered_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        
        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Timer
        self.timer = self. create_timer(1.0 / self.update_rate, self.diagnostics_callback)
        
        self.get_logger().info("EKF Diagnostics started")
    
    def _update_sensor_stats(self, sensor_key: str):
        """Update rate statistics for a sensor."""
        sensor = self.sensors[sensor_key]
        sensor.msg_count += 1
        sensor.timestamps.append(time.time())
        sensor.last_time = time.time()
        
        # Calculate rate
        if len(sensor.timestamps) >= 2:
            duration = sensor.timestamps[-1] - sensor. timestamps[0]
            if duration > 0:
                sensor.rate_hz = (len(sensor.timestamps) - 1) / duration
    
    def odom_raw_callback(self, msg: Odometry):
        self. latest_odom_raw = msg
        self._update_sensor_stats('wheel_odom')
    
    def odom_filtered_callback(self, msg:  Odometry):
        self.latest_odom_filtered = msg
        self._update_sensor_stats('ekf_output')
    
    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self._update_sensor_stats('imu')
    
    def _check_covariance(self, cov: list, name: str) -> tuple:
        """Check if covariance values are reasonable."""
        if not cov:
            return DiagnosticStatus. ERROR, f"{name} covariance not available"
        
        # Check diagonal values (0, 7, 14, 21, 28, 35 for 6x6)
        diag_indices = [0, 7, 14, 21, 28, 35]
        
        for i in diag_indices[: 3]:  # Check x, y, z (or yaw)
            if i < len(cov):
                if cov[i] > self.max_covariance:
                    return DiagnosticStatus. WARN, f"{name} covariance high:  {cov[i]:. 4f}"
                if cov[i] < 0:
                    return DiagnosticStatus.ERROR, f"{name} covariance negative"
        
        return DiagnosticStatus.OK, f"{name} covariance OK"
    
    def diagnostics_callback(self):
        """Publish diagnostics and update display."""
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        # =====================================================================
        # Check Wheel Odometry
        # =====================================================================
        odom_status = DiagnosticStatus()
        odom_status.name = "EKF:  Wheel Odometry Input"
        odom_status. hardware_id = "tmr_localization"
        
        odom_sensor = self.sensors['wheel_odom']
        odom_status.values.append(KeyValue(key='Rate (Hz)', value=f'{odom_sensor.rate_hz:.1f}'))
        odom_status.values.append(KeyValue(key='Messages', value=str(odom_sensor.msg_count)))
        
        if odom_sensor.rate_hz < self.min_odom_rate:
            odom_status.level = DiagnosticStatus. WARN
            odom_status. message = f"Low rate: {odom_sensor.rate_hz:.1f} Hz"
        elif odom_sensor. rate_hz == 0:
            odom_status.level = DiagnosticStatus.ERROR
            odom_status.message = "No data received"
        else:
            odom_status.level = DiagnosticStatus.OK
            odom_status.message = f"OK: {odom_sensor.rate_hz:.1f} Hz"
        
        diag_msg.status. append(odom_status)
        
        # =====================================================================
        # Check IMU
        # =====================================================================
        imu_status = DiagnosticStatus()
        imu_status.name = "EKF: IMU Input"
        imu_status.hardware_id = "tmr_localization"
        
        imu_sensor = self.sensors['imu']
        imu_status.values.append(KeyValue(key='Rate (Hz)', value=f'{imu_sensor.rate_hz:.1f}'))
        imu_status. values.append(KeyValue(key='Messages', value=str(imu_sensor.msg_count)))
        
        if imu_sensor.rate_hz < self. min_imu_rate: 
            imu_status.level = DiagnosticStatus.WARN
            imu_status.message = f"Low rate: {imu_sensor.rate_hz:. 1f} Hz"
        elif imu_sensor.rate_hz == 0:
            imu_status.level = DiagnosticStatus.ERROR
            imu_status.message = "No data received"
        else:
            imu_status.level = DiagnosticStatus.OK
            imu_status.message = f"OK: {imu_sensor.rate_hz:.1f} Hz"
        
        diag_msg.status.append(imu_status)
        
        # =====================================================================
        # Check EKF Output
        # =====================================================================
        ekf_status = DiagnosticStatus()
        ekf_status.name = "EKF:  Filtered Output"
        ekf_status. hardware_id = "tmr_localization"
        
        ekf_sensor = self.sensors['ekf_output']
        ekf_status.values.append(KeyValue(key='Rate (Hz)', value=f'{ekf_sensor.rate_hz:.1f}'))
        
        if self.latest_odom_filtered:
            pos = self.latest_odom_filtered.pose.pose.position
            ekf_status.values.append(KeyValue(key='Position', value=f'({pos.x:.3f}, {pos.y:.3f})'))
            
            # Check covariance
            cov_level, cov_msg = self._check_covariance(
                list(self.latest_odom_filtered.pose.covariance), "Pose")
            ekf_status.values.append(KeyValue(key='Covariance', value=cov_msg))
        
        if ekf_sensor.rate_hz == 0:
            ekf_status.level = DiagnosticStatus.ERROR
            ekf_status.message = "EKF not producing output"
        else:
            ekf_status.level = DiagnosticStatus.OK
            ekf_status.message = f"OK: {ekf_sensor. rate_hz:.1f} Hz"
        
        diag_msg.status.append(ekf_status)
        
        # Publish diagnostics
        self.diag_pub.publish(diag_msg)
        
        # Print to terminal
        self._print_status()
    
    def _print_status(self):
        """Print status to terminal."""
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("╔" + "═"*58 + "╗")
        print("║" + "  EKF DIAGNOSTICS". center(58) + "║")
        print("╠" + "═"*58 + "╣")
        
        # Sensor rates
        print("║ SENSOR INPUTS: ".ljust(59) + "║")
        for key, sensor in self.sensors. items():
            status = "✅" if sensor.rate_hz > 0 else "❌"
            print(f"║   {status} {sensor.name:20s}:  {sensor.rate_hz:6.1f} Hz". ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Position comparison
        print("║ POSITION: ".ljust(59) + "║")
        if self.latest_odom_raw:
            pos = self.latest_odom_raw.pose. pose.position
            print(f"║   Raw:       X={pos.x:+7.3f}  Y={pos.y:+7.3f} m".ljust(59) + "║")
        
        if self.latest_odom_filtered:
            pos = self.latest_odom_filtered.pose.pose.position
            print(f"║   Filtered: X={pos.x:+7.3f}  Y={pos.y:+7.3f} m". ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Covariance
        print("║ COVARIANCE (filtered):".ljust(59) + "║")
        if self.latest_odom_filtered: 
            cov = self.latest_odom_filtered.pose.covariance
            print(f"║   X:    {cov[0]:.6f}". ljust(59) + "║")
            print(f"║   Y:   {cov[7]:.6f}".ljust(59) + "║")
            print(f"║   Yaw: {cov[35]:.6f}".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Overall status
        all_ok = all(s.rate_hz > 0 for s in self.sensors.values())
        if all_ok:
            print("║  ✅ EKF RUNNING NORMALLY".ljust(59) + "║")
        else:
            print("║  ⚠️  EKF HAS ISSUES".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        print("║  Press Ctrl+C to exit".ljust(59) + "║")
        print("╚" + "═"*58 + "╝")


def main():
    rclpy.init()
    
    node = EKFDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
