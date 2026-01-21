#!/usr/bin/env python3
"""
Phase 1 Health Monitor

Real-time system health monitoring dashboard.
Shows status of all Phase 1 components.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray

import time
import os
import sys
from collections import deque


class HealthMonitor(Node):
    """Real-time health monitoring."""
    
    def __init__(self):
        super().__init__('phase1_health_monitor')
        
        # Connection status
        self.vex_connected = False
        self.imu_connected = False
        self.camera_connected = False
        self.tof_connected = False
        
        # Message tracking
        self.msg_times = {
            'imu': deque(maxlen=100),
            'camera': deque(maxlen=100),
            'tof': deque(maxlen=100),
            'odom': deque(maxlen=100),
            'cmd_vel': deque(maxlen=100),
        }
        
        # Sensor values
        self.imu_temp = 0.0
        self.last_cmd_vel = (0.0, 0.0)
        self.last_odom_vel = (0.0, 0.0)
        self.odom_pos = (0.0, 0.0, 0.0)
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Connection subscribers
        self.create_subscription(Bool, '/vex/connected', 
            lambda m: setattr(self, 'vex_connected', m.data), 10)
        self.create_subscription(Bool, '/imu/connected',
            lambda m: setattr(self, 'imu_connected', m.data), 10)
        self.create_subscription(Bool, '/camera/connected',
            lambda m: setattr(self, 'camera_connected', m.data), 10)
        self.create_subscription(Bool, '/tof/connected',
            lambda m: setattr(self, 'tof_connected', m.data), 10)
        
        # Data subscribers
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(Image, '/camera/image_raw', self.camera_cb, qos)
        self.create_subscription(Image, '/tof/depth/image_raw', self.tof_cb, qos)
        self.create_subscription(Odometry, '/odom', self. odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Float32, '/imu/temperature', 
            lambda m: setattr(self, 'imu_temp', m. data), 10)
        
        # Update timer
        self.create_timer(0.5, self.update_display)
    
    def imu_cb(self, msg):
        self.msg_times['imu'].append(time.time())
    
    def camera_cb(self, msg):
        self.msg_times['camera'].append(time.time())
    
    def tof_cb(self, msg):
        self.msg_times['tof'].append(time. time())
    
    def odom_cb(self, msg):
        self.msg_times['odom'].append(time.time())
        self.odom_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            0.0  # TODO: extract yaw
        )
        self.last_odom_vel = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )
    
    def cmd_vel_cb(self, msg):
        self.msg_times['cmd_vel'].append(time.time())
        self.last_cmd_vel = (msg.linear.x, msg.angular.z)
    
    def calc_rate(self, times):
        """Calculate message rate from timestamps."""
        if len(times) < 2:
            return 0.0
        
        # Only use recent messages
        now = time.time()
        recent = [t for t in times if now - t < 2.0]
        
        if len(recent) < 2:
            return 0.0
        
        return len(recent) / (recent[-1] - recent[0] + 0.001)
    
    def status_icon(self, connected, rate=None, min_rate=0):
        """Get status icon."""
        if not connected:
            return "❌"
        if rate is not None and rate < min_rate: 
            return "⚠️"
        return "✅"
    
    def update_display(self):
        """Update terminal display."""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Calculate rates
        rates = {k: self.calc_rate(v) for k, v in self.msg_times.items()}
        
        print("╔" + "═"*58 + "╗")
        print("║" + "  PHASE 1 HEALTH MONITOR". center(58) + "║")
        print("╠" + "═"*58 + "╣")
        
        # Connection status
        print("║ CONNECTIONS: ".ljust(59) + "║")
        print(f"║   {self.status_icon(self.vex_connected)} VEX Brain:   {'Connected' if self.vex_connected else 'Disconnected'}".ljust(59) + "║")
        print(f"║   {self.status_icon(self.imu_connected)} IMU:        {'Connected' if self.imu_connected else 'Disconnected'}".ljust(59) + "║")
        print(f"║   {self.status_icon(self.camera_connected)} Camera:     {'Connected' if self.camera_connected else 'Disconnected'}".ljust(59) + "║")
        print(f"║   {self.status_icon(self.tof_connected)} ToF Camera:  {'Connected' if self.tof_connected else 'Disconnected'}".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Data rates
        print("║ DATA RATES:".ljust(59) + "║")
        print(f"║   {self.status_icon(True, rates['imu'], 50)} IMU:        {rates['imu']: 6.1f} Hz (target: 100)".ljust(59) + "║")
        print(f"║   {self.status_icon(True, rates['odom'], 20)} Odometry:   {rates['odom']:6.1f} Hz (target: 50)".ljust(59) + "║")
        print(f"║   {self.status_icon(True, rates['camera'], 5)} Camera:     {rates['camera']:6.1f} Hz (target: 30)".ljust(59) + "║")
        print(f"║   {self.status_icon(True, rates['tof'], 5)} ToF:        {rates['tof']:6.1f} Hz (target: 15)".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Sensor values
        print("║ SENSOR VALUES:".ljust(59) + "║")
        print(f"║   IMU Temperature: {self.imu_temp: 5.1f} °C".ljust(59) + "║")
        print(f"║   Position: X={self.odom_pos[0]:+6.3f}  Y={self.odom_pos[1]:+6.3f} m".ljust(59) + "║")
        print(f"║   Velocity: Lin={self.last_odom_vel[0]:+5.2f} m/s  Ang={self.last_odom_vel[1]:+5.2f} rad/s".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Command values
        print("║ COMMANDS:".ljust(59) + "║")
        print(f"║   cmd_vel: Lin={self.last_cmd_vel[0]:+5.2f} m/s  Ang={self.last_cmd_vel[1]:+5.2f} rad/s".ljust(59) + "║")
        print(f"║   Rate:     {rates['cmd_vel']:6.1f} Hz".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Overall status
        all_connected = all([self.vex_connected, self.imu_connected, 
                            self.camera_connected, self. tof_connected])
        rates_ok = rates['imu'] > 50 and rates['odom'] > 20
        
        if all_connected and rates_ok: 
            status = "✅ SYSTEM HEALTHY"
            color = "\033[92m"  # Green
        elif all_connected: 
            status = "⚠️  SYSTEM DEGRADED (low rates)"
            color = "\033[93m"  # Yellow
        else:
            status = "❌ SYSTEM UNHEALTHY"
            color = "\033[91m"  # Red
        
        print(f"║ {color}{status}\033[0m".ljust(68) + "║")
        
        print("╠" + "═"*58 + "╣")
        print("║  Press Ctrl+C to exit".ljust(59) + "║")
        print("╚" + "═"*58 + "╝")


def main():
    rclpy.init()
    
    node = HealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node. destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
