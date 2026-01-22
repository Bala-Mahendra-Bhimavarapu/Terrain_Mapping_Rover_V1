#!/usr/bin/env python3
"""
Interactive Covariance Tuner

Allows real-time adjustment of EKF covariance parameters. 
Changes are displayed but must be saved to config file manually.

Usage:
    ros2 run tmr_localization covariance_tuner.py
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import os
import sys
import yaml
from typing import Dict, Any
from dataclasses import dataclass


@dataclass
class TuningParameter:
    """A tunable parameter."""
    name: str
    value: float
    min_val: float
    max_val:  float
    step: float
    description: str


class CovarianceTunerNode(Node):
    """Interactive EKF covariance tuner."""
    
    def __init__(self):
        super().__init__('covariance_tuner')
        
        # Tunable parameters
        self.params: Dict[str, TuningParameter] = {
            'odom_x': TuningParameter(
                'Odom X Variance', 0.01, 0.001, 1.0, 0.005,
                'Wheel odometry X position variance'),
            'odom_y':  TuningParameter(
                'Odom Y Variance', 0.01, 0.001, 1.0, 0.005,
                'Wheel odometry Y position variance'),
            'odom_yaw': TuningParameter(
                'Odom Yaw Variance', 0.03, 0.001, 1.0, 0.01,
                'Wheel odometry yaw variance'),
            'odom_vx': TuningParameter(
                'Odom Vx Variance', 0.01, 0.001, 1.0, 0.005,
                'Wheel odometry linear velocity variance'),
            'odom_vyaw': TuningParameter(
                'Odom Vyaw Variance', 0.03, 0.001, 1.0, 0.01,
                'Wheel odometry angular velocity variance'),
            'imu_yaw': TuningParameter(
                'IMU Yaw Variance', 0.01, 0.001, 1.0, 0.005,
                'IMU orientation yaw variance'),
            'imu_vyaw': TuningParameter(
                'IMU Vyaw Variance', 0.001, 0.0001, 0.1, 0.001,
                'IMU angular velocity variance'),
            'imu_ax': TuningParameter(
                'IMU Ax Variance', 0.01, 0.001, 1.0, 0.005,
                'IMU linear acceleration X variance'),
            'process_pos': TuningParameter(
                'Process Pos Noise', 0.05, 0.001, 1.0, 0.01,
                'Process noise for position'),
            'process_yaw': TuningParameter(
                'Process Yaw Noise', 0.06, 0.001, 1.0, 0.01,
                'Process noise for yaw'),
        }
        
        self.selected_idx = 0
        self.param_keys = list(self.params.keys())
        
        # Latest sensor data for display
        self.latest_odom_raw = None
        self.latest_odom_filtered = None
        self.latest_imu = None
        
        # Subscribers
        self.create_subscription(Odometry, '/vex/odom_raw', 
            lambda m: setattr(self, 'latest_odom_raw', m), 10)
        self.create_subscription(Odometry, '/odometry/filtered',
            lambda m: setattr(self, 'latest_odom_filtered', m), 10)
        self.create_subscription(Imu, '/imu/data',
            lambda m: setattr(self, 'latest_imu', m), 10)
        
        # Timer for display
        self.timer = self.create_timer(0.2, self.update_display)
        
        # Keyboard input setup
        self.setup_keyboard()
        
        self.get_logger().info("Covariance Tuner started")
        self.get_logger().info("Use arrow keys to adjust, 's' to save config")
    
    def setup_keyboard(self):
        """Setup keyboard input (Linux only)."""
        if sys. platform != 'win32':
            import termios
            import tty
            self.old_settings = termios. tcgetattr(sys.stdin)
    
    def cleanup_keyboard(self):
        """Restore terminal settings."""
        if sys. platform != 'win32': 
            import termios
            termios.tcsetattr(sys. stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """Get keyboard input (non-blocking)."""
        if sys.platform == 'win32':
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt. getwch()
            return None
        else:
            import select
            import termios
            import tty
            
            if select.select([sys.stdin], [], [], 0)[0]:
                tty.setraw(sys.stdin.fileno())
                try:
                    key = sys. stdin.read(1)
                    if key == '\x1b':  # Escape sequence
                        key += sys.stdin.read(2)
                    return key
                finally:
                    termios.tcsetattr(sys.stdin, termios. TCSADRAIN, self.old_settings)
            return None
    
    def update_display(self):
        """Update the display and handle input."""
        # Handle keyboard input
        key = self. get_key()
        if key:
            self.handle_key(key)
        
        # Clear and redraw
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("╔" + "═"*66 + "╗")
        print("║" + "  EKF COVARIANCE TUNER".center(66) + "║")
        print("╠" + "═"*66 + "╣")
        
        # Instructions
        print("║  ↑/↓:  Select parameter  ←/→: Adjust value  s: Save  q: Quit". ljust(67) + "║")
        print("╠" + "═"*66 + "╣")
        
        # Parameters
        print("║ TUNABLE PARAMETERS:".ljust(67) + "║")
        print("║" + "-"*66 + "║")
        
        for i, key in enumerate(self.param_keys):
            param = self.params[key]
            selected = "►" if i == self.selected_idx else " "
            bar_width = 20
            fill = int((param.value - param.min_val) / (param.max_val - param.min_val) * bar_width)
            bar = "█" * fill + "░" * (bar_width - fill)
            
            line = f"{selected} {param.name:20s} [{bar}] {param.value:.5f}"
            print(f"║ {line}".ljust(67) + "║")
        
        print("╠" + "═"*66 + "╣")
        
        # Current sensor data
        print("║ SENSOR STATUS:".ljust(67) + "║")
        odom_ok = "✅" if self.latest_odom_raw else "❌"
        ekf_ok = "✅" if self.latest_odom_filtered else "❌"
        imu_ok = "✅" if self.latest_imu else "❌"
        print(f"║   Wheel Odom: {odom_ok}  EKF Output: {ekf_ok}  IMU: {imu_ok}".ljust(67) + "║")
        
        print("╠" + "═"*66 + "╣")
        
        # Selected parameter description
        selected_param = self.params[self.param_keys[self.selected_idx]]
        print(f"║ {selected_param.description}".ljust(67) + "║")
        print(f"║ Range: [{selected_param.min_val:.4f} - {selected_param.max_val:.4f}]  Step: {selected_param.step:.4f}".ljust(67) + "║")
        
        print("╚" + "═"*66 + "╝")
    
    def handle_key(self, key:  str):
        """Handle keyboard input."""
        if key == '\x1b[A':  # Up arrow
            self.selected_idx = (self.selected_idx - 1) % len(self.param_keys)
        elif key == '\x1b[B':  # Down arrow
            self.selected_idx = (self.selected_idx + 1) % len(self.param_keys)
        elif key == '\x1b[C':  # Right arrow - increase
            param_key = self.param_keys[self.selected_idx]
            param = self.params[param_key]
            param.value = min(param.value + param.step, param.max_val)
        elif key == '\x1b[D':  # Left arrow - decrease
            param_key = self.param_keys[self.selected_idx]
            param = self.params[param_key]
            param.value = max(param.value - param.step, param.min_val)
        elif key.lower() == 's':
            self.save_config()
        elif key.lower() == 'q' or key == '\x03':  # q or Ctrl+C
            self.cleanup_keyboard()
            raise KeyboardInterrupt
    
    def save_config(self):
        """Save current parameters to a YAML file."""
        config = {
            'tuned_covariances': {
                'wheel_odometry':  {
                    'x_variance': self.params['odom_x'].value,
                    'y_variance': self.params['odom_y'].value,
                    'yaw_variance': self.params['odom_yaw'].value,
                    'vx_variance': self. params['odom_vx']. value,
                    'vyaw_variance': self.params['odom_vyaw'].value,
                },
                'imu':  {
                    'yaw_variance': self.params['imu_yaw'].value,
                    'vyaw_variance': self.params['imu_vyaw'].value,
                    'ax_variance': self.params['imu_ax'].value,
                },
                'process_noise': {
                    'position': self.params['process_pos'].value,
                    'yaw': self.params['process_yaw'].value,
                }
            }
        }
        
        filename = 'tuned_covariances.yaml'
        with open(filename, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        self.get_logger().info(f"Configuration saved to {filename}")
        print(f"\n✅ Saved to {filename}")


def main():
    rclpy.init()
    
    node = CovarianceTunerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup_keyboard()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
