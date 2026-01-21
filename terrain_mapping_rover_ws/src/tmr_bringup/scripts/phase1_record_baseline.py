#!/usr/bin/env python3
"""
Phase 1 Baseline Recording

Records baseline sensor data for performance comparison.
Robot should be stationary during this recording.
"""

import rclpy
from rclpy.node import Node
from rclpy. qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import time
import json
import math
import numpy as np
from datetime import datetime
from pathlib import Path


class BaselineRecorder(Node):
    """Records baseline sensor data."""
    
    def __init__(self):
        super().__init__('phase1_baseline_recorder')
        
        self.imu_data = []
        self.odom_data = []
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    def imu_callback(self, msg):
        self.imu_data.append({
            'time': time.time(),
            'accel_x': msg.linear_acceleration.x,
            'accel_y': msg.linear_acceleration.y,
            'accel_z': msg.linear_acceleration.z,
            'gyro_x': msg.angular_velocity.x,
            'gyro_y': msg.angular_velocity.y,
            'gyro_z': msg.angular_velocity.z,
        })
    
    def odom_callback(self, msg):
        self.odom_data.append({
            'time': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose. position.y,
            'vx': msg.twist.twist. linear.x,
            'vz': msg.twist.twist. angular.z,
        })
    
    def record(self, duration=30.0):
        """Record baseline data."""
        print("\n" + "="*50)
        print("  PHASE 1 BASELINE RECORDING")
        print("="*50)
        print(f"\nRecording for {duration} seconds...")
        print("⚠️  Keep robot STATIONARY!\n")
        
        self.imu_data.clear()
        self.odom_data.clear()
        
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            progress = int(elapsed / duration * 50)
            print(f"\r[{'='*progress}{' '*(50-progress)}] {elapsed:.1f}s / {duration}s", end='')
        
        print("\n\nProcessing data...")
        return self.analyze()
    
    def analyze(self):
        """Analyze recorded data."""
        results = {
            'timestamp': datetime.now().isoformat(),
            'imu':  {},
            'odom': {},
        }
        
        if self.imu_data:
            # IMU statistics
            accel_x = [d['accel_x'] for d in self.imu_data]
            accel_y = [d['accel_y'] for d in self.imu_data]
            accel_z = [d['accel_z'] for d in self.imu_data]
            gyro_x = [d['gyro_x'] for d in self.imu_data]
            gyro_y = [d['gyro_y'] for d in self. imu_data]
            gyro_z = [d['gyro_z'] for d in self.imu_data]
            
            results['imu'] = {
                'sample_count': len(self.imu_data),
                'rate_hz': len(self.imu_data) / 30.0,
                'accel':  {
                    'x': {'mean': np.mean(accel_x), 'std': np.std(accel_x)},
                    'y': {'mean': np.mean(accel_y), 'std': np.std(accel_y)},
                    'z': {'mean': np.mean(accel_z), 'std': np.std(accel_z)},
                    'magnitude_mean': np.mean([math.sqrt(x**2+y**2+z**2) 
                                               for x,y,z in zip(accel_x, accel_y, accel_z)]),
                },
                'gyro': {
                    'x': {'mean':  np.mean(gyro_x), 'std': np.std(gyro_x)},
                    'y': {'mean':  np.mean(gyro_y), 'std': np.std(gyro_y)},
                    'z': {'mean':  np.mean(gyro_z), 'std': np.std(gyro_z)},
                }
            }
        
        if self.odom_data:
            # Odometry statistics
            x_vals = [d['x'] for d in self.odom_data]
            y_vals = [d['y'] for d in self. odom_data]
            
            results['odom'] = {
                'sample_count': len(self.odom_data),
                'rate_hz': len(self.odom_data) / 30.0,
                'drift':  {
                    'x': max(x_vals) - min(x_vals),
                    'y':  max(y_vals) - min(y_vals),
                    'total': math.sqrt((max(x_vals)-min(x_vals))**2 + 
                                       (max(y_vals)-min(y_vals))**2),
                }
            }
        
        return results
    
    def save_results(self, results, filename=None):
        """Save results to file."""
        if filename is None:
            filename = f"phase1_baseline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        path = Path(filename)
        with open(path, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\nResults saved to:  {path}")
        return path
    
    def print_results(self, results):
        """Print results summary."""
        print("\n" + "="*50)
        print("  BASELINE RESULTS")
        print("="*50)
        
        if results['imu']: 
            imu = results['imu']
            print(f"\nIMU ({imu['sample_count']} samples, {imu['rate_hz']:.1f} Hz):")
            print(f"  Accelerometer (m/s²):")
            print(f"    X: {imu['accel']['x']['mean']: +.4f} ± {imu['accel']['x']['std']:.4f}")
            print(f"    Y: {imu['accel']['y']['mean']: +.4f} ± {imu['accel']['y']['std']:.4f}")
            print(f"    Z: {imu['accel']['z']['mean']:+.4f} ± {imu['accel']['z']['std']:.4f}")
            print(f"    Magnitude: {imu['accel']['magnitude_mean']:.4f} (expected: 9.81)")
            print(f"  Gyroscope (rad/s):")
            print(f"    X: {imu['gyro']['x']['mean']: +.6f} ± {imu['gyro']['x']['std']:. 6f}")
            print(f"    Y: {imu['gyro']['y']['mean']:+.6f} ± {imu['gyro']['y']['std']:.6f}")
            print(f"    Z: {imu['gyro']['z']['mean']:+.6f} ± {imu['gyro']['z']['std']:.6f}")
        
        if results['odom']: 
            odom = results['odom']
            print(f"\nOdometry ({odom['sample_count']} samples, {odom['rate_hz']:. 1f} Hz):")
            print(f"  Drift (stationary):")
            print(f"    X: {odom['drift']['x']*1000:.2f} mm")
            print(f"    Y: {odom['drift']['y']*1000:.2f} mm")
            print(f"    Total: {odom['drift']['total']*1000:.2f} mm")
        
        print("\n" + "="*50)


def main():
    rclpy.init()
    
    node = BaselineRecorder()
    
    try:
        results = node.record(duration=30.0)
        node.print_results(results)
        
        save = input("\nSave results to file? (y/n): ").lower()
        if save == 'y':
            node. save_results(results)
        
    except KeyboardInterrupt:
        print("\n\nRecording cancelled")
    finally:
        node. destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
