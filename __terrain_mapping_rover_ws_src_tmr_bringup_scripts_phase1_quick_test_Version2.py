#!/usr/bin/env python3
"""
Phase 1 Quick Test

A fast sanity check that verifies basic connectivity without moving the robot.
Use this for quick validation before full testing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry

import time
import sys


class QuickTest(Node):
    """Quick connectivity test."""
    
    def __init__(self):
        super().__init__('phase1_quick_test')
        
        self.results = {
            'vex':  False,
            'imu':  False,
            'camera': False,
            'tof': False,
            'odom': False,
        }
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            depth=1
        )
        
        # Subscribers
        self.create_subscription(Bool, '/vex/connected', 
            lambda m: self._set_result('vex', m.data), 10)
        self.create_subscription(Bool, '/imu/connected',
            lambda m: self._set_result('imu', m. data), 10)
        self.create_subscription(Bool, '/camera/connected',
            lambda m: self._set_result('camera', m.data), 10)
        self.create_subscription(Bool, '/tof/connected',
            lambda m: self._set_result('tof', m. data), 10)
        self.create_subscription(Odometry, '/odom',
            lambda m: self._set_result('odom', True), qos)
    
    def _set_result(self, key, value):
        self.results[key] = value
    
    def run_test(self, timeout=5.0):
        """Run quick connectivity test."""
        print("\n" + "="*50)
        print("  PHASE 1 QUICK TEST")
        print("="*50)
        print("\nChecking connectivity (no motion)...\n")
        
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if all connected
            if all(self.results.values()):
                break
        
        # Print results
        all_pass = True
        for name, connected in self.results.items():
            status = "✅" if connected else "❌"
            print(f"  {status} {name. upper():10s}:  {'Connected' if connected else 'NOT CONNECTED'}")
            if not connected:
                all_pass = False
        
        print("\n" + "-"*50)
        
        if all_pass:
            print("✅ QUICK TEST PASSED - All systems connected!")
            print("   Ready for full Phase 1 testing.")
        else:
            print("❌ QUICK TEST FAILED - Some systems not connected")
            print("   Check connections and try again.")
        
        print("-"*50 + "\n")
        
        return all_pass


def main():
    rclpy.init()
    
    node = QuickTest()
    
    try:
        success = node.run_test()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())