#!/usr/bin/env python3
"""
Phase 1 System Check

Verifies that all Phase 1 components are running and communicating properly. 

Checks:
- ROS nodes are running
- Topics are being published
- Services are available
- TF tree is valid
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, Imu, PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import subprocess
import time
from typing import Dict, List, Tuple
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PENDING = "⏳"
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class TestResult: 
    name: str
    status: TestStatus
    message: str
    details: str = ""


class Phase1SystemCheck(Node):
    """Phase 1 system check node."""
    
    def __init__(self):
        super().__init__('phase1_system_check')
        
        self.results: List[TestResult] = []
        
        # Expected topics
        self.expected_topics = {
            '/cmd_vel':  Twist,
            '/odom': Odometry,
            '/imu/data': Imu,
            '/camera/image_raw': Image,
            '/tof/depth/image_raw': Image,
            '/tof/points': PointCloud2,
            '/vex/connected':  Bool,
            '/imu/connected': Bool,
            '/camera/connected': Bool,
            '/tof/connected': Bool,
        }
        
        # Topic reception tracking
        self.topic_received:  Dict[str, bool] = {t: False for t in self.expected_topics}
        self.topic_counts: Dict[str, int] = {t: 0 for t in self.expected_topics}
        
        # Expected nodes
        self.expected_nodes = [
            '/vex_serial_node',
            '/imu_node',
            '/camera_node',
            '/tof_camera_node',
            '/robot_state_publisher',
        ]
        
        # Expected services
        self.expected_services = [
            '/vex/emergency_stop',
            '/imu/calibrate',
        ]
        
        # Create subscribers for all expected topics
        self._create_subscribers()
        
        self.get_logger().info("Phase 1 System Check initialized")
    
    def _create_subscribers(self):
        """Create subscribers for all expected topics."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            depth=1
        )
        
        for topic, msg_type in self.expected_topics.items():
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._topic_callback(t),
                qos
            )
    
    def _topic_callback(self, topic:  str):
        """Callback for topic reception."""
        self.topic_received[topic] = True
        self.topic_counts[topic] += 1
    
    def check_nodes(self) -> TestResult:
        """Check if expected nodes are running."""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            running_nodes = result.stdout.strip().split('\n')
            missing_nodes = []
            
            for node in self.expected_nodes:
                if node not in running_nodes: 
                    missing_nodes.append(node)
            
            if not missing_nodes:
                return TestResult(
                    name="Node Check",
                    status=TestStatus.PASSED,
                    message=f"All {len(self.expected_nodes)} expected nodes running",
                    details="\n".join(self.expected_nodes)
                )
            else:
                return TestResult(
                    name="Node Check",
                    status=TestStatus. FAILED,
                    message=f"Missing {len(missing_nodes)} nodes",
                    details="Missing: " + ", ".join(missing_nodes)
                )
                
        except Exception as e:
            return TestResult(
                name="Node Check",
                status=TestStatus. FAILED,
                message=f"Error checking nodes: {e}",
            )
    
    def check_topics(self, timeout: float = 5.0) -> TestResult:
        """Check if expected topics are being published."""
        # Wait for topics
        start_time = time.time()
        while time.time() - start_time < timeout: 
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if all(self.topic_received.values()):
                break
        
        received = sum(1 for v in self.topic_received.values() if v)
        total = len(self.expected_topics)
        
        missing = [t for t, r in self.topic_received.items() if not r]
        
        if received == total:
            return TestResult(
                name="Topic Check",
                status=TestStatus. PASSED,
                message=f"All {total} topics active",
            )
        elif received > total / 2:
            return TestResult(
                name="Topic Check",
                status=TestStatus.WARNING,
                message=f"{received}/{total} topics active",
                details="Missing: " + ", ".join(missing)
            )
        else:
            return TestResult(
                name="Topic Check",
                status=TestStatus.FAILED,
                message=f"Only {received}/{total} topics active",
                details="Missing: " + ", ".join(missing)
            )
    
    def check_services(self) -> TestResult:
        """Check if expected services are available."""
        try:
            result = subprocess. run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            available_services = result.stdout.strip().split('\n')
            missing_services = []
            
            for service in self.expected_services:
                if service not in available_services:
                    missing_services.append(service)
            
            if not missing_services: 
                return TestResult(
                    name="Service Check",
                    status=TestStatus.PASSED,
                    message=f"All {len(self.expected_services)} services available",
                )
            else:
                return TestResult(
                    name="Service Check",
                    status=TestStatus.WARNING,
                    message=f"Missing {len(missing_services)} services",
                    details="Missing: " + ", ".join(missing_services)
                )
                
        except Exception as e: 
            return TestResult(
                name="Service Check",
                status=TestStatus.FAILED,
                message=f"Error checking services:  {e}",
            )
    
    def check_tf(self) -> TestResult:
        """Check TF tree."""
        try:
            result = subprocess.run(
                ['ros2', 'run', 'tf2_ros', 'tf2_echo', 'base_link', 'odom'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if "Transform" in result.stdout or result.returncode == 0:
                return TestResult(
                    name="TF Check",
                    status=TestStatus.PASSED,
                    message="TF tree valid",
                )
            else:
                return TestResult(
                    name="TF Check",
                    status=TestStatus.WARNING,
                    message="TF may not be complete",
                    details=result.stderr[: 200] if result.stderr else ""
                )
                
        except subprocess.TimeoutExpired:
            return TestResult(
                name="TF Check",
                status=TestStatus.WARNING,
                message="TF check timed out",
            )
        except Exception as e:
            return TestResult(
                name="TF Check",
                status=TestStatus.FAILED,
                message=f"Error checking TF:  {e}",
            )
    
    def check_topic_rates(self, duration: float = 3.0) -> TestResult:
        """Check topic publish rates."""
        # Reset counts
        self.topic_counts = {t: 0 for t in self.expected_topics}
        
        # Collect data
        start_time = time. time()
        while time.time() - start_time < duration: 
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calculate rates
        rates = {}
        for topic, count in self.topic_counts. items():
            rates[topic] = count / duration
        
        # Check minimum rates
        min_rates = {
            '/imu/data': 50.0,
            '/odom': 20.0,
            '/camera/image_raw': 10.0,
            '/tof/depth/image_raw': 10.0,
        }
        
        issues = []
        for topic, min_rate in min_rates.items():
            if topic in rates and rates[topic] < min_rate:
                issues.append(f"{topic}: {rates[topic]:.1f} Hz (min: {min_rate})")
        
        if not issues:
            return TestResult(
                name="Rate Check",
                status=TestStatus.PASSED,
                message="All topic rates acceptable",
            )
        else:
            return TestResult(
                name="Rate Check",
                status=TestStatus.WARNING,
                message=f"{len(issues)} topics below minimum rate",
                details="\n".join(issues)
            )
    
    def run_all_checks(self) -> List[TestResult]:
        """Run all system checks."""
        self.results = []
        
        print("\n" + "="*60)
        print("  PHASE 1 SYSTEM CHECK")
        print("="*60 + "\n")
        
        # Run checks
        print("Running node check...")
        self.results.append(self.check_nodes())
        
        print("Running topic check...")
        self.results.append(self.check_topics())
        
        print("Running service check...")
        self.results.append(self.check_services())
        
        print("Running TF check...")
        self.results.append(self. check_tf())
        
        print("Running rate check...")
        self.results.append(self. check_topic_rates())
        
        return self.results
    
    def print_results(self):
        """Print test results."""
        print("\n" + "="*60)
        print("  RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            status_icon = result.status.value
            print(f"{status_icon} {result.name}:  {result.message}")
            
            if result.details:
                for line in result.details.split('\n'):
                    print(f"    {line}")
            
            if result.status == TestStatus. PASSED:
                passed += 1
            elif result.status == TestStatus.FAILED:
                failed += 1
            elif result. status == TestStatus.WARNING: 
                warnings += 1
        
        print("\n" + "-"*60)
        print(f"Summary: {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ PHASE 1 SYSTEM CHECK PASSED\n")
            return True
        else:
            print("\n❌ PHASE 1 SYSTEM CHECK FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase1SystemCheck()
    
    try:
        node.run_all_checks()
        success = node.print_results()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
