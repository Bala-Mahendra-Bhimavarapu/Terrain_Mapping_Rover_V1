#!/usr/bin/env python3
"""
Mission Health Monitor Node.

Monitors all rover subsystems and publishes overall health status. 

Author: Rover Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from rover_interfaces.msg import (
    MissionHealth, MissionEvent, VexStatus,
    ClassificationArray, LandmarkVerification
)

from mission_health import HealthMonitor
from mission_health.health_monitor import HealthStatus
from mission_health.subsystem_monitor import (
    SLAMMonitor, LocalizationMonitor, MotorMonitor
)


class MissionHealthNode(Node):
    """Mission health monitoring node."""
    
    def __init__(self):
        super().__init__('mission_health_node')
        
        self._declare_parameters()
        
        # Initialize health monitor
        self. health_monitor = HealthMonitor(
            timeout_sec=self.get_parameter('subsystem_timeout').value
        )
        
        # Register subsystems
        for name in ['slam', 'localization', 'navigation', 'perception', 'motors']:
            self.health_monitor.register_subsystem(name)
        
        # Initialize subsystem monitors
        self.slam_monitor = SLAMMonitor(
            min_features=self.get_parameter('min_slam_features').value
        )
        self.loc_monitor = LocalizationMonitor(
            max_position_cov=self.get_parameter('max_position_covariance').value
        )
        self.motor_monitor = MotorMonitor(
            min_battery_voltage=self.get_parameter('min_battery_voltage').value
        )
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, sensor_qos)
        self.vex_status_sub = self. create_subscription(
            VexStatus, '/vex/status', self._vex_status_callback, 10)
        self.classification_sub = self.create_subscription(
            ClassificationArray, '/ai_camera/classification',
            self._classification_callback, 10)
        self.landmark_verif_sub = self.create_subscription(
            LandmarkVerification, '/landmarks/verification',
            self._landmark_callback, 10)
        
        # Publishers
        self.health_pub = self.create_publisher(MissionHealth, '/mission_health', 10)
        self.event_pub = self.create_publisher(MissionEvent, '/mission_events', 10)
        
        # Timer for periodic health checks
        rate = self.get_parameter('update_rate').value
        self. timer = self.create_timer(1.0 / rate, self._health_check_callback)
        
        self.last_status = HealthStatus.OK
        
        self.get_logger().info('Mission Health Monitor started')
    
    def _declare_parameters(self):
        """Declare parameters."""
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('subsystem_timeout', 2.0)
        self.declare_parameter('min_slam_features', 50)
        self.declare_parameter('max_position_covariance', 0.5)
        self.declare_parameter('min_battery_voltage', 10.5)
    
    def _odom_callback(self, msg:  Odometry):
        """Handle odometry for localization health."""
        self.loc_monitor.update_from_covariance(list(msg.pose.covariance))
        status, message, metrics = self.loc_monitor.check_health()
        self.health_monitor.update_subsystem('localization', status, message, metrics)
    
    def _vex_status_callback(self, msg: VexStatus):
        """Handle VEX status for motor health."""
        self.motor_monitor.update(
            connected=msg.connected,
            battery_voltage=msg.battery_voltage,
            temperatures=list(msg.motor_temperatures)
        )
        status, message, metrics = self. motor_monitor.check_health()
        self.health_monitor. update_subsystem('motors', status, message, metrics)
    
    def _classification_callback(self, msg: ClassificationArray):
        """Handle classification for perception health."""
        if len(msg.classifications) > 0:
            avg_conf = sum(c.confidence for c in msg.classifications) / len(msg.classifications)
            if avg_conf > 0.7:
                status = HealthStatus.OK
            elif avg_conf > 0.4:
                status = HealthStatus.DEGRADED
            else:
                status = HealthStatus. CRITICAL
            
            self.health_monitor. update_subsystem(
                'perception', status, 
                f"Avg confidence: {avg_conf:.2f}",
                {'avg_confidence': avg_conf}
            )
    
    def _landmark_callback(self, msg:  LandmarkVerification):
        """Handle landmark verification."""
        if msg.status == 'CRITICAL':
            self._publish_event('WARNING', 'LANDMARK_DRIFT',
                f"Landmark {msg.label} position error: {msg.error_distance:.2f}m")
    
    def _health_check_callback(self):
        """Periodic health check and publish."""
        state = self.health_monitor.compute_overall_health()
        
        # Publish health message
        msg = MissionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = state. status. value
        msg.slam_status = state.slam_status.value
        msg.localization_status = state.localization_status.value
        msg.navigation_status = state.navigation_status.value
        msg.perception_status = state.perception_status. value
        msg.motor_status = state.motor_status.value
        msg.slam_quality = state.slam_quality
        msg.localization_confidence = state.localization_confidence
        msg.vo_feature_count = state.vo_feature_count
        msg. loop_closure_count = state. loop_closure_count
        msg.position_uncertainty = state.position_uncertainty
        msg.orientation_uncertainty = state.orientation_uncertainty
        
        self.health_pub.publish(msg)
        
        # Publish event if status changed
        if state.status != self.last_status:
            severity = 'ERROR' if state.status == HealthStatus.CRITICAL else 'WARNING'
            self._publish_event(severity, 'STATUS_CHANGE',
                f"Mission status changed:  {self.last_status.value} -> {state.status.value}")
            self.last_status = state. status
    
    def _publish_event(self, severity: str, event_type: str, message: str):
        """Publish a mission event."""
        event = MissionEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.severity = severity
        event.event_type = event_type
        event.message = message
        self.event_pub.publish(event)
        
        # Also log it
        if severity == 'ERROR' or severity == 'CRITICAL':
            self.get_logger().error(f"[{event_type}] {message}")
        elif severity == 'WARNING':
            self.get_logger().warn(f"[{event_type}] {message}")
        else:
            self.get_logger().info(f"[{event_type}] {message}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()