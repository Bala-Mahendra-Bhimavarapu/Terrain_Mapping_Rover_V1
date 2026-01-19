#!/usr/bin/env python3
"""
Beacon Homing Node - Fully autonomous navigation to Bluetooth beacon.

This node provides COMPLETE autonomous navigation to a Bluetooth beacon
WITHOUT requiring any prebuilt map or path planning. It uses:

1. Bluetooth RSSI for beacon direction estimation (gradient descent)
2. ToF depth camera for obstacle detection
3. Reactive obstacle avoidance (no map needed)
4. Signal gradient tracking to find beacon

The robot will:
- Spin/explore to find beacon signal
- Move toward improving signal strength
- Avoid obstacles reactively using depth camera
- Stop when within arrival distance of beacon

Topics:
    Subscribed:
        /bluetooth/beacons (BeaconArray) - Beacon detections
        /tof/depth/image_raw (Image) - Depth for obstacle detection
        /odom (Odometry) - Robot pose
    
    Published:
        /vex/cmd_vel (Twist) - Velocity commands
        /beacon_homing/status (String) - Status messages
        /beacon_homing/state (BeaconHomingState) - Detailed state

Services:
    /beacon_homing/start (SetBool) - Start/stop homing
    /beacon_homing/set_target (SetString) - Set target beacon UUID

Author:  Rover Team
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rover_interfaces.msg import BeaconArray, BeaconHomingState

from cv_bridge import CvBridge
import numpy as np
import math

from beacon_homing import BeaconTracker, GradientNavigator, ObstacleDetector


class BeaconHomingNode(Node):
    """Autonomous beacon homing node."""
    
    def __init__(self):
        super().__init__('beacon_homing_node')
        
        # === PARAMETERS ===
        self._declare_parameters()
        
        self. target_uuid = self.get_parameter('target_uuid').value
        self.arrival_distance = self.get_parameter('arrival_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self. lost_timeout = self.get_parameter('lost_timeout').value
        
        # === COMPONENTS ===
        self.tracker = BeaconTracker(
            arrival_distance=self.arrival_distance,
            lost_timeout=self.lost_timeout
        )
        
        self. navigator = GradientNavigator(
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel,
            arrival_distance=self.arrival_distance,
            lost_timeout=self.lost_timeout,
            obstacle_stop_dist=self.obstacle_threshold
        )
        
        self.obstacle_detector = ObstacleDetector(
            obstacle_threshold=self.obstacle_threshold,
            max_range=self.get_parameter('max_detection_range').value
        )
        
        self.bridge = CvBridge()
        
        # === STATE ===
        self.enabled = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.latest_depth = None
        self.latest_beacon_rssi = None
        self.latest_beacon_distance = None
        
        # === QOS ===
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # === SUBSCRIBERS ===
        self.beacon_sub = self.create_subscription(
            BeaconArray, '/bluetooth/beacons',
            self._beacon_callback, 10
        )
        
        self.depth_sub = self.create_subscription(
            Image, '/tof/depth/image_raw',
            self._depth_callback, sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self._odom_callback, sensor_qos
        )
        
        # === PUBLISHERS ===
        self.cmd_pub = self.create_publisher(Twist, '/vex/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/beacon_homing/status', 10)
        self.state_pub = self.create_publisher(BeaconHomingState, '/beacon_homing/state', 10)
        
        # === SERVICES ===
        self.start_srv = self.create_service(
            SetBool, '/beacon_homing/start',
            self._start_callback
        )
        
        # === MAIN LOOP TIMER ===
        self.timer = self.create_timer(0.1, self._control_loop)  # 10Hz
        
        # === STARTUP ===
        self.get_logger().info('='*60)
        self.get_logger().info('  BEACON HOMING NODE')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('  Fully autonomous navigation to Bluetooth beacon')
        self.get_logger().info('  NO prebuilt map required!')
        self.get_logger().info('')
        self.get_logger().info('  Start with: ')
        self.get_logger().info('    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"')
        self.get_logger().info('')
        self.get_logger().info('  Stop with:')
        self.get_logger().info('    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data:  false}"')
        self.get_logger().info('')
        self.get_logger().info(f'  Target beacon: {self.target_uuid or "ANY"}')
        self.get_logger().info(f'  Arrival distance: {self.arrival_distance}m')
        self.get_logger().info('='*60)
    
    def _declare_parameters(self):
        """Declare all parameters."""
        self.declare_parameter('target_uuid', '')
        self.declare_parameter('arrival_distance', 0.5)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('obstacle_threshold', 0.4)
        self.declare_parameter('max_detection_range', 2.5)
        self.declare_parameter('lost_timeout', 30.0)
        self.declare_parameter('depth_scale', 0.001)
    
    def _beacon_callback(self, msg: BeaconArray):
        """Handle beacon detections."""
        self. latest_beacon_rssi = None
        self.latest_beacon_distance = None
        
        for beacon in msg.beacons:
            # Filter by target UUID if specified
            if self.target_uuid and beacon.uuid != self.target_uuid:
                continue
            
            self.latest_beacon_rssi = beacon.rssi
            self. latest_beacon_distance = beacon. distance
            break
    
    def _depth_callback(self, msg: Image):
        """Handle depth image."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth:  {e}')
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry."""
        self.current_x = msg.pose.pose.position. x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q. y + q.z * q. z)
        )
    
    def _start_callback(self, request, response):
        """Handle start/stop service."""
        self.enabled = request.data
        
        if self.enabled:
            self.tracker.reset()
            self.navigator.reset()
            self.get_logger().info('Beacon homing STARTED')
            response.message = "Beacon homing started"
        else:
            # Stop robot
            self._stop_robot()
            self.get_logger().info('Beacon homing STOPPED')
            response.message = "Beacon homing stopped"
        
        response.success = True
        return response
    
    def _control_loop(self):
        """Main control loop at 10Hz."""
        if not self.enabled:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # === UPDATE BEACON TRACKER ===
        beacon_state = self.tracker.update(
            rssi=self.latest_beacon_rssi,
            distance=self.latest_beacon_distance,
            robot_x=self.current_x,
            robot_y=self.current_y,
            robot_yaw=self.current_yaw,
            current_time=current_time
        )
        
        # === DETECT OBSTACLES ===
        if self.latest_depth is not None:
            depth_scale = self.get_parameter('depth_scale').value
            obstacle_info = self.obstacle_detector.detect_from_depth_image(
                self.latest_depth, depth_scale
            )
        else:
            # No depth data - assume clear
            from beacon_homing.obstacle_detector import ObstacleInfo
            obstacle_info = ObstacleInfo(
                front_blocked=False, front_distance=10.0,
                left_blocked=False, left_distance=10.0,
                right_blocked=False, right_distance=10.0,
                best_direction=0.0, obstacle_count=0
            )
        
        # === COMPUTE NAVIGATION COMMAND ===
        command = self. navigator.compute_command(beacon_state, obstacle_info)
        
        # === EXECUTE COMMAND ===
        twist = Twist()
        twist.linear. x = command.linear_velocity
        twist.angular.z = command.angular_velocity
        self. cmd_pub.publish(twist)
        
        # === PUBLISH STATUS ===
        status_msg = String()
        status_msg.data = f"[{command.state}] {command.message}"
        self.status_pub.publish(status_msg)
        
        # === PUBLISH DETAILED STATE ===
        state_msg = BeaconHomingState()
        state_msg.header. stamp = self.get_clock().now().to_msg()
        state_msg.enabled = self.enabled
        state_msg.state = command.state
        state_msg.beacon_detected = beacon_state.detected
        state_msg.beacon_distance = beacon_state.distance
        state_msg.beacon_rssi = beacon_state.signal_strength
        state_msg.signal_improving = beacon_state.signal_improving
        state_msg. estimated_direction = beacon_state.estimated_direction
        state_msg.confidence = beacon_state.confidence
        state_msg.obstacle_detected = obstacle_info.front_blocked
        state_msg. obstacle_distance = obstacle_info.front_distance
        state_msg.linear_velocity = command.linear_velocity
        state_msg.angular_velocity = command.angular_velocity
        state_msg.message = command.message
        self.state_pub.publish(state_msg)
        
        # === LOG STATE CHANGES ===
        self._log_state(command, beacon_state)
        
        # === CHECK FOR COMPLETION ===
        if command.state == "ARRIVED":
            self.get_logger().info('🎉 ARRIVED AT BEACON!')
            self.enabled = False
            self._stop_robot()
        elif command.state == "LOST":
            self.get_logger().warn('⚠️ Beacon signal lost!')
            self.enabled = False
            self._stop_robot()
    
    def _stop_robot(self):
        """Send stop command."""
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def _log_state(self, command, beacon_state):
        """Log state changes."""
        # Only log occasionally to avoid spam
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
            self._last_state = ""
        
        self._log_counter += 1
        
        # Log every 20 cycles (~2 seconds) or on state change
        if command.state != self._last_state: 
            self. get_logger().info(f'State:  {command.state} - {command.message}')
            self._last_state = command. state
        elif self._log_counter >= 20:
            self._log_counter = 0
            if beacon_state.detected:
                self.get_logger().info(
                    f'{command.state}:  dist={beacon_state.distance:.2f}m, '
                    f'rssi={beacon_state.signal_strength}dBm, '
                    f'conf={beacon_state.confidence:.0%}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = BeaconHomingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        # Ensure robot stops
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()