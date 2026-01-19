#!/usr/bin/env python3
"""
Waypoint Follower Node.

Loads waypoints from file and sends them to Nav2 for execution.
Supports following waypoints recorded by this rover or another rover.

Author: Rover Team
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import FollowWaypoints, NavigateToPose

from rover_interfaces.msg import WaypointList

from waypoint_manager import WaypointStorage


class WaypointFollowerNode(Node):
    """Waypoint follower node."""
    
    def __init__(self):
        super().__init__('waypoint_follower_node')
        
        self. declare_parameter('storage_directory', '~/.ros/waypoints')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('reverse_on_complete', False)
        
        self.storage = WaypointStorage(
            storage_dir=self.get_parameter('storage_directory').value
        )
        
        self.waypoints = []
        self.current_index = 0
        self.following = False
        self.reverse_direction = False
        
        self. follow_action = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.status_pub = self.create_publisher(String, '/waypoint_follower/status', 10)
        
        self.load_srv = self.create_service(Trigger, '/waypoint_follower/load', self._load_callback)
        self.start_srv = self. create_service(Trigger, '/waypoint_follower/start', self._start_callback)
        self.stop_srv = self. create_service(Trigger, '/waypoint_follower/stop', self._stop_callback)
        self.next_srv = self.create_service(Trigger, '/waypoint_follower/next', self._next_callback)
        
        self.waypoint_sub = self.create_subscription(
            WaypointList, '/recorded_waypoints', self._waypoints_callback, 10)
        
        auto_load = self.get_parameter('waypoint_file').value
        if auto_load:
            self._load_waypoints(auto_load)
        
        self.get_logger().info('Waypoint Follower Node started')
    
    def _load_waypoints(self, filename: str) -> bool:
        """Load waypoints from file."""
        waypoint_data = self.storage.load_yaml(filename)
        if waypoint_data is None:
            waypoint_data = self.storage.load_json(filename)
        
        if waypoint_data is None: 
            self.get_logger().error(f'Failed to load waypoints from {filename}')
            return False
        
        self.waypoints = []
        for wp in waypoint_data.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp. x
            pose.pose.position.y = wp.y
            pose.pose.position.z = wp.z
            pose.pose.orientation.x = wp.qx
            pose.pose.orientation.y = wp.qy
            pose.pose.orientation.z = wp.qz
            pose.pose.orientation.w = wp.qw
            self.waypoints.append(pose)
        
        self.current_index = 0
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from mission {waypoint_data.mission_id}')
        
        return True
    
    def _waypoints_callback(self, msg:  WaypointList):
        """Receive waypoints from recorder."""
        self.waypoints = []
        for wp in msg.waypoints:
            self.waypoints.append(wp. pose)
        
        self. get_logger().info(f'Received {len(self.waypoints)} waypoints')
    
    def _load_callback(self, request, response):
        """Load waypoints service."""
        filename = self.get_parameter('waypoint_file').value
        if not filename:
            files = self.storage.list_files()
            if files:
                filename = files[-1]
            else:
                response.success = False
                response.message = "No waypoint files found"
                return response
        
        if self._load_waypoints(filename):
            response.success = True
            response.message = f"Loaded {len(self.waypoints)} waypoints"
        else:
            response. success = False
            response.message = "Failed to load waypoints"
        
        return response
    
    def _start_callback(self, request, response):
        """Start following waypoints."""
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints loaded"
            return response
        
        if not self.follow_action.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = "Nav2 action server not available"
            return response
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints
        
        self.follow_action.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        ).add_done_callback(self._goal_response_callback)
        
        self.following = True
        response.success = True
        response. message = f"Following {len(self.waypoints)} waypoints"
        self._publish_status("Following waypoints")
        
        return response
    
    def _stop_callback(self, request, response):
        """Stop following."""
        self.following = False
        self. follow_action._cancel_goal_async
        
        response.success = True
        response.message = "Stopped following waypoints"
        self._publish_status("Stopped")
        
        return response
    
    def _next_callback(self, request, response):
        """Go to next waypoint only."""
        if not self.waypoints or self.current_index >= len(self.waypoints):
            response.success = False
            response.message = "No more waypoints"
            return response
        
        if not self.nav_action.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = "Nav2 action server not available"
            return response
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.waypoints[self.current_index]
        
        self.nav_action. send_goal_async(goal_msg)
        
        response.success = True
        response. message = f"Navigating to waypoint {self.current_index}"
        self. current_index += 1
        
        return response
    
    def _feedback_callback(self, feedback_msg):
        """Handle follow waypoints feedback."""
        current_wp = feedback_msg.feedback.current_waypoint
        self._publish_status(f"At waypoint {current_wp}/{len(self.waypoints)}")
    
    def _goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future. result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        goal_handle.get_result_async().add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """Handle result."""
        result = future.result().result
        missed = result.missed_waypoints
        
        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
            self._publish_status(f"Completed with {len(missed)} missed")
        else:
            self. get_logger().info('All waypoints completed!')
            self._publish_status("Completed all waypoints")
        
        self.following = False
        
        if self.get_parameter('loop_waypoints').value:
            self._start_callback(None, type('obj', (object,), {'success': False, 'message': ''})())
    
    def _publish_status(self, status:  str):
        """Publish status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node. destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()