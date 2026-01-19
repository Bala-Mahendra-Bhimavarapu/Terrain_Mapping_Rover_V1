#!/usr/bin/env python3
"""
Waypoint Recorder Node. 

Records waypoints during rover operation based on time, distance, or events. 
Saves waypoints to YAML/JSON files for later replay. 

Author:  Rover Team
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
import tf2_ros
from tf2_ros import TransformException

from rover_interfaces.msg import Waypoint, WaypointList

from waypoint_manager import WaypointStorage, WaypointRecorder
from waypoint_manager.waypoint_storage import WaypointListData, WaypointData
from waypoint_manager.waypoint_recorder import RecorderConfig

from datetime import datetime


class WaypointRecorderNode(Node):
    """Waypoint recorder node."""
    
    def __init__(self):
        super().__init__('waypoint_recorder_node')
        
        self._declare_parameters()
        
        config = RecorderConfig(
            time_interval_sec=self.get_parameter('time_interval_sec').value,
            distance_interval_m=self.get_parameter('distance_interval_m').value,
            angle_interval_rad=self.get_parameter('angle_interval_rad').value,
            min_distance_between=self.get_parameter('min_distance_between').value
        )
        
        self. recorder = WaypointRecorder(config)
        self.storage = WaypointStorage(
            storage_dir=self.get_parameter('storage_directory').value
        )
        
        self. map_frame = self.get_parameter('map_frame').value
        self. base_frame = self.get_parameter('base_frame').value
        self.rover_id = self.get_parameter('rover_id').value
        
        self.tf_buffer = tf2_ros. Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.waypoints_pub = self.create_publisher(WaypointList, '/recorded_waypoints', 10)
        self.status_pub = self.create_publisher(String, '/waypoint_recorder/status', 10)
        
        self.start_srv = self.create_service(SetBool, '/waypoint_recorder/start', self._start_callback)
        self.stop_srv = self.create_service(Trigger, '/waypoint_recorder/stop', self._stop_callback)
        self.save_srv = self.create_service(Trigger, '/waypoint_recorder/save', self._save_callback)
        self.record_srv = self.create_service(Trigger, '/waypoint_recorder/record_now', self._record_now_callback)
        
        self.timer = self.create_timer(0.1, self._timer_callback)
        
        self.recording = False
        self.current_mission_id = ""
        
        self.get_logger().info('Waypoint Recorder Node started')
    
    def _declare_parameters(self):
        """Declare parameters."""
        self.declare_parameter('time_interval_sec', 5.0)
        self.declare_parameter('distance_interval_m', 0.5)
        self.declare_parameter('angle_interval_rad', 0.5)
        self.declare_parameter('min_distance_between', 0.1)
        self.declare_parameter('storage_directory', '~/.ros/waypoints')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rover_id', 'rover_01')
        self.declare_parameter('auto_save', True)
    
    def _get_current_pose(self):
        """Get current pose from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return transform
        except TransformException as e:
            self.get_logger().debug(f'Could not get transform: {e}')
            return None
    
    def _timer_callback(self):
        """Periodic callback for auto-recording."""
        if not self.recording:
            return
        
        transform = self._get_current_pose()
        if transform is None: 
            return
        
        t = transform.transform.translation
        r = transform.transform.rotation
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        waypoint = self.recorder.check_and_record(
            t. x, t.y, t. z, r.x, r.y, r.z, r.w, current_time)
        
        if waypoint: 
            self.get_logger().info(f'Recorded waypoint {waypoint.id} at ({t.x:.2f}, {t.y:.2f})')
            self._publish_waypoints()
    
    def _start_callback(self, request, response):
        """Start recording service callback."""
        mode = "AUTO" if request.data else "MANUAL"
        self.recorder.start_recording(mode)
        self.recording = True
        self.current_mission_id = self.storage.generate_mission_id()
        
        response.success = True
        response.message = f"Recording started in {mode} mode, mission:  {self.current_mission_id}"
        self.get_logger().info(response.message)
        
        self._publish_status(f"Recording:  {mode}")
        return response
    
    def _stop_callback(self, request, response):
        """Stop recording service callback."""
        waypoints = self.recorder.stop_recording()
        self.recording = False
        
        if self.get_parameter('auto_save').value and waypoints:
            self._save_waypoints()
        
        response.success = True
        response.message = f"Recording stopped, {len(waypoints)} waypoints recorded"
        self.get_logger().info(response.message)
        
        self._publish_status("Stopped")
        return response
    
    def _save_callback(self, request, response):
        """Save waypoints service callback."""
        filepath = self._save_waypoints()
        
        if filepath:
            response.success = True
            response.message = f"Waypoints saved to {filepath}"
        else:
            response.success = False
            response.message = "No waypoints to save"
        
        return response
    
    def _record_now_callback(self, request, response):
        """Manual record service callback."""
        transform = self._get_current_pose()
        if transform is None: 
            response.success = False
            response. message = "Could not get current pose"
            return response
        
        t = transform.transform.translation
        r = transform.transform.rotation
        
        waypoint = self.recorder. record_manual(t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        self._publish_waypoints()
        
        response.success = True
        response.message = f"Recorded waypoint {waypoint.id} at ({t.x:.2f}, {t.y:.2f})"
        self.get_logger().info(response.message)
        
        return response
    
    def _save_waypoints(self) -> str:
        """Save current waypoints to file."""
        if not self.recorder.waypoints:
            return ""
        
        waypoint_list = WaypointListData(
            mission_id=self.current_mission_id or self.storage.generate_mission_id(),
            created_at=datetime.now().isoformat(),
            created_by_rover_id=self.rover_id,
            waypoints=self.recorder.waypoints
        )
        
        filepath = self.storage. save_yaml(waypoint_list, waypoint_list.mission_id)
        self.get_logger().info(f'Saved {len(self.recorder.waypoints)} waypoints to {filepath}')
        
        return filepath
    
    def _publish_waypoints(self):
        """Publish current waypoints."""
        msg = WaypointList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self.current_mission_id
        
        for wp in self.recorder.waypoints:
            waypoint_msg = Waypoint()
            waypoint_msg.waypoint_id = wp.id
            waypoint_msg.pose.header.frame_id = self.map_frame
            waypoint_msg.pose.pose.position.x = wp.x
            waypoint_msg.pose.pose.position.y = wp.y
            waypoint_msg.pose.pose.position.z = wp.z
            waypoint_msg.pose.pose.orientation.x = wp.qx
            waypoint_msg.pose.pose. orientation.y = wp.qy
            waypoint_msg.pose.pose.orientation.z = wp.qz
            waypoint_msg.pose.pose.orientation. w = wp.qw
            waypoint_msg.label = wp.label
            waypoint_msg.recording_method = wp.recording_method
            msg.waypoints.append(waypoint_msg)
        
        self.waypoints_pub.publish(msg)
    
    def _publish_status(self, status: str):
        """Publish recorder status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()