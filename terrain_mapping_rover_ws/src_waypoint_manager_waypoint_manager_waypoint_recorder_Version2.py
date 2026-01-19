"""
Waypoint recording logic. 

Records waypoints based on time, distance, or manual triggers.
"""

import math
from typing import List, Optional
from dataclasses import dataclass
from datetime import datetime

from .waypoint_storage import WaypointData


@dataclass
class RecorderConfig:
    """Recorder configuration."""
    time_interval_sec: float = 5.0
    distance_interval_m: float = 0.5
    angle_interval_rad: float = 0.5
    min_distance_between: float = 0.1


class WaypointRecorder: 
    """Records waypoints during rover operation."""
    
    def __init__(self, config: Optional[RecorderConfig] = None):
        """Initialize recorder."""
        self.config = config or RecorderConfig()
        self.waypoints: List[WaypointData] = []
        self._next_id = 0
        self._last_record_time = 0.0
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0
        self._recording = False
        self._mode = "AUTO"
    
    def start_recording(self, mode: str = "AUTO"):
        """Start recording waypoints."""
        self._recording = True
        self._mode = mode. upper()
        self.waypoints = []
        self._next_id = 0
    
    def stop_recording(self) -> List[WaypointData]:
        """Stop recording and return waypoints."""
        self._recording = False
        return self. waypoints
    
    def clear(self):
        """Clear all recorded waypoints."""
        self. waypoints = []
        self._next_id = 0
    
    def record_manual(self, x: float, y:  float, z: float,
                      qx: float, qy:  float, qz: float, qw: float,
                      label: str = "") -> WaypointData: 
        """Manually record a waypoint."""
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        waypoint = self._create_waypoint(x, y, z, qx, qy, qz, qw, yaw, "MANUAL", label)
        self.waypoints.append(waypoint)
        return waypoint
    
    def check_and_record(self, x: float, y: float, z: float,
                         qx: float, qy: float, qz: float, qw:  float,
                         current_time: float) -> Optional[WaypointData]:
        """Check if a waypoint should be recorded based on current mode."""
        if not self._recording:
            return None
        
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        should_record = False
        method = "AUTO"
        
        if self._mode == "TIME": 
            if current_time - self._last_record_time >= self. config.time_interval_sec:
                should_record = True
                method = "TIME"
        elif self._mode == "DISTANCE": 
            distance = math.sqrt((x - self._last_x)**2 + (y - self._last_y)**2)
            if distance >= self.config.distance_interval_m:
                should_record = True
                method = "DISTANCE"
        elif self._mode == "AUTO":
            time_ok = current_time - self._last_record_time >= self.config.time_interval_sec
            distance = math.sqrt((x - self._last_x)**2 + (y - self._last_y)**2)
            distance_ok = distance >= self. config.distance_interval_m
            angle_diff = abs(yaw - self._last_yaw)
            if angle_diff > math.pi:
                angle_diff = 2 * math. pi - angle_diff
            angle_ok = angle_diff >= self.config.angle_interval_rad
            
            if time_ok or distance_ok or angle_ok:
                should_record = True
                method = "AUTO"
        
        if should_record: 
            if self.waypoints:
                last = self.waypoints[-1]
                dist_to_last = math.sqrt((x - last. x)**2 + (y - last.y)**2)
                if dist_to_last < self.config.min_distance_between:
                    return None
            
            waypoint = self._create_waypoint(x, y, z, qx, qy, qz, qw, yaw, method)
            self.waypoints.append(waypoint)
            
            self._last_record_time = current_time
            self._last_x = x
            self._last_y = y
            self._last_yaw = yaw
            
            return waypoint
        
        return None
    
    def _create_waypoint(self, x: float, y: float, z: float,
                         qx: float, qy:  float, qz: float, qw: float,
                         yaw: float, method: str, label: str = "") -> WaypointData:
        """Create a new waypoint."""
        waypoint = WaypointData(
            id=self._next_id,
            x=x, y=y, z=z,
            qx=qx, qy=qy, qz=qz, qw=qw,
            yaw=yaw,
            label=label,
            timestamp=datetime.now().isoformat(),
            recording_method=method
        )
        self._next_id += 1
        return waypoint