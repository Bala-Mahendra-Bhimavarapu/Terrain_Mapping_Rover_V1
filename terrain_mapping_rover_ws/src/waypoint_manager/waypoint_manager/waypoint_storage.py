"""
Waypoint storage and file I/O.

Handles saving and loading waypoints to/from YAML and JSON files.
"""

import json
import yaml
import os
from typing import List, Optional
from dataclasses import dataclass, asdict
from datetime import datetime


@dataclass
class WaypointData:
    """Single waypoint data."""
    id: int
    x: float
    y: float
    z: float
    qx: float
    qy:  float
    qz: float
    qw: float
    yaw: float  # Convenience field
    label: str = ""
    timestamp: str = ""
    recording_method: str = "MANUAL"


@dataclass
class WaypointListData:
    """List of waypoints with metadata."""
    mission_id: str
    created_at: str
    created_by_rover_id: str
    waypoints: List[WaypointData]


class WaypointStorage: 
    """
    Handles waypoint storage operations.
    
    Supports YAML and JSON formats. 
    """
    
    def __init__(self, storage_dir: str = "~/.ros/waypoints"):
        """
        Initialize waypoint storage. 
        
        Args:
            storage_dir: Directory to store waypoint files
        """
        self. storage_dir = os.path.expanduser(storage_dir)
        os.makedirs(self.storage_dir, exist_ok=True)
    
    def save_yaml(self, waypoints: WaypointListData, filename:  str) -> str:
        """
        Save waypoints to YAML file.
        
        Args:
            waypoints:  Waypoint list data
            filename: File name (without extension)
            
        Returns:
            Full path to saved file
        """
        filepath = os.path.join(self.storage_dir, f"{filename}.yaml")
        
        data = asdict(waypoints)
        
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        return filepath
    
    def save_json(self, waypoints: WaypointListData, filename: str) -> str:
        """Save waypoints to JSON file."""
        filepath = os.path.join(self.storage_dir, f"{filename}.json")
        
        data = asdict(waypoints)
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        return filepath
    
    def load_yaml(self, filename: str) -> Optional[WaypointListData]: 
        """Load waypoints from YAML file."""
        filepath = os.path.join(self.storage_dir, f"{filename}.yaml")
        
        if not os.path.exists(filepath):
            # Try with full path
            filepath = filename if os.path.exists(filename) else None
        
        if not filepath or not os.path.exists(filepath):
            return None
        
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        
        return self._dict_to_waypoint_list(data)
    
    def load_json(self, filename: str) -> Optional[WaypointListData]:
        """Load waypoints from JSON file."""
        filepath = os.path.join(self.storage_dir, f"{filename}.json")
        
        if not os.path.exists(filepath):
            filepath = filename if os.path.exists(filename) else None
        
        if not filepath or not os. path.exists(filepath):
            return None
        
        with open(filepath, 'r') as f:
            data = json. load(f)
        
        return self._dict_to_waypoint_list(data)
    
    def list_files(self) -> List[str]:
        """List all waypoint files in storage directory."""
        files = []
        for f in os.listdir(self. storage_dir):
            if f.endswith('.yaml') or f.endswith('.json'):
                files.append(f)
        return sorted(files)
    
    def _dict_to_waypoint_list(self, data: dict) -> WaypointListData: 
        """Convert dictionary to WaypointListData."""
        waypoints = []
        for wp_data in data.get('waypoints', []):
            waypoints.append(WaypointData(**wp_data))
        
        return WaypointListData(
            mission_id=data.get('mission_id', ''),
            created_at=data.get('created_at', ''),
            created_by_rover_id=data.get('created_by_rover_id', ''),
            waypoints=waypoints
        )
    
    @staticmethod
    def generate_mission_id() -> str:
        """Generate unique mission ID."""
        return datetime.now().strftime("mission_%Y%m%d_%H%M%S")
