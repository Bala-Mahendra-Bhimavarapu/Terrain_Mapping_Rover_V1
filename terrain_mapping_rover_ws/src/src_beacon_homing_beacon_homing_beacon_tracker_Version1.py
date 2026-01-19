"""
Beacon Tracker - Tracks beacon signal strength over time and position.

Uses signal gradient to determine direction to beacon.
"""

import math
import time
from typing import Optional, Tuple, List, Deque
from dataclasses import dataclass
from collections import deque
import numpy as np


@dataclass
class BeaconReading:
    """Single beacon reading with robot pose."""
    timestamp: float
    rssi: int
    distance: float
    robot_x: float
    robot_y: float
    robot_yaw: float


@dataclass
class BeaconState:
    """Current beacon tracking state."""
    detected: bool
    distance: float
    signal_strength: int
    signal_improving: bool
    estimated_direction: float  # Radians, in robot frame
    confidence: float  # 0-1
    time_since_detection: float


class BeaconTracker: 
    """
    Tracks Bluetooth beacon signal over time.
    
    Uses multiple readings at different positions to estimate
    the direction to the beacon using signal gradient descent.
    """
    
    def __init__(
        self,
        history_size: int = 50,
        gradient_window: int = 10,
        lost_timeout: float = 10.0,
        arrival_distance: float = 0.5
    ):
        """
        Initialize beacon tracker.
        
        Args:
            history_size: Max readings to keep in history
            gradient_window:  Readings to use for gradient calculation
            lost_timeout: Seconds before beacon considered lost
            arrival_distance: Distance to consider "arrived" (meters)
        """
        self. history_size = history_size
        self.gradient_window = gradient_window
        self.lost_timeout = lost_timeout
        self.arrival_distance = arrival_distance
        
        self.readings: Deque[BeaconReading] = deque(maxlen=history_size)
        self.last_detection_time = 0.0
        self._estimated_direction = 0.0
        self._confidence = 0.0
    
    def update(
        self,
        rssi: Optional[int],
        distance: Optional[float],
        robot_x: float,
        robot_y: float,
        robot_yaw:  float,
        current_time: float
    ) -> BeaconState:
        """
        Update tracker with new beacon reading.
        
        Args:
            rssi:  Signal strength (None if not detected)
            distance:  Estimated distance (None if not detected)
            robot_x, robot_y: Current robot position
            robot_yaw: Current robot heading (radians)
            current_time: Current timestamp
            
        Returns:
            BeaconState with current tracking status
        """
        detected = rssi is not None and distance is not None
        
        if detected:
            self.last_detection_time = current_time
            
            # Add reading to history
            reading = BeaconReading(
                timestamp=current_time,
                rssi=rssi,
                distance=distance,
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw
            )
            self.readings.append(reading)
            
            # Calculate signal gradient direction
            self._update_gradient_estimate()
            
            # Check if signal is improving
            signal_improving = self._is_signal_improving()
            
            return BeaconState(
                detected=True,
                distance=distance,
                signal_strength=rssi,
                signal_improving=signal_improving,
                estimated_direction=self._estimated_direction,
                confidence=self._confidence,
                time_since_detection=0.0
            )
        else:
            # Beacon not detected
            time_since = current_time - self.last_detection_time
            
            return BeaconState(
                detected=False,
                distance=-1.0,
                signal_strength=0,
                signal_improving=False,
                estimated_direction=self._estimated_direction,
                confidence=max(0, self._confidence - 0.1),
                time_since_detection=time_since
            )
    
    def _update_gradient_estimate(self):
        """
        Estimate direction to beacon using signal gradient. 
        
        Uses recent readings at different positions to determine
        which direction has improving signal strength.
        """
        if len(self.readings) < 3:
            self._confidence = 0.2
            return
        
        # Get recent readings
        recent = list(self.readings)[-self.gradient_window:]
        
        if len(recent) < 3:
            return
        
        # Calculate weighted gradient
        # Points with stronger signal get higher weight
        positions = []
        weights = []
        
        min_rssi = min(r. rssi for r in recent)
        max_rssi = max(r.rssi for r in recent)
        rssi_range = max(1, max_rssi - min_rssi)
        
        for reading in recent:
            positions.append((reading.robot_x, reading. robot_y))
            # Normalize RSSI to 0-1 weight (stronger = higher weight)
            weight = (reading.rssi - min_rssi) / rssi_range
            weights.append(weight)
        
        # Calculate weighted centroid (beacon is likely in direction of stronger signals)
        total_weight = sum(weights)
        if total_weight < 0.1:
            return
        
        centroid_x = sum(p[0] * w for p, w in zip(positions, weights)) / total_weight
        centroid_y = sum(p[1] * w for p, w in zip(positions, weights)) / total_weight
        
        # Current position
        current = recent[-1]
        
        # Direction from current position to weighted centroid
        dx = centroid_x - current. robot_x
        dy = centroid_y - current.robot_y
        
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            # Haven't moved enough
            self._confidence = max(0.3, self._confidence - 0.1)
            return
        
        # Estimated direction in world frame
        world_direction = math.atan2(dy, dx)
        
        # Convert to robot frame
        self._estimated_direction = self._normalize_angle(world_direction - current.robot_yaw)
        
        # Confidence based on consistency and signal spread
        self._confidence = min(0.9, 0.4 + (rssi_range / 20.0))
    
    def _is_signal_improving(self) -> bool:
        """Check if signal strength is improving over recent readings."""
        if len(self.readings) < 5:
            return False
        
        recent = list(self.readings)[-10:]
        
        if len(recent) < 5:
            return False
        
        # Compare first half to second half
        mid = len(recent) // 2
        first_half_avg = sum(r.rssi for r in recent[: mid]) / mid
        second_half_avg = sum(r. rssi for r in recent[mid:]) / (len(recent) - mid)
        
        # Signal improving if second half is stronger (less negative)
        return second_half_avg > first_half_avg + 1
    
    def get_best_direction(self) -> Tuple[float, float]:
        """
        Get the estimated direction to beacon.
        
        Returns:
            Tuple of (direction_radians, confidence)
        """
        return self._estimated_direction, self._confidence
    
    def is_arrived(self) -> bool:
        """Check if we've arrived at the beacon."""
        if not self.readings:
            return False
        
        latest = self.readings[-1]
        return latest.distance < self.arrival_distance
    
    def reset(self):
        """Reset tracker state."""
        self.readings.clear()
        self._estimated_direction = 0.0
        self._confidence = 0.0
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math. pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle