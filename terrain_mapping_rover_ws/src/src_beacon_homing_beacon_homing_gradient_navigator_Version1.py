"""
Gradient Navigator - Navigates toward beacon using signal gradient.

Combines beacon tracking with obstacle avoidance for autonomous homing.
"""

import math
from typing import Tuple, Optional
from dataclasses import dataclass

from .beacon_tracker import BeaconState
from .obstacle_detector import ObstacleInfo


@dataclass
class NavigationCommand:
    """Navigation command output."""
    linear_velocity: float   # m/s
    angular_velocity: float  # rad/s
    state: str              # SEARCHING, APPROACHING, AVOIDING, ARRIVED, LOST
    message: str            # Human-readable status


class GradientNavigator: 
    """
    Navigates toward beacon using signal gradient descent
    with reactive obstacle avoidance.
    
    States:
    - SEARCHING: Spinning/exploring to find beacon signal
    - APPROACHING: Moving toward beacon
    - AVOIDING:  Obstacle detected, maneuvering around
    - ARRIVED: Within arrival distance
    - LOST:  Beacon signal lost for too long
    """
    
    def __init__(
        self,
        # Speed limits
        max_linear_vel: float = 0.2,
        max_angular_vel: float = 1.0,
        min_linear_vel: float = 0.05,
        
        # Navigation parameters
        approach_gain: float = 0.5,
        rotation_gain: float = 1.0,
        
        # Thresholds
        heading_threshold: float = 0.3,  # Radians
        arrival_distance: float = 0.5,
        lost_timeout: float = 30.0,
        
        # Obstacle avoidance
        obstacle_stop_dist: float = 0.3,
        obstacle_slow_dist: float = 0.6
    ):
        """Initialize gradient navigator."""
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.min_linear_vel = min_linear_vel
        
        self.approach_gain = approach_gain
        self.rotation_gain = rotation_gain
        
        self.heading_threshold = heading_threshold
        self.arrival_distance = arrival_distance
        self.lost_timeout = lost_timeout
        
        self.obstacle_stop_dist = obstacle_stop_dist
        self. obstacle_slow_dist = obstacle_slow_dist
        
        self._state = "SEARCHING"
        self._search_direction = 1  # 1 = CCW, -1 = CW
        self._avoid_direction = 0.0
    
    def compute_command(
        self,
        beacon_state: BeaconState,
        obstacle_info: ObstacleInfo,
        preferred_direction: Optional[float] = None
    ) -> NavigationCommand:
        """
        Compute navigation command based on beacon and obstacle state.
        
        Args:
            beacon_state: Current beacon tracking state
            obstacle_info: Current obstacle detection state
            preferred_direction: Optional preferred direction hint (radians)
            
        Returns:
            NavigationCommand with velocities and state
        """
        
        # === STATE:  ARRIVED ===
        if beacon_state.detected and beacon_state.distance < self. arrival_distance:
            self._state = "ARRIVED"
            return NavigationCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
                state="ARRIVED",
                message=f"Arrived at beacon!  Distance: {beacon_state.distance:.2f}m"
            )
        
        # === STATE: LOST ===
        if not beacon_state.detected and beacon_state.time_since_detection > self.lost_timeout:
            self._state = "LOST"
            return NavigationCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
                state="LOST",
                message=f"Beacon lost for {beacon_state.time_since_detection:.1f}s"
            )
        
        # === STATE: AVOIDING (obstacle in front) ===
        if obstacle_info.front_blocked:
            self._state = "AVOIDING"
            return self._compute_avoidance_command(obstacle_info, beacon_state)
        
        # === STATE: SEARCHING (no beacon detected) ===
        if not beacon_state.detected:
            self._state = "SEARCHING"
            return self._compute_search_command(obstacle_info)
        
        # === STATE:  APPROACHING (beacon detected, no obstacle) ===
        self._state = "APPROACHING"
        return self._compute_approach_command(beacon_state, obstacle_info)
    
    def _compute_search_command(self, obstacle_info: ObstacleInfo) -> NavigationCommand:
        """Compute command for searching behavior."""
        
        # Spin in place to scan for beacon
        # Occasionally move forward if clear
        
        if obstacle_info.front_blocked:
            # Just spin
            return NavigationCommand(
                linear_velocity=0.0,
                angular_velocity=self._search_direction * self.max_angular_vel * 0.5,
                state="SEARCHING",
                message="Searching for beacon (spinning)..."
            )
        else:
            # Spin slowly and creep forward
            return NavigationCommand(
                linear_velocity=self.min_linear_vel,
                angular_velocity=self._search_direction * self.max_angular_vel * 0.3,
                state="SEARCHING",
                message="Searching for beacon (exploring)..."
            )
    
    def _compute_approach_command(
        self,
        beacon_state: BeaconState,
        obstacle_info:  ObstacleInfo
    ) -> NavigationCommand:
        """Compute command for approaching beacon."""
        
        direction = beacon_state.estimated_direction
        confidence = beacon_state.confidence
        distance = beacon_state.distance
        
        # Angular velocity to turn toward beacon
        angular_vel = self. rotation_gain * direction
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # Linear velocity based on alignment and distance
        if abs(direction) > self.heading_threshold:
            # Need to turn more - reduce forward speed
            linear_vel = self.min_linear_vel
        else:
            # Reasonably aligned - move forward
            # Slow down as we get closer
            distance_factor = min(1.0, distance / 2.0)
            confidence_factor = 0.5 + 0.5 * confidence
            
            linear_vel = self.max_linear_vel * distance_factor * confidence_factor
            linear_vel = max(self.min_linear_vel, linear_vel)
        
        # Slow down if obstacle nearby (but not blocking)
        if obstacle_info. front_distance < self.obstacle_slow_dist:
            slow_factor = obstacle_info.front_distance / self.obstacle_slow_dist
            linear_vel *= slow_factor
        
        improving = "↑" if beacon_state.signal_improving else "→"
        
        return NavigationCommand(
            linear_velocity=linear_vel,
            angular_velocity=angular_vel,
            state="APPROACHING",
            message=f"Approaching beacon {improving} dist={distance:.2f}m conf={confidence:.0%}"
        )
    
    def _compute_avoidance_command(
        self,
        obstacle_info: ObstacleInfo,
        beacon_state: BeaconState
    ) -> NavigationCommand:
        """Compute command for obstacle avoidance."""
        
        # Determine avoidance direction
        # Prefer direction toward beacon if possible
        
        if beacon_state.detected and beacon_state.confidence > 0.5:
            # Try to go around obstacle toward beacon
            beacon_dir = beacon_state.estimated_direction
            
            if beacon_dir > 0 and not obstacle_info.left_blocked:
                # Beacon is left and left is clear
                self._avoid_direction = 0.7
            elif beacon_dir < 0 and not obstacle_info.right_blocked:
                # Beacon is right and right is clear
                self._avoid_direction = -0.7
            else:
                # Use obstacle detector suggestion
                self._avoid_direction = obstacle_info.best_direction
        else:
            # No beacon info, use obstacle detector
            self._avoid_direction = obstacle_info.best_direction
        
        # Very close - just rotate
        if obstacle_info.front_distance < self.obstacle_stop_dist:
            return NavigationCommand(
                linear_velocity=0.0,
                angular_velocity=self._avoid_direction * self.max_angular_vel,
                state="AVOIDING",
                message=f"Obstacle!  Rotating to avoid (dist={obstacle_info.front_distance:.2f}m)"
            )
        
        # Somewhat close - slow down and turn
        return NavigationCommand(
            linear_velocity=self.min_linear_vel,
            angular_velocity=self._avoid_direction * self. max_angular_vel * 0.7,
            state="AVOIDING",
            message=f"Obstacle ahead, maneuvering (dist={obstacle_info.front_distance:.2f}m)"
        )
    
    @property
    def state(self) -> str:
        """Get current navigation state."""
        return self._state
    
    def reset(self):
        """Reset navigator state."""
        self._state = "SEARCHING"
        self._search_direction = 1
        self._avoid_direction = 0.0