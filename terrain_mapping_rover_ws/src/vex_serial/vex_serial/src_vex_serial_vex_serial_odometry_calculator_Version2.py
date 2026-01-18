"""
Odometry calculator for differential drive robot.
Converts wheel encoder ticks to pose estimates.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class OdometryState:
    """Current odometry state."""
    x: float = 0.0           # Position X (meters)
    y: float = 0.0           # Position Y (meters)
    theta: float = 0.0       # Heading (radians)
    v_linear: float = 0.0    # Linear velocity (m/s)
    v_angular: float = 0.0   # Angular velocity (rad/s)
    v_left: float = 0.0      # Left wheel velocity (m/s)
    v_right: float = 0.0     # Right wheel velocity (m/s)


class OdometryCalculator: 
    """
    Calculates odometry for a differential drive robot.
    
    Uses the standard differential drive kinematic model. 
    Handles encoder tick wraparound and direction compensation.
    """
    
    def __init__(
        self,
        wheel_radius: float = 0.0508,      # 5.08 cm
        track_width: float = 0.2921,        # 29.21 cm
        ticks_per_revolution: int = 900,    # VEX V5 with 18: 1 cartridge
        invert_left:  bool = False,          # Invert left encoder
        invert_right: bool = True,          # Invert right encoder (mirrored motor)
    ):
        """
        Initialize odometry calculator.
        
        Args:
            wheel_radius:  Wheel radius in meters
            track_width: Distance between wheel centers in meters
            ticks_per_revolution:  Encoder ticks per wheel revolution
            invert_left:  Whether to invert left encoder direction
            invert_right: Whether to invert right encoder direction
        """
        self.wheel_radius = wheel_radius
        self. track_width = track_width
        self.ticks_per_rev = ticks_per_revolution
        self.invert_left = invert_left
        self. invert_right = invert_right
        
        # Derived constants
        self.meters_per_tick = (2.0 * math.pi * wheel_radius) / ticks_per_revolution
        
        # State
        self._state = OdometryState()
        self._prev_left_ticks:  int = 0
        self._prev_right_ticks: int = 0
        self._initialized = False
        self._prev_timestamp_ms:  int = 0
    
    @property
    def state(self) -> OdometryState: 
        """Get current odometry state."""
        return self._state
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset odometry to specified pose."""
        self._state = OdometryState(x=x, y=y, theta=theta)
        self._initialized = False
    
    def update(
        self,
        left_ticks: int,
        right_ticks: int,
        timestamp_ms: int
    ) -> OdometryState:
        """
        Update odometry with new encoder readings.
        
        Args:
            left_ticks:  Cumulative left encoder ticks
            right_ticks: Cumulative right encoder ticks
            timestamp_ms:  Timestamp in milliseconds
            
        Returns:
            Updated odometry state
        """
        # Initialize on first update
        if not self._initialized:
            self._prev_left_ticks = left_ticks
            self._prev_right_ticks = right_ticks
            self._prev_timestamp_ms = timestamp_ms
            self._initialized = True
            return self._state
        
        # Calculate tick deltas
        delta_left = left_ticks - self._prev_left_ticks
        delta_right = right_ticks - self._prev_right_ticks
        
        # Apply direction inversion BEFORE odometry math
        # This compensates for physically mirrored motors
        if self. invert_left:
            delta_left = -delta_left
        if self.invert_right:
            delta_right = -delta_right
        
        # Calculate time delta
        delta_time_ms = timestamp_ms - self._prev_timestamp_ms
        if delta_time_ms <= 0:
            delta_time_ms = 1  # Prevent division by zero
        delta_time_s = delta_time_ms / 1000.0
        
        # Convert ticks to meters
        delta_left_m = delta_left * self.meters_per_tick
        delta_right_m = delta_right * self.meters_per_tick
        
        # Differential drive kinematics
        # Distance traveled by robot center
        delta_distance = (delta_left_m + delta_right_m) / 2.0
        
        # Change in heading
        delta_theta = (delta_right_m - delta_left_m) / self.track_width
        
        # Update pose using midpoint integration
        # This is more accurate than simple Euler integration
        mid_theta = self._state.theta + delta_theta / 2.0
        
        self._state.x += delta_distance * math.cos(mid_theta)
        self._state.y += delta_distance * math.sin(mid_theta)
        self._state.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self._state.theta = self._normalize_angle(self._state.theta)
        
        # Calculate velocities
        self._state.v_left = delta_left_m / delta_time_s
        self._state.v_right = delta_right_m / delta_time_s
        self._state.v_linear = delta_distance / delta_time_s
        self._state.v_angular = delta_theta / delta_time_s
        
        # Store previous values
        self._prev_left_ticks = left_ticks
        self._prev_right_ticks = right_ticks
        self._prev_timestamp_ms = timestamp_ms
        
        return self._state
    
    def get_wheel_velocities_for_twist(
        self,
        linear:  float,
        angular: float
    ) -> Tuple[float, float]:
        """
        Convert twist command to wheel velocities.
        
        Args:
            linear:  Linear velocity (m/s)
            angular:  Angular velocity (rad/s)
            
        Returns:
            Tuple of (left_velocity, right_velocity) in m/s
        """
        # Differential drive inverse kinematics
        left_vel = linear - (angular * self.track_width / 2.0)
        right_vel = linear + (angular * self.track_width / 2.0)
        
        return left_vel, right_vel
    
    def get_twist_for_wheel_velocities(
        self,
        left_vel: float,
        right_vel: float
    ) -> Tuple[float, float]:
        """
        Convert wheel velocities to twist. 
        
        Args:
            left_vel: Left wheel velocity (m/s)
            right_vel: Right wheel velocity (m/s)
            
        Returns: 
            Tuple of (linear_velocity, angular_velocity)
        """
        linear = (left_vel + right_vel) / 2.0
        angular = (right_vel - left_vel) / self.track_width
        
        return linear, angular
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle