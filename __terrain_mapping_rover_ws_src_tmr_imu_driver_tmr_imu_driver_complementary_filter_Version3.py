"""
Complementary Filter for IMU Orientation Estimation

Combines accelerometer and gyroscope data to estimate orientation (roll and pitch).
Yaw cannot be estimated from accelerometer alone (requires magnetometer).

The complementary filter is a simple but effective sensor fusion technique: 
- High-pass filter on gyroscope (good for short-term, drifts long-term)
- Low-pass filter on accelerometer (good for long-term, noisy short-term)

filtered_angle = alpha * (filtered_angle + gyro_rate * dt) + (1 - alpha) * accel_angle
"""

import math
import time
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class Orientation:
    """Container for orientation data"""
    roll: float      # Rotation about X axis (rad)
    pitch: float     # Rotation about Y axis (rad)
    yaw: float       # Rotation about Z axis (rad) - only from gyro integration
    timestamp: float


class ComplementaryFilter:
    """
    Complementary filter for IMU orientation estimation.
    
    Fuses accelerometer and gyroscope data to estimate roll and pitch. 
    Yaw is integrated from gyroscope only (will drift without magnetometer).
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize complementary filter.
        
        Args:
            alpha: Filter coefficient (0-1). Higher values trust gyro more.
                   Typical values: 0.95-0.99
        """
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("Alpha must be between 0 and 1")
        
        self.alpha = alpha
        
        # Current orientation estimate
        self.roll = 0.0
        self. pitch = 0.0
        self.yaw = 0.0
        
        # Timing
        self.last_time:  Optional[float] = None
        
        # Initialized flag
        self.initialized = False
    
    def reset(self):
        """Reset filter state"""
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        self.initialized = False
    
    def _accel_to_angles(
        self, 
        accel_x: float, 
        accel_y: float, 
        accel_z: float
    ) -> Tuple[float, float]: 
        """
        Calculate roll and pitch from accelerometer data. 
        
        Assumes the sensor is relatively stationary (acceleration ≈ gravity).
        
        Args:
            accel_x: X acceleration (m/s^2)
            accel_y: Y acceleration (m/s^2)
            accel_z: Z acceleration (m/s^2)
        
        Returns:
            Tuple of (roll, pitch) in radians
        """
        # Roll: rotation about X axis
        # atan2(y, z) gives angle of gravity vector in YZ plane
        roll = math.atan2(accel_y, accel_z)
        
        # Pitch: rotation about Y axis
        # Need to account for roll to get accurate pitch
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_magnitude > 0.1:  # Avoid division by zero
            pitch = math.asin(-accel_x / accel_magnitude)
        else:
            pitch = 0.0
        
        return (roll, pitch)
    
    def update(
        self,
        accel_x: float,
        accel_y:  float,
        accel_z: float,
        gyro_x: float,
        gyro_y: float,
        gyro_z: float,
        timestamp: Optional[float] = None
    ) -> Orientation:
        """
        Update filter with new sensor data.
        
        Args:
            accel_x, accel_y, accel_z: Accelerometer readings (m/s^2)
            gyro_x, gyro_y, gyro_z:  Gyroscope readings (rad/s)
            timestamp: Optional timestamp (seconds). If None, uses current time. 
        
        Returns:
            Updated orientation estimate
        """
        current_time = timestamp if timestamp is not None else time. time()
        
        # Calculate angles from accelerometer
        accel_roll, accel_pitch = self._accel_to_angles(accel_x, accel_y, accel_z)
        
        if not self.initialized or self.last_time is None:
            # First update - initialize from accelerometer
            self.roll = accel_roll
            self.pitch = accel_pitch
            self.yaw = 0.0
            self.last_time = current_time
            self.initialized = True
        else:
            # Calculate time delta
            dt = current_time - self.last_time
            self.last_time = current_time
            
            # Limit dt to reasonable range (handle timer glitches)
            dt = max(0.001, min(dt, 0.1))
            
            # Integrate gyroscope for orientation change
            # Note: This is a simplified integration that works for small angles
            # For large rotations, quaternion integration would be more accurate
            gyro_roll = self.roll + gyro_x * dt
            gyro_pitch = self.pitch + gyro_y * dt
            gyro_yaw = self. yaw + gyro_z * dt
            
            # Apply complementary filter for roll and pitch
            self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll
            self. pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
            
            # Yaw is gyro-only (no accelerometer reference)
            self.yaw = gyro_yaw
            
            # Normalize yaw to [-pi, pi]
            while self.yaw > math.pi:
                self.yaw -= 2.0 * math. pi
            while self.yaw < -math.pi:
                self.yaw += 2.0 * math.pi
        
        return Orientation(
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
            timestamp=current_time
        )
    
    def get_orientation(self) -> Orientation:
        """
        Get current orientation estimate.
        
        Returns:
            Current orientation
        """
        return Orientation(
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
            timestamp=self.last_time if self.last_time else 0.0
        )
    
    def get_quaternion(self) -> Tuple[float, float, float, float]: 
        """
        Get orientation as quaternion (x, y, z, w).
        
        Returns:
            Quaternion representation of current orientation
        """
        # Convert Euler angles to quaternion
        # Using ZYX convention (yaw-pitch-roll)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def set_yaw(self, yaw: float):
        """
        Manually set yaw angle (useful for resetting drift).
        
        Args:
            yaw: New yaw angle in radians
        """
        self.yaw = yaw


class MovingAverageFilter:
    """
    Simple moving average filter for smoothing sensor data.
    """
    
    def __init__(self, window_size: int = 5):
        """
        Initialize moving average filter.
        
        Args:
            window_size: Number of samples to average
        """
        self.window_size = window_size
        self.buffer = []
    
    def reset(self):
        """Clear filter buffer"""
        self.buffer = []
    
    def update(self, value: float) -> float:
        """
        Add new value and return filtered result.
        
        Args:
            value: New sample
        
        Returns:
            Filtered value (average of buffer)
        """
        self.buffer.append(value)
        
        if len(self. buffer) > self.window_size:
            self.buffer.pop(0)
        
        return sum(self.buffer) / len(self.buffer)


class Vector3Filter:
    """
    Moving average filter for 3D vectors. 
    """
    
    def __init__(self, window_size: int = 5):
        """
        Initialize 3D vector filter.
        
        Args:
            window_size: Number of samples to average
        """
        self.x_filter = MovingAverageFilter(window_size)
        self.y_filter = MovingAverageFilter(window_size)
        self.z_filter = MovingAverageFilter(window_size)
    
    def reset(self):
        """Clear filter buffers"""
        self.x_filter.reset()
        self.y_filter.reset()
        self.z_filter.reset()
    
    def update(self, x: float, y: float, z: float) -> Tuple[float, float, float]: 
        """
        Add new vector and return filtered result.
        
        Args:
            x, y, z: New sample components
        
        Returns:
            Filtered (x, y, z)
        """
        return (
            self.x_filter. update(x),
            self.y_filter.update(y),
            self.z_filter. update(z)
        )