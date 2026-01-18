"""
IMU filtering utilities. 

Provides a complementary filter for orientation estimation
and other signal processing utilities.
"""

import math
from typing import Tuple


class ComplementaryFilter:
    """
    Complementary filter for orientation estimation.
    
    Combines accelerometer and gyroscope data to estimate
    roll and pitch angles.  The filter blends: 
    - Short-term accuracy from gyroscope integration
    - Long-term stability from accelerometer (gravity reference)
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize filter. 
        
        Args:
            alpha: Filter coefficient (0-1). Higher values trust gyro more.
                   Typical values: 0.95-0.99
        """
        self.alpha = alpha
        self.roll = 0.0   # radians
        self.pitch = 0.0  # radians
        self._last_time = None
    
    def update(
        self,
        accel_x: float,
        accel_y: float,
        accel_z: float,
        gyro_x: float,
        gyro_y: float,
        timestamp: float
    ) -> Tuple[float, float]: 
        """
        Update filter with new sensor data.
        
        Args:
            accel_x, accel_y, accel_z: Accelerometer readings (m/s^2)
            gyro_x, gyro_y:  Gyroscope readings for roll/pitch (rad/s)
            timestamp: Current time in seconds
            
        Returns: 
            Tuple of (roll, pitch) in radians
        """
        # Calculate dt
        if self._last_time is None:
            self._last_time = timestamp
            # Initialize from accelerometer
            self.roll, self.pitch = self._accel_to_angles(accel_x, accel_y, accel_z)
            return self.roll, self.pitch
        
        dt = timestamp - self._last_time
        self._last_time = timestamp
        
        if dt <= 0 or dt > 1.0:
            return self.roll, self.pitch
        
        # Calculate angles from accelerometer
        accel_roll, accel_pitch = self._accel_to_angles(accel_x, accel_y, accel_z)
        
        # Integrate gyroscope
        gyro_roll = self. roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt
        
        # Apply complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        
        return self.roll, self. pitch
    
    def reset(self):
        """Reset filter state."""
        self.roll = 0.0
        self.pitch = 0.0
        self._last_time = None
    
    @staticmethod
    def _accel_to_angles(ax: float, ay: float, az: float) -> Tuple[float, float]:
        """
        Calculate roll and pitch from accelerometer data.
        
        Assumes the only acceleration is gravity. 
        """
        # Roll: rotation about X axis
        roll = math.atan2(ay, math.sqrt(ax * ax + az * az))
        
        # Pitch: rotation about Y axis
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        
        return roll, pitch
