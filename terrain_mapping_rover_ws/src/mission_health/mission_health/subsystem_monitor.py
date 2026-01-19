"""
Subsystem-specific health monitoring.

Provides health checking logic for each subsystem type.
"""

from typing import Dict, Optional, Tuple
from . health_monitor import HealthStatus
import numpy as np


class SubsystemMonitor:
    """Base class for subsystem monitors."""
    
    def __init__(self, name: str):
        self.name = name
        self. status = HealthStatus.OFFLINE
        self.message = ""
        self.metrics: Dict[str, float] = {}
    
    def check_health(self) -> Tuple[HealthStatus, str, Dict[str, float]]:
        """Check health and return status, message, metrics."""
        raise NotImplementedError


class SLAMMonitor(SubsystemMonitor):
    """Monitor for SLAM subsystem."""
    
    def __init__(
        self,
        min_features: int = 50,
        min_loop_closures: int = 0,
        max_reproj_error: float = 2.0
    ):
        super().__init__('slam')
        self.min_features = min_features
        self.min_loop_closures = min_loop_closures
        self.max_reproj_error = max_reproj_error
        
        self.feature_count = 0
        self.loop_closures = 0
        self.reproj_error = 0.0
        self.quality = 0.0
    
    def update(
        self,
        feature_count: int,
        loop_closures: int,
        reproj_error: float
    ):
        """Update SLAM metrics."""
        self.feature_count = feature_count
        self.loop_closures = loop_closures
        self.reproj_error = reproj_error
        
        # Calculate quality score (0-1)
        feature_score = min(1.0, feature_count / (self.min_features * 2))
        error_score = max(0.0, 1.0 - reproj_error / self.max_reproj_error)
        self.quality = (feature_score + error_score) / 2
    
    def check_health(self) -> Tuple[HealthStatus, str, Dict[str, float]]:
        """Check SLAM health."""
        self.metrics = {
            'feature_count':  float(self.feature_count),
            'loop_closures': float(self.loop_closures),
            'reproj_error':  self.reproj_error,
            'quality': self.quality
        }
        
        if self. feature_count < self.min_features // 2:
            return HealthStatus.CRITICAL, "Very low feature count", self.metrics
        elif self.feature_count < self.min_features:
            return HealthStatus. DEGRADED, "Low feature count", self.metrics
        elif self.reproj_error > self.max_reproj_error:
            return HealthStatus.DEGRADED, "High reprojection error", self.metrics
        else:
            return HealthStatus.OK, "SLAM operating normally", self.metrics


class LocalizationMonitor(SubsystemMonitor):
    """Monitor for localization (EKF) subsystem."""
    
    def __init__(
        self,
        max_position_cov: float = 0.5,
        max_orientation_cov: float = 0.1
    ):
        super().__init__('localization')
        self.max_position_cov = max_position_cov
        self.max_orientation_cov = max_orientation_cov
        
        self.position_cov = 0.0
        self.orientation_cov = 0.0
        self.confidence = 1.0
    
    def update_from_covariance(self, covariance: list):
        """Update from odometry covariance matrix."""
        # Extract diagonal elements
        # covariance is 6x6 row-major:  [x, y, z, roll, pitch, yaw]
        if len(covariance) >= 36:
            self.position_cov = np.sqrt(covariance[0]**2 + covariance[7]**2)
            self.orientation_cov = np.sqrt(covariance[35])
            
            # Calculate confidence (inverse of uncertainty)
            pos_conf = max(0, 1 - self.position_cov / self.max_position_cov)
            ori_conf = max(0, 1 - self.orientation_cov / self.max_orientation_cov)
            self.confidence = (pos_conf + ori_conf) / 2
    
    def check_health(self) -> Tuple[HealthStatus, str, Dict[str, float]]: 
        """Check localization health."""
        self.metrics = {
            'position_uncertainty': self.position_cov,
            'orientation_uncertainty': self.orientation_cov,
            'confidence': self.confidence
        }
        
        if self. position_cov > self.max_position_cov * 2:
            return HealthStatus.CRITICAL, "Very high position uncertainty", self.metrics
        elif self.position_cov > self.max_position_cov:
            return HealthStatus. DEGRADED, "High position uncertainty", self.metrics
        elif self.orientation_cov > self. max_orientation_cov: 
            return HealthStatus.DEGRADED, "High orientation uncertainty", self.metrics
        else:
            return HealthStatus.OK, "Localization stable", self.metrics


class MotorMonitor(SubsystemMonitor):
    """Monitor for motor/VEX subsystem."""
    
    def __init__(
        self,
        max_temperature: float = 55.0,
        min_battery_voltage: float = 10.5
    ):
        super().__init__('motors')
        self.max_temperature = max_temperature
        self.min_battery_voltage = min_battery_voltage
        
        self.temperatures = [0.0, 0.0, 0.0, 0.0]
        self.battery_voltage = 12.0
        self.connected = False
    
    def update(
        self,
        connected: bool,
        battery_voltage: float,
        temperatures: list
    ):
        """Update motor metrics."""
        self. connected = connected
        self.battery_voltage = battery_voltage
        self.temperatures = temperatures[: 4] if len(temperatures) >= 4 else [0]*4
    
    def check_health(self) -> Tuple[HealthStatus, str, Dict[str, float]]:
        """Check motor health."""
        max_temp = max(self.temperatures)
        
        self.metrics = {
            'battery_voltage': self.battery_voltage,
            'max_temperature': max_temp,
            'connected': 1.0 if self.connected else 0.0
        }
        
        if not self.connected:
            return HealthStatus.CRITICAL, "VEX V5 disconnected", self.metrics
        elif self.battery_voltage < self.min_battery_voltage:
            return HealthStatus.CRITICAL, "Battery critically low", self.metrics
        elif max_temp > self.max_temperature:
            return HealthStatus.DEGRADED, "Motor overheating", self.metrics
        elif self.battery_voltage < self.min_battery_voltage + 1.0:
            return HealthStatus.DEGRADED, "Battery low", self.metrics
        else:
            return HealthStatus.OK, "Motors operating normally", self.metrics
