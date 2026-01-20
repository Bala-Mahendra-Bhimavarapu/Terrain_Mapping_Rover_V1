"""
MPU6050 I2C Driver

Low-level driver for the MPU6050 6-axis IMU (3-axis accelerometer + 3-axis gyroscope).

Features:
- Configurable accelerometer and gyroscope ranges
- Digital Low Pass Filter (DLPF) configuration
- Temperature reading
- Raw and calibrated data output

Hardware connections (I2C):
- VCC -> 3.3V (or 5V with level shifter)
- GND -> GND
- SDA -> GPIO2 (I2C1 SDA on Pi 5)
- SCL -> GPIO3 (I2C1 SCL on Pi 5)
- AD0 -> GND (for address 0x68) or VCC (for address 0x69)
"""

import struct
import time
from typing import Tuple, Optional
from dataclasses import dataclass

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    print("WARNING: smbus2 not available.  Using mock I2C.")


@dataclass
class IMUData:
    """Container for IMU readings"""
    accel_x:  float  # m/s^2
    accel_y: float  # m/s^2
    accel_z: float  # m/s^2
    gyro_x: float   # rad/s
    gyro_y:  float   # rad/s
    gyro_z: float   # rad/s
    temperature: float  # Celsius
    timestamp:  float    # seconds


class MPU6050:
    """
    MPU6050 IMU Driver
    
    Provides interface to read accelerometer, gyroscope, and temperature
    data from the MPU6050 via I2C.
    """
    
    # ==========================================================================
    # Register Addresses
    # ==========================================================================
    
    # Configuration registers
    REG_PWR_MGMT_1 = 0x6B
    REG_PWR_MGMT_2 = 0x6C
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_SMPLRT_DIV = 0x19
    REG_INT_ENABLE = 0x38
    REG_INT_STATUS = 0x3A
    
    # Data registers
    REG_ACCEL_XOUT_H = 0x3B
    REG_ACCEL_XOUT_L = 0x3C
    REG_ACCEL_YOUT_H = 0x3D
    REG_ACCEL_YOUT_L = 0x3E
    REG_ACCEL_ZOUT_H = 0x3F
    REG_ACCEL_ZOUT_L = 0x40
    REG_TEMP_OUT_H = 0x41
    REG_TEMP_OUT_L = 0x42
    REG_GYRO_XOUT_H = 0x43
    REG_GYRO_XOUT_L = 0x44
    REG_GYRO_YOUT_H = 0x45
    REG_GYRO_YOUT_L = 0x46
    REG_GYRO_ZOUT_H = 0x47
    REG_GYRO_ZOUT_L = 0x48
    
    # WHO_AM_I register (should return 0x68)
    REG_WHO_AM_I = 0x75
    
    # ==========================================================================
    # Configuration Constants
    # ==========================================================================
    
    # Accelerometer ranges and scale factors
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18
    
    ACCEL_SCALE = {
        2: 16384.0,   # LSB/g for ±2g
        4: 8192.0,    # LSB/g for ±4g
        8: 4096.0,    # LSB/g for ±8g
        16: 2048.0,   # LSB/g for ±16g
    }
    
    ACCEL_RANGE_MAP = {
        2: ACCEL_RANGE_2G,
        4: ACCEL_RANGE_4G,
        8: ACCEL_RANGE_8G,
        16: ACCEL_RANGE_16G,
    }
    
    # Gyroscope ranges and scale factors
    GYRO_RANGE_250 = 0x00
    GYRO_RANGE_500 = 0x08
    GYRO_RANGE_1000 = 0x10
    GYRO_RANGE_2000 = 0x18
    
    GYRO_SCALE = {
        250: 131.0,    # LSB/(°/s) for ±250°/s
        500: 65.5,     # LSB/(°/s) for ±500°/s
        1000: 32.8,    # LSB/(°/s) for ±1000°/s
        2000: 16.4,    # LSB/(°/s) for ±2000°/s
    }
    
    GYRO_RANGE_MAP = {
        250: GYRO_RANGE_250,
        500: GYRO_RANGE_500,
        1000: GYRO_RANGE_1000,
        2000: GYRO_RANGE_2000,
    }
    
    # Gravity constant
    GRAVITY = 9.80665  # m/s^2
    
    # Degrees to radians
    DEG_TO_RAD = 0.017453292519943295  # pi / 180
    
    def __init__(
        self,
        bus: int = 1,
        address: int = 0x68,
        accel_range_g: int = 2,
        gyro_range_dps: int = 250,
        dlpf_mode: int = 3
    ):
        """
        Initialize MPU6050 driver. 
        
        Args:
            bus: I2C bus number (usually 1 on Raspberry Pi)
            address: I2C address (0x68 or 0x69)
            accel_range_g: Accelerometer range in g (2, 4, 8, or 16)
            gyro_range_dps: Gyroscope range in degrees/sec (250, 500, 1000, 2000)
            dlpf_mode: Digital Low Pass Filter mode (0-6)
        """
        self.bus_num = bus
        self.address = address
        self.bus:  Optional[smbus2.SMBus] = None
        
        # Validate and set ranges
        if accel_range_g not in self.ACCEL_SCALE:
            raise ValueError(f"Invalid accel_range_g:  {accel_range_g}. Must be 2, 4, 8, or 16")
        if gyro_range_dps not in self.GYRO_SCALE:
            raise ValueError(f"Invalid gyro_range_dps: {gyro_range_dps}. Must be 250, 500, 1000, or 2000")
        
        self.accel_range = accel_range_g
        self.gyro_range = gyro_range_dps
        self.dlpf_mode = dlpf_mode
        
        # Scale factors
        self.accel_scale = self.ACCEL_SCALE[accel_range_g]
        self.gyro_scale = self.GYRO_SCALE[gyro_range_dps]
        
        # Calibration offsets
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        # Connection state
        self.connected = False
        
    def connect(self) -> bool:
        """
        Connect to the MPU6050 via I2C.
        
        Returns:
            True if connection successful
        """
        if not SMBUS_AVAILABLE:
            print("WARNING: smbus2 not available, using mock mode")
            self.connected = True
            return True
        
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            
            # Verify device identity
            who_am_i = self.bus.read_byte_data(self.address, self.REG_WHO_AM_I)
            if who_am_i != 0x68 and who_am_i != 0x98:  # Some clones return 0x98
                print(f"WARNING:  Unexpected WHO_AM_I value:  0x{who_am_i:02X}")
            
            # Wake up the MPU6050 (it starts in sleep mode)
            self._wake_up()
            
            # Configure sensor
            self._configure()
            
            self.connected = True
            return True
            
        except Exception as e:
            print(f"Failed to connect to MPU6050: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close I2C connection"""
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        self.connected = False
    
    def _wake_up(self):
        """Wake up the MPU6050 from sleep mode"""
        # Clear sleep bit (bit 6) and set clock source to PLL with X gyro
        self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x01)
        time.sleep(0.1)  # Wait for device to wake up
    
    def _configure(self):
        """Configure sensor ranges and DLPF"""
        # Set accelerometer range
        accel_config = self.ACCEL_RANGE_MAP[self.accel_range]
        self.bus.write_byte_data(self.address, self. REG_ACCEL_CONFIG, accel_config)
        
        # Set gyroscope range
        gyro_config = self.GYRO_RANGE_MAP[self.gyro_range]
        self.bus.write_byte_data(self.address, self.REG_GYRO_CONFIG, gyro_config)
        
        # Set DLPF mode
        self. bus.write_byte_data(self.address, self.REG_CONFIG, self.dlpf_mode)
        
        # Set sample rate divider (1kHz / (1 + divider))
        # For 100 Hz: divider = 9
        self.bus.write_byte_data(self.address, self.REG_SMPLRT_DIV, 9)
    
    def _read_word(self, reg_h: int) -> int:
        """Read a 16-bit signed value from two consecutive registers"""
        high = self.bus.read_byte_data(self.address, reg_h)
        low = self.bus.read_byte_data(self.address, reg_h + 1)
        
        value = (high << 8) | low
        
        # Convert to signed
        if value >= 0x8000:
            value = value - 0x10000
        
        return value
    
    def _read_all_raw(self) -> Tuple[int, int, int, int, int, int, int]:
        """
        Read all sensor data in a single I2C transaction for efficiency.
        
        Returns:
            Tuple of (ax, ay, az, temp, gx, gy, gz) as raw integers
        """
        if not SMBUS_AVAILABLE or self.bus is None:
            # Return mock data for testing without hardware
            return (0, 0, 16384, 0, 0, 0, 0)  # 1g on Z axis
        
        # Read 14 bytes starting from ACCEL_XOUT_H
        data = self.bus. read_i2c_block_data(self.address, self. REG_ACCEL_XOUT_H, 14)
        
        # Unpack the data (big-endian signed shorts)
        ax = struct.unpack('>h', bytes(data[0:2]))[0]
        ay = struct.unpack('>h', bytes(data[2:4]))[0]
        az = struct. unpack('>h', bytes(data[4:6]))[0]
        temp = struct.unpack('>h', bytes(data[6:8]))[0]
        gx = struct.unpack('>h', bytes(data[8:10]))[0]
        gy = struct.unpack('>h', bytes(data[10:12]))[0]
        gz = struct.unpack('>h', bytes(data[12:14]))[0]
        
        return (ax, ay, az, temp, gx, gy, gz)
    
    def read_raw(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int], int]:
        """
        Read raw sensor values. 
        
        Returns:
            Tuple of ((ax, ay, az), (gx, gy, gz), temp) as raw integers
        """
        ax, ay, az, temp, gx, gy, gz = self._read_all_raw()
        return ((ax, ay, az), (gx, gy, gz), temp)
    
    def read(self) -> IMUData:
        """
        Read calibrated sensor data.
        
        Returns:
            IMUData with calibrated values in SI units
        """
        ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw = self._read_all_raw()
        
        # Convert accelerometer to m/s^2
        accel_x = (ax_raw / self.accel_scale) * self.GRAVITY - self.accel_offset[0]
        accel_y = (ay_raw / self.accel_scale) * self.GRAVITY - self.accel_offset[1]
        accel_z = (az_raw / self.accel_scale) * self.GRAVITY - self.accel_offset[2]
        
        # Convert gyroscope to rad/s
        gyro_x = (gx_raw / self.gyro_scale) * self.DEG_TO_RAD - self.gyro_offset[0]
        gyro_y = (gy_raw / self. gyro_scale) * self.DEG_TO_RAD - self.gyro_offset[1]
        gyro_z = (gz_raw / self.gyro_scale) * self.DEG_TO_RAD - self.gyro_offset[2]
        
        # Convert temperature (formula from datasheet)
        temperature = (temp_raw / 340.0) + 36.53
        
        return IMUData(
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            temperature=temperature,
            timestamp=time.time()
        )
    
    def set_calibration(
        self,
        accel_offset: Tuple[float, float, float],
        gyro_offset: Tuple[float, float, float]
    ):
        """
        Set calibration offsets. 
        
        Args:
            accel_offset: (x, y, z) accelerometer offsets in m/s^2
            gyro_offset: (x, y, z) gyroscope offsets in rad/s
        """
        self.accel_offset = list(accel_offset)
        self.gyro_offset = list(gyro_offset)
    
    def calibrate(self, samples: int = 500, delay: float = 0.005) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Perform calibration by averaging samples while stationary.
        
        The sensor should be stationary and level during calibration.
        Accelerometer Z-axis calibration assumes gravity pointing down.
        
        Args:
            samples: Number of samples to average
            delay: Delay between samples in seconds
        
        Returns:
            Tuple of (accel_offset, gyro_offset)
        """
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        
        # Temporarily clear offsets
        old_accel_offset = self. accel_offset. copy()
        old_gyro_offset = self.gyro_offset.copy()
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        # Collect samples
        for _ in range(samples):
            data = self.read()
            accel_sum[0] += data.accel_x
            accel_sum[1] += data.accel_y
            accel_sum[2] += data.accel_z
            gyro_sum[0] += data.gyro_x
            gyro_sum[1] += data.gyro_y
            gyro_sum[2] += data.gyro_z
            time.sleep(delay)
        
        # Calculate averages
        accel_offset = (
            accel_sum[0] / samples,
            accel_sum[1] / samples,
            accel_sum[2] / samples - self. GRAVITY  # Expect 1g on Z when level
        )
        
        gyro_offset = (
            gyro_sum[0] / samples,
            gyro_sum[1] / samples,
            gyro_sum[2] / samples
        )
        
        # Apply new calibration
        self.accel_offset = list(accel_offset)
        self.gyro_offset = list(gyro_offset)
        
        return (accel_offset, gyro_offset)
    
    def read_temperature(self) -> float:
        """
        Read temperature only.
        
        Returns:
            Temperature in Celsius
        """
        if not SMBUS_AVAILABLE or self. bus is None:
            return 25.0  # Mock value
        
        temp_raw = self._read_word(self.REG_TEMP_OUT_H)
        return (temp_raw / 340.0) + 36.53
    
    def self_test(self) -> bool:
        """
        Perform basic self-test.
        
        Returns:
            True if self-test passed
        """
        try:
            # Read WHO_AM_I
            if SMBUS_AVAILABLE and self.bus is not None:
                who_am_i = self.bus.read_byte_data(self.address, self.REG_WHO_AM_I)
                if who_am_i != 0x68 and who_am_i != 0x98:
                    return False
            
            # Read some data and verify it's reasonable
            data = self.read()
            
            # Check that readings are within reasonable ranges
            accel_magnitude = (data.accel_x**2 + data.accel_y**2 + data.accel_z**2)**0.5
            if not (5.0 < accel_magnitude < 15.0):  # Should be near 9.8 m/s^2
                return False
            
            # Check temperature is reasonable
            if not (-40.0 < data.temperature < 85.0):
                return False
            
            return True
            
        except Exception: 
            return False
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
        return False
