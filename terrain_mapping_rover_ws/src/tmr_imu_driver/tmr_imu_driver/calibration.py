#!/usr/bin/env python3
"""
IMU Calibration Tool

Standalone tool for calibrating the MPU6050 IMU. 
Determines accelerometer and gyroscope offsets while the sensor is stationary. 

Usage:
    ros2 run tmr_imu_driver calibrate_imu
    ros2 run tmr_imu_driver calibrate_imu --samples 1000 --output calibration. yaml
"""

import argparse
import time
import sys
import yaml
from typing import Tuple, Dict, Any

from . mpu6050 import MPU6050


class IMUCalibrator:
    """
    IMU calibration helper class.
    """
    
    def __init__(
        self,
        bus: int = 1,
        address: int = 0x68,
        accel_range_g: int = 2,
        gyro_range_dps: int = 250
    ):
        """
        Initialize calibrator.
        
        Args:
            bus: I2C bus number
            address: I2C address
            accel_range_g:  Accelerometer range in g
            gyro_range_dps: Gyroscope range in degrees/sec
        """
        self.imu = MPU6050(
            bus=bus,
            address=address,
            accel_range_g=accel_range_g,
            gyro_range_dps=gyro_range_dps
        )
    
    def connect(self) -> bool:
        """Connect to IMU"""
        return self.imu.connect()
    
    def disconnect(self):
        """Disconnect from IMU"""
        self.imu. disconnect()
    
    def calibrate(
        self,
        samples: int = 500,
        delay: float = 0.01,
        progress_callback=None
    ) -> Tuple[Dict[str, float], Dict[str, float]]: 
        """
        Perform calibration.
        
        Args:
            samples: Number of samples to collect
            delay: Delay between samples in seconds
            progress_callback: Optional callback(current, total) for progress
        
        Returns:
            Tuple of (accel_offsets_dict, gyro_offsets_dict)
        """
        print(f"\nCollecting {samples} samples...")
        print("Keep the IMU stationary and level!\n")
        
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        
        # Clear existing calibration
        self.imu.set_calibration((0, 0, 0), (0, 0, 0))
        
        for i in range(samples):
            data = self.imu.read()
            
            accel_sum[0] += data.accel_x
            accel_sum[1] += data.accel_y
            accel_sum[2] += data.accel_z
            gyro_sum[0] += data.gyro_x
            gyro_sum[1] += data.gyro_y
            gyro_sum[2] += data.gyro_z
            
            if progress_callback:
                progress_callback(i + 1, samples)
            
            # Print progress bar
            if (i + 1) % 50 == 0 or i == samples - 1:
                progress = (i + 1) / samples * 100
                bar_length = 40
                filled = int(bar_length * (i + 1) / samples)
                bar = '=' * filled + '-' * (bar_length - filled)
                print(f"\r[{bar}] {progress:.1f}%", end='', flush=True)
            
            time.sleep(delay)
        
        print("\n")
        
        # Calculate offsets
        accel_offset = {
            'x': accel_sum[0] / samples,
            'y': accel_sum[1] / samples,
            'z': accel_sum[2] / samples - MPU6050.GRAVITY  # Expect 1g on Z
        }
        
        gyro_offset = {
            'x': gyro_sum[0] / samples,
            'y': gyro_sum[1] / samples,
            'z':  gyro_sum[2] / samples
        }
        
        return (accel_offset, gyro_offset)
    
    def save_calibration(
        self,
        accel_offset: Dict[str, float],
        gyro_offset: Dict[str, float],
        filename:  str
    ):
        """
        Save calibration to YAML file.
        
        Args:
            accel_offset: Accelerometer offsets
            gyro_offset:  Gyroscope offsets
            filename: Output filename
        """
        calibration = {
            'imu_calibration': {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'accelerometer':  {
                    'offset_x': float(accel_offset['x']),
                    'offset_y': float(accel_offset['y']),
                    'offset_z': float(accel_offset['z']),
                },
                'gyroscope': {
                    'offset_x':  float(gyro_offset['x']),
                    'offset_y': float(gyro_offset['y']),
                    'offset_z': float(gyro_offset['z']),
                }
            }
        }
        
        with open(filename, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False)
        
        print(f"Calibration saved to:  {filename}")
    
    def verify_calibration(
        self,
        accel_offset: Dict[str, float],
        gyro_offset: Dict[str, float],
        samples: int = 100
    ) -> Dict[str, Any]:
        """
        Verify calibration by checking residual errors.
        
        Args:
            accel_offset: Accelerometer offsets to verify
            gyro_offset:  Gyroscope offsets to verify
            samples: Number of samples for verification
        
        Returns: 
            Dictionary with verification results
        """
        # Apply calibration
        self.imu.set_calibration(
            (accel_offset['x'], accel_offset['y'], accel_offset['z']),
            (gyro_offset['x'], gyro_offset['y'], gyro_offset['z'])
        )
        
        print(f"\nVerifying calibration with {samples} samples...")
        
        accel_errors = {'x': [], 'y': [], 'z': []}
        gyro_errors = {'x':  [], 'y': [], 'z': []}
        
        for _ in range(samples):
            data = self.imu.read()
            
            # Expected:  accel = (0, 0, 9.81), gyro = (0, 0, 0)
            accel_errors['x'].append(data.accel_x)
            accel_errors['y'].append(data.accel_y)
            accel_errors['z'].append(data.accel_z - MPU6050.GRAVITY)
            gyro_errors['x'].append(data.gyro_x)
            gyro_errors['y']. append(data.gyro_y)
            gyro_errors['z'].append(data.gyro_z)
            
            time.sleep(0.01)
        
        # Calculate statistics
        def calc_stats(values):
            mean = sum(values) / len(values)
            variance = sum((v - mean)**2 for v in values) / len(values)
            std = variance**0.5
            return {'mean': mean, 'std': std, 'max': max(abs(v) for v in values)}
        
        results = {
            'accelerometer': {
                'x': calc_stats(accel_errors['x']),
                'y': calc_stats(accel_errors['y']),
                'z': calc_stats(accel_errors['z']),
            },
            'gyroscope':  {
                'x': calc_stats(gyro_errors['x']),
                'y': calc_stats(gyro_errors['y']),
                'z':  calc_stats(gyro_errors['z']),
            }
        }
        
        return results


def main():
    """Main entry point for calibration tool"""
    parser = argparse.ArgumentParser(description='MPU6050 IMU Calibration Tool')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number')
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x68, 
                        help='I2C address (hex)')
    parser.add_argument('--samples', type=int, default=500, 
                        help='Number of calibration samples')
    parser.add_argument('--output', type=str, default='imu_calibration.yaml',
                        help='Output calibration file')
    parser.add_argument('--verify', action='store_true',
                        help='Verify calibration after collecting')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("MPU6050 IMU Calibration Tool")
    print("=" * 60)
    print(f"\nI2C Bus: {args.bus}")
    print(f"I2C Address: 0x{args.address:02X}")
    print(f"Samples: {args.samples}")
    print(f"Output: {args.output}")
    print("\n" + "=" * 60)
    
    # Create calibrator
    calibrator = IMUCalibrator(bus=args.bus, address=args.address)
    
    # Connect
    print("\nConnecting to IMU...")
    if not calibrator.connect():
        print("ERROR: Failed to connect to IMU!")
        sys.exit(1)
    print("Connected successfully!")
    
    # Perform self-test
    print("\nPerforming self-test...")
    if calibrator.imu.self_test():
        print("Self-test PASSED")
    else:
        print("WARNING: Self-test failed, proceeding anyway...")
    
    # Wait for user to position sensor
    print("\n" + "=" * 60)
    print("IMPORTANT: Place the IMU on a flat, level surface.")
    print("The sensor must be completely stationary during calibration!")
    print("=" * 60)
    input("\nPress Enter when ready to start calibration...")
    
    # Countdown
    for i in range(3, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)
    
    try:
        # Perform calibration
        accel_offset, gyro_offset = calibrator.calibrate(samples=args.samples)
        
        # Print results
        print("\n" + "=" * 60)
        print("CALIBRATION RESULTS")
        print("=" * 60)
        print("\nAccelerometer offsets (m/s²):")
        print(f"  X: {accel_offset['x']: +.6f}")
        print(f"  Y: {accel_offset['y']:+.6f}")
        print(f"  Z: {accel_offset['z']:+.6f}")
        print("\nGyroscope offsets (rad/s):")
        print(f"  X: {gyro_offset['x']:+.6f}")
        print(f"  Y: {gyro_offset['y']:+.6f}")
        print(f"  Z: {gyro_offset['z']:+.6f}")
        
        # Verify if requested
        if args.verify:
            results = calibrator.verify_calibration(accel_offset, gyro_offset)
            
            print("\n" + "=" * 60)
            print("VERIFICATION RESULTS")
            print("=" * 60)
            print("\nAccelerometer residual errors (m/s²):")
            for axis in ['x', 'y', 'z']:
                stats = results['accelerometer'][axis]
                print(f"  {axis. upper()}: mean={stats['mean']: +.4f}, std={stats['std']:.4f}, max={stats['max']:.4f}")
            
            print("\nGyroscope residual errors (rad/s):")
            for axis in ['x', 'y', 'z']: 
                stats = results['gyroscope'][axis]
                print(f"  {axis.upper()}: mean={stats['mean']:+. 6f}, std={stats['std']:.6f}, max={stats['max']:.6f}")
        
        # Save calibration
        calibrator.save_calibration(accel_offset, gyro_offset, args.output)
        
        # Print ROS parameter format
        print("\n" + "=" * 60)
        print("ROS 2 PARAMETER FORMAT")
        print("=" * 60)
        print("\nAdd these to your launch file or parameter file:\n")
        print(f"  accel_offset_x: {accel_offset['x']:.6f}")
        print(f"  accel_offset_y:  {accel_offset['y']:.6f}")
        print(f"  accel_offset_z: {accel_offset['z']:.6f}")
        print(f"  gyro_offset_x: {gyro_offset['x']:.6f}")
        print(f"  gyro_offset_y: {gyro_offset['y']:.6f}")
        print(f"  gyro_offset_z:  {gyro_offset['z']:.6f}")
        
    except KeyboardInterrupt:
        print("\n\nCalibration cancelled by user")
    finally:
        calibrator.disconnect()
    
    print("\nCalibration complete!")


if __name__ == '__main__': 
    main()
