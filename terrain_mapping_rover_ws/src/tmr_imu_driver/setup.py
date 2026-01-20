from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tmr_imu_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker for ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package. xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.py'))),
        # Config files
        (os.path. join('share', package_name, 'config'), 
            glob(os.path. join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Bala',
    maintainer_email='user@todo.todo',
    description='MPU6050 IMU driver for Terrain Mapping Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = tmr_imu_driver.imu_node:main',
            'calibrate_imu = tmr_imu_driver.calibration:main',
            'imu_diagnostics = tmr_imu_driver.imu_diagnostics:main',
        ],
    },
)
