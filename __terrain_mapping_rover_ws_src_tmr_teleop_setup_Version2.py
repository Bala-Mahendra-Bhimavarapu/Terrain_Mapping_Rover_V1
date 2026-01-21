from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tmr_teleop'

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
            glob(os.path. join('launch', '*.py'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os. path.join('config', '*. yaml'))),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Bala',
    maintainer_email='user@todo.todo',
    description='Teleoperation package for Terrain Mapping Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = tmr_teleop.teleop_keyboard:main',
            'teleop_gamepad = tmr_teleop.teleop_gamepad:main',
            'velocity_smoother = tmr_teleop.velocity_smoother:main',
            'teleop_mux = tmr_teleop.teleop_mux:main',
        ],
    },
)