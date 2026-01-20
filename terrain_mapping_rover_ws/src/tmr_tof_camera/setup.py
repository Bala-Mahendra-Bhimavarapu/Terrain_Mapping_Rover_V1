from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tmr_tof_camera'

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
    description='Arducam ToF Camera driver for Terrain Mapping Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_camera_node = tmr_tof_camera. tof_camera_node:main',
            'tof_diagnostics = tmr_tof_camera.tof_diagnostics:main',
        ],
    },
)
