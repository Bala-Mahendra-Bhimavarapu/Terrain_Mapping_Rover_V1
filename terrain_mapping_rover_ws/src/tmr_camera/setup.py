from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tmr_camera'

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
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Bala',
    maintainer_email='user@todo. todo',
    description='IMX500 Camera driver for Terrain Mapping Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = tmr_camera.camera_node:main',
            'camera_diagnostics = tmr_camera.camera_diagnostics:main',
        ],
    },
)
