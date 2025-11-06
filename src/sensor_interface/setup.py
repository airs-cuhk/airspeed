from setuptools import setup
import os
from glob import glob

package_name = 'sensor_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airspeedbox',
    maintainer_email='airspeedbox@example.com',
    description='Realsense D435i camera node for ROS2',
    license='MIT',
    tests_require=['pytest'],
)
