import os
import glob

from setuptools import find_packages, setup

package_name = 'airspeed_robot_interface'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob.glob('config/*.json')),
    (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airspeed',
    maintainer_email='airspeed@cuhk.edu.cn',
    description='Airspeed data collection Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "airspeed_robot_interface=airspeed_robot_interface.airspeed_robot_interface:main",
        ],
    },
)
