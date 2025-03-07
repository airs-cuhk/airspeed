from setuptools import find_packages, setup
import os
import glob
package_name = 'airspeed_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airspeed',
    maintainer_email='airspeed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_client_ros2 = airspeed_converter.mocap_client_ros2:main',
            'mocap_client_multithreading_ros2 = airspeed_converter.mocap_client_multithreading_ros2:main',
            'mocap_sub = airspeed_converter.mocap_sub:main',
            "airspeed_converter = airspeed_converter.teleoperation_interface:main"
        ],
    },
)
