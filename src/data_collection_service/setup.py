from setuptools import setup
import os
from glob import glob

package_name = 'data_collection_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'h5py', 'numpy'],
    zip_safe=True,
    maintainer='airspeedbox',
    maintainer_email='airspeedbox@example.com',
    description='Robot data storage package for collecting and storing robot interface data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_storage_node = data_collection_service.data_storage_node:main',
        ],
    },
)
