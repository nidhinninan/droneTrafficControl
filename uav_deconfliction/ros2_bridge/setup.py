from setuptools import setup
import os
from glob import glob

package_name = 'ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={package_name: '.'}, # Treat current directory as the package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 Bridge for UAV Deconfliction System',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ros2_bridge.bridge_node:main',
            'normalization_service = ros2_bridge.normalization_service:main',
            'qgc_watcher = ros2_bridge.qgc_watcher_node:main',
        ],
    },
)
