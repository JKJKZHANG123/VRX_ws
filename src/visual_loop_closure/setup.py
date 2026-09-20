from setuptools import setup
from glob import glob
import os

package_name = 'visual_loop_closure'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='user',
    maintainer_email='user@example.com',
    description='Visual loop closure detection and pose graph optimization for LiDAR odometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_slam_node = visual_loop_closure.visual_slam_node:main',
        ],
    },
)
