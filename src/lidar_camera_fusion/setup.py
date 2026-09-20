from setuptools import setup
from glob import glob
import os

package_name = 'lidar_camera_fusion'

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
    description='LiDAR point cloud coloring using camera RGB images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_colorizer = lidar_camera_fusion.pointcloud_colorizer:main',
        ],
    },
)
