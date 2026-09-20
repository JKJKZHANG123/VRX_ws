from setuptools import setup
import os
from glob import glob

package_name = 'usv_perception'

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
    maintainer='jkjkzhang',
    maintainer_email='jkjkzhang@todo.todo',
    description='USV camera perception for VRX: buoy detection, colored cloud, water-surface filtering.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'buoy_detector = usv_perception.buoy_detector:main',
            'cloud_colorizer = usv_perception.cloud_colorizer:main',
            'water_filter = usv_perception.water_filter:main',
        ],
    },
)
