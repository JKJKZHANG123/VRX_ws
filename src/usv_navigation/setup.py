from setuptools import setup
import os
from glob import glob

package_name = 'usv_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['pytest.ini']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkjkzhang',
    maintainer_email='jkjkzhang@todo.todo',
    description='Nav2 integration for WAM-V autonomous navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap_3d_markers = usv_navigation.costmap_3d_markers:main',
            'feature_nav_node = usv_navigation.feature_nav_node:main',
            'click_to_goal = usv_navigation.click_to_goal:main',
            'nav_data_logger = usv_navigation.nav_data_logger:main',
            'clock_bridge_watchdog = usv_navigation.clock_bridge_watchdog:main',
            'dynamic_obstacle_tracker = usv_navigation.dynamic_obstacle_tracker:main',
        ],
    },
)
