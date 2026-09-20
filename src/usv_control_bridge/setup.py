from setuptools import setup
import os
from glob import glob

package_name = 'usv_control_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkjkzhang',
    maintainer_email='jkjkzhang@todo.todo',
    description='M5 control bridge: cmd_vel to WAM-V thruster commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_bridge = usv_control_bridge.control_bridge:main',
        ],
    },
)
