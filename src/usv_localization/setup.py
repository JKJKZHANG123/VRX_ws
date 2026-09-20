import os
from glob import glob
from setuptools import setup

package_name = 'usv_localization'

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
    entry_points={
        'console_scripts': [
            'covariance_injector = usv_localization.covariance_injector:main',
        ],
    },
    maintainer='jkjkzhang',
    maintainer_email='jkjkzhang@todo.todo',
    description='GPS + IMU + Point-LIO EKF fusion for the WAM-V (VRX sim)',
    license='MIT',
)
