from setuptools import setup

package_name = 'lidar_timestamp_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkjkzhang',
    maintainer_email='jkjkzhang@todo.todo',
    description='Filter non-finite points from VRX gpu_ray clouds for Point-LIO',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cloud_filter = lidar_timestamp_adapter.cloud_filter_node:main',
        ],
    },
)
