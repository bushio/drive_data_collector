from setuptools import setup
import os
import glob
package_name = 'drive_data_collector'
submodules = ["drive_data_collector/ros2_numpy"]
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*xml')),
        (os.path.join('share', package_name), glob.glob('launch/*py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bushio',
    maintainer_email='satoshi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_data_collector_node = drive_data_collector.lidar_data_collector:main',
        ],
    },
)
