from setuptools import setup
import os
import glob
package_name = 'drive_data_collector'
submodules = []
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*xml'))
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
            'point_cloud_publisher_node = drive_data_collector.point_cloud_publisher:main',
            'mileage_calculator_node = drive_data_collector.mileage_calculator:main',
            'drivedata_collector_node = drive_data_collector.drivedata_collector_node:main'
        ],
    },
)
