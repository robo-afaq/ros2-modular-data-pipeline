import os
from glob import glob
from setuptools import setup

package_name = 'ros2_modular_data_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afaq',
    maintainer_email='afaq@example.com',
    description='ROS 2 modular data pipeline with logging and plotting.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = ros2_modular_data_pipeline.simple_node:main', 'sensor_publisher = ros2_modular_data_pipeline.sensor_publisher:main',
            'processor_node = ros2_modular_data_pipeline.processor_node:main',
        ],
    },
)
