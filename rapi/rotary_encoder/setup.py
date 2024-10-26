from setuptools import setup
import os
from glob import glob

package_name = 'rotary_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Rotary Encoder ROS2 Node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rotary_encoder_node = rotary_encoder.rotary_encoder_node:main',
        ],
    },
)
