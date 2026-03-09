#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name='navigation',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/navigation']),
        ('share/navigation', ['package.xml']),
        ('share/navigation/config', [
            'config/nav2_params.yaml',
            'config/localization_gps.yaml',
            'config/waypoints_square.yaml',
            'config/waypoints_figure8.yaml',
            'config/waypoints_corridor.yaml',
        ]),
        ('share/navigation/launch', [
            'launch/navigation.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    author='Prajjwal',
    author_email='prajjwal@example.com',
    maintainer='Prajjwal',
    maintainer_email='prajjwal@example.com',
    url='https://github.com/DPRIcky/Husky_mobile_robot_automation',
    description='Autonomous navigation for Clearpath A300',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_navigator=navigation.waypoint_navigator:main',
        ],
    },
)
