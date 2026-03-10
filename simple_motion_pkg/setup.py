from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_motion_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prajjwal',
    maintainer_email='prajjwal@todo.todo',
    description='Minimal path-following P-controller for Clearpath A300',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_follower = simple_motion_pkg.path_follower:main',
            'twist_mux = simple_motion_pkg.twist_mux:main',
        ],
    },
)
