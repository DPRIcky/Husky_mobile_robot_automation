from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomy_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prajjwal',
    maintainer_email='prajjwal@todo.todo',
    description='Bringup launch for A* trajectory planner + motion on Clearpath A300',
    license='MIT',
    entry_points={'console_scripts': []},
)
