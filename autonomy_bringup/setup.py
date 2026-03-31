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
        # ArUco marker model (model.config, model.sdf, pre-generated texture PNG)
        (os.path.join('share', package_name, 'models', 'aruco_marker_0'),
            glob('models/aruco_marker_0/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_0',
                      'meshes'),
            glob('models/aruco_marker_0/meshes/*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_0',
                      'materials', 'textures'),
            glob('models/aruco_marker_0/materials/textures/*.png')),
        # Utility scripts (spawn helpers)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prajjwal',
    maintainer_email='prajjwal@todo.todo',
    description='Bringup launch for A* trajectory planner + motion on Clearpath A300',
    license='MIT',
    entry_points={
        'console_scripts': [
            'plot_localisation = autonomy_bringup.plot_localisation:main',
            'aruco_detector = autonomy_bringup.aruco_detector:main',
        ],
    },
)
