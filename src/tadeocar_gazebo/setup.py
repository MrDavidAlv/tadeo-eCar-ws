from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tadeocar_gazebo'


def get_data_files_recursive(source_dir, install_prefix):
    """Generate data_files entries for a directory tree."""
    data_files = []
    for dirpath, _, filenames in os.walk(source_dir):
        if filenames:
            install_dir = os.path.join(
                install_prefix, os.path.relpath(dirpath, source_dir)
            )
            files = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((install_dir, files))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]
data_files += get_data_files_recursive(
    'models', os.path.join('share', package_name, 'models'))
data_files += get_data_files_recursive(
    'worlds', os.path.join('share', package_name, 'worlds'))
# The generators ship with the worlds they produce: a world edited by hand and
# a generator left behind is how the map and the world drift apart again.
data_files.append(
    (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MrDavidAlv',
    maintainer_email='davidalvarezzzzzzzz@gmail.com',
    description='TadeoeCar Gz Sim simulation, bridge and spawn for ROS2 Humble',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'odom_to_tf = tadeocar_gazebo.odom_to_tf:main',
        ],
    },
)
