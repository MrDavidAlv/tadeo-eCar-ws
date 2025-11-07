from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tadeocar_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='axioma',
    maintainer_email='davidalvarez33@gmail.com',
    description='Web-based control system for TadeoeCar 4WS4WD robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_control = tadeocar_control.web_control_node:main',
            'fourws_kinematics = tadeocar_control.fourws_kinematics_node:main',
            'xbox_control = tadeocar_control.xbox_control_node:main',
        ],
    },
)
