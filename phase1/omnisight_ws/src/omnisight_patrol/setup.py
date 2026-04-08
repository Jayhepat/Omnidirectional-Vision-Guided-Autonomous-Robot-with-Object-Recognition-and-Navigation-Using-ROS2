from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'omnisight_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omnisight Team',
    maintainer_email='omnisight@vitbhopal.ac.in',
    description='Omnisight Autonomous Indoor Patrol Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'patrol_master         = omnisight_patrol.patrol_master:main',
            'motion_control        = omnisight_patrol.motion_control:main',
            'pan_tilt_scanner      = omnisight_patrol.pan_tilt_scanner:main',
            'scene_monitor         = omnisight_patrol.scene_monitor:main',
            'face_recognition_node = omnisight_patrol.face_recognition_node:main',
            'alert_manager         = omnisight_patrol.alert_manager:main',
            'obstacle_avoidance    = omnisight_patrol.obstacle_avoidance:main',
        ],
    },
)
