import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'mario_circuit'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/mario_circuit/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian',
    maintainer_email='sebastianag2002@gmail.com',
    description='Mario Circuit ROS2 Package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_segmentation = mario_circuit.lane_segmentation:main',
            'homography_transformer = mario_circuit.homography_transformer:main',
            'goal_follower = mario_circuit.goal_follower:main'
        ],
    },
)
