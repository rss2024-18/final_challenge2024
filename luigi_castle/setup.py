from setuptools import setup
import os, glob

package_name = 'luigi_castle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/luigi_castle/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
         (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='dorah@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'path_builder = luigi_castle.path_builder:main',
            # 'path_follower = luigi_castle.path_follower:main',
            'path_builder_sim = luigi_castle.path_builder_sim:main',
            'path_follower_sim = luigi_castle.path_follower_sim:main',
        ],
    },
)
