from setuptools import find_packages, setup
import os
import glob

package_name = 't3mpt3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. Installs all your launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),

        # 2. Installs your config files (like your SLAM yaml)
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),

        # 3. Installs your RViz files
        (os.path.join('share', package_name, 'launch', 'rviz'), glob.glob('launch/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='willmer@wohlen.se',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          # FIXED: Removed the .py
          'mission_control_node = t3mpt3d.mission_control:main',

          # ADDED: Make sure your other nodes are here too so Gazebo/SLAM can use them!
          'frontier_detector_node = t3mpt3d.frontier_detector_node:main',
          'navigation_node = t3mpt3d.navigation_node:main',
          'path_follower_node = t3mpt3d.path_follower_node:main',
        ],
    },
)