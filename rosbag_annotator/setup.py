from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rosbag_annotator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 bag annotation tool for EV data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'annotation_gui = rosbag_annotator.scripts.annotation_gui:main',
            'bag_player = rosbag_annotator.scripts.bag_player:main',
            'annotation_merger = rosbag_annotator.scripts.annotation_merger:main',
        ],
    },
)
