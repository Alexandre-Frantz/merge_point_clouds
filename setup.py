from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'merge_point_clouds'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandre Frantz',
    maintainer_email='alexandre.frantz@uni.lu',
    description='A ROS2 package to merge SLAM maps generated from LiDAR and RGB-D cameras',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'merge_point_clouds = merge_point_clouds.merge_point_clouds:main',
        ],
    },
)
