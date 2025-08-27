import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),\
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suhyeon',
    maintainer_email='ssh@inha.edu',
    description='000',
    license='000',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_detector = perception.ball_detector_node:main',
            'ball_follower = perception.ball_follower_node:main',
        ],
    },
)