import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hannibal Paul',
    maintainer_email='ubuntu@todo.todo',
    description='A ROS 2 package to send offboard setpoints',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_offboard = px4_offboard.setpoint_member_function:main',
        ],
    },
)
