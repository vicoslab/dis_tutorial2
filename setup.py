import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dis_tutorial2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vicos',
    maintainer_email='matej.dobrevski@fri.uni-lj.si',
    description='DIS tutorial 2 packages - demonstrating basic ROS2 capabilities.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'random_velocity_publisher = dis_tutorial2.random_velocity_publisher_node:main',
            'go_to_position = dis_tutorial2.go_to_position_simple_node:main',
            'achtung_die_turtle = dis_tutorial2.achtung_die_turtles:main',
            'intersection = dis_tutorial2.intersection:main'
        ],
    },

)
