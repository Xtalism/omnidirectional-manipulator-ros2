import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'dm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Piña Olivas',
    maintainer_email='manuel.pina.olivas@gmail.com',
    description='Omnidirectional Manipulator for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = dm.my_publisher:main',
            'my_second_publisher = dm.my_second_publisher:main',
            'my_subscriber = dm.my_subscriber:main'
        ],
    },
)