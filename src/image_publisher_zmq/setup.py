#!/usr/bin/env python3
from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'image_publisher_zmq'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # индекс пакета для ament
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        # launch-файлы
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',            # ROS 2 Python API
        'pyzmq',            # ZeroMQ
        'opencv-python',    # OpenCV (cv2)
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='anton.belolipetskij@gmail.com',
    description='Publishes camera frames over ZMQ and TF transform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # формát:  <имя-команды> = <python-модуль>:<функция main>
            'zmq_node        = image_publisher_zmq.image_publisher_zmq:main',
            'image_subscriber = image_publisher_zmq.image_subscriber_zmq:main',
        ],
    },
)

