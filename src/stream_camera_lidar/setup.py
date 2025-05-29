from setuptools import setup
from glob import glob
import os


package_name = 'stream_camera_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kulikov',
    maintainer_email='nikita.kulikov.13@yandex.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sync_node = stream_camera_lidar.sync_node:main'
        ],
    },
)
