from setuptools import find_packages, setup

package_name = 'fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='anton.belolipetskij@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_lidar_fusion = fusion.camera_lidar_fusion:main',
            'camera_lidar_fusion2 = fusion.camera_lidar_fusion2:main',
            'sync_node = fusion.sync:main',
        ],
    },
   
)
