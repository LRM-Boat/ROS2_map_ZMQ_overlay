from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    package_name = 'stream_camera_lidar'
    package_share_directory = get_package_share_directory(package_name)
    cfg_dir = os.path.join(package_share_directory, 'cfg')
    calibration_file = os.path.join(cfg_dir, 'VLP16db.yaml')
    
    # ЛИДАР НОДЫ 
    velodyne_driver_node = Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='screen',
                                                   parameters=[os.path.join(cfg_dir, 'velodyne_driver_node.yaml')])
    
    with open(os.path.join(cfg_dir, 'velodyne_transform_node.yaml'), 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    
    convert_params['calibration'] = os.path.join(cfg_dir, 'VLP16db.yaml')

    velodyne_transform_node = Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='screen',
                                                      parameters=[convert_params])


    # КАМЕРА НОДА
    camera_node = Node(
            package='ximea_cam',
            executable='ximea_frame_publisher',
            name = 'camera',
            output='screen')
    
    # СИНХРОНИЗАЦИЯ ДАННЫХ 
    sync_camera_lidar = Node(
                            package='stream_camera_lidar',
                            executable='sync_node',
                            name = 'sync',
                            output='screen'
                        )

    # Публикация статического TF для камеры
    tf_node =Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'velodyne']
        )

    return LaunchDescription([
        velodyne_driver_node,
        velodyne_transform_node,
        camera_node,
        sync_camera_lidar,
        tf_node
    ])

if __name__ == '__main__':
    generate_launch_description()

