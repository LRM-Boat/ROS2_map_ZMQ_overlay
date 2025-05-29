from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch the Map Overlay Node (tests)
        
        Node(
                            package='stream_camera_lidar',
                            executable='sync_node',
                            name = 'sync_node',
                            output='screen'
                        ),


        
        
        
        Node(
                    package='onnx_segmentation',
                    executable='onnx_segmentation_node',
                    name='onnx_segmentation_node',
                    output='screen',
                    parameters=[
                        {"image_topic": "/image_raw"},
                        {"lidar_topic": "/synced/velodyne_points"},
                        {"transform_topic": "/velodyne_to_map_transform"}
                    ]
                ),
        
        

        
        TimerAction(
            period=3.0,  # Delay in seconds
            actions=[

                Node(
                    package='onnx_segmentation',
                    executable='tests',
                    name='tests',
                    output='screen',
                    parameters=[
                        {"lookup_rate": 0.05},
                        {"lidar_frame": "velodyne"},
                        {"map_frame": "velodyne"}
                    ]
                ), 
            ]
        ),

        Node(
                    package='fusion',
                    executable='camera_lidar_fusion',
                    name='camera_lidar_fusion',
                    output='screen',
                ),

        Node(
                    package='lidar_map_overlay',
                    executable='map_overlay_node',
                    name='map_overlay_node',
                    output='screen',
                    parameters=[
                        {"draw_distance": 4.0}

                    ]
                ),

    ])
