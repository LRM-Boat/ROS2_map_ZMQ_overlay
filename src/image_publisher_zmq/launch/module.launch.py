from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch the Map Overlay Node (tests)
        
        Node(
                            package='ximea_cam',
                            executable='ximea_frame_publisher',
                            name = 'ximea_frame_publisher',
                            output='screen'
                        ),

        
        Node(
                    package='stream_camera_lidar',
                    executable='sync_node',
                    name='sync_node',
                    output='screen'
                   
                ),
        
        Node(
                    package='image_publisher_zmq',
                    executable='zmq_node',
                    name='zmq_pub',
                    output='screen'
                   
                ),
        
        Node(
                    package='image_publisher_zmq',
                    executable='image_subscriber',
                    name='zmq_sub',
                    output='screen'
                   
                ),
        

    

    ])
