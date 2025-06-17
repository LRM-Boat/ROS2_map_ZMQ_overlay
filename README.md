# ROS2_map_ZMQ_overlay

# Описание 

Диаграмма проекта 

![Альтернативный текст](./images/diagramm.jpg)

Данный репозиторий не включает в себя SLAM модуль.

Данное рабочее пространство включаетв в себя пакеты для наложения на карту местности классов поверхностей.

# Ноды



 ros2 run ximea_cam ximea_frame_publisher 
 
 ros2 run stream_camera_lidar sync_node
 
 ros2 run image_publisher_zmq zmq_node 
 
 ros2 run image_publisher_zmq image_subscriber
 
 Или launch фаил

 ros2 launch image_publisher_zmq module.launch.py
