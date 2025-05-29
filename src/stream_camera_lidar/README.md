# Синхронизация данных с камеры и лидара 

Синхронизация по принципу: берем последний кадр с кеша в момент обновления кадра с лидара. 

## Запуск

предварительно нужно скачать репозиторий для ximea

```bash 
ros2 launch stream_camera_lidar lidar_camera_launch.py 
```

Интересующие топики: `/synced/velodyne_points` и `/synced/ximea_frames`


## Записываем rosbag

сжатый

```bash
ros2 bag record /synced/velodyne_points /synced/ximea_frames_raw --compression-mode message --compression-format zstd
```

не сжатый со всеми параметрами 

```bash
ros2 bag record /ximea_frames_raw /ximea_frames_calibrated /ximea_camera_info /velodyne_points /synced/velodyne_points /synced/ximea_frames_raw /tf_static
```

только с синхронизированными 
```bash
ros2 bag record /ximea_camera_info /synced/velodyne_points /synced/ximea_frames_raw /tf_static
```

## Воспроизводим 

```bash 
ros2 bag play path/to/db3 --loop
```