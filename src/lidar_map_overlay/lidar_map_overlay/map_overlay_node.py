import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import tf2_geometry_msgs
from sensor_msgs.msg import PointField


class MapOverlayNode(Node):
    def __init__(self):
        super().__init__('map_overlay_node')

        self.draw_distance = self.declare_parameter('draw_distance', 3.0).value

        # Подписка на карту, облако точек и трансформацию
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_points = self.create_subscription(PointCloud2, '/colored_lidar_points', self.pointcloud_callback, 10)
        self.subscription_transform = self.create_subscription(TransformStamped, '/velodyne_fusion_transform', self.transform_callback, 10)

        # Публикация обновленной карты
        self.map_pub = self.create_publisher(OccupancyGrid, '/colored_map', 10)

        # Инициализация переменных
        self.current_map = None
        self.buffered_map_data = None  # Буфер для хранения кумулятивных данных карты
        self.current_transform = None
        self.new_transform = False
        self.new_pointcloud = False

        self.class_intensity = np.array([200, 70, 100, 120, 150, 250, 250], dtype=np.int8)

        self.latest_pointcloud = None
        self.get_logger().info('map_overlay_started')

    def map_callback(self, msg):
        self.get_logger().info("Карта получена.")
        self.current_map = msg

        total_cells = self.current_map.info.width * self.current_map.info.height

        if self.buffered_map_data is None or len(self.buffered_map_data) != total_cells:
            self.get_logger().info(f"Инициализация карты: width={self.current_map.info.width}, height={self.current_map.info.height}, cells={total_cells}")
            self.buffered_map_data = np.full(total_cells, -1, dtype=np.int8)
        else:
            self.get_logger().info("Размер карты не изменился.")

    def transform_callback(self, msg: TransformStamped):
        self.current_transform = msg
        self.new_transform = True
        self.process_data_if_ready()

    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg
        self.new_pointcloud = True
        self.process_data_if_ready()

    def process_data_if_ready(self):
        if not self.new_transform or not self.new_pointcloud:
            return

        self.new_transform = False
        self.new_pointcloud = False

        if self.current_map is None:
            self.get_logger().warn("Карта ещё не получена.")
            return

        if self.current_transform is None or self.latest_pointcloud is None:
            self.get_logger().warn("Не хватает данных для построения карты.")
            return

        # Читаем поля PointCloud2: x,y,z,class_id
        # Проверяем, что поле class_id существует
        field_names = [f.name for f in self.latest_pointcloud.fields]
        if 'class_id' not in field_names:
            self.get_logger().error("В облаке точек отсутствует поле 'class_id'.")
            return

        points = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z", "class_id"), skip_nans=True)
        map_resolution = self.current_map.info.resolution
        map_origin = self.current_map.info.origin

        for point in points:
            x, y, z, class_id = point

            # Фильтры по условиям
            distance = math.sqrt(x**2 + y**2 + z**2)
            if distance > self.draw_distance:
                continue
            if z > 2.0:
                continue
            if z > 1.0:
                class_id = 6
            if x < 0.0:
                continue
            if not (0 <= class_id < len(self.class_intensity)):
                continue

            # Преобразуем точку из локальных координат в глобальные
            point_in_robot_frame = tf2_geometry_msgs.PointStamped()
            point_in_robot_frame.header.frame_id = 'velodyne'
            point_in_robot_frame.point.x = float(x)
            point_in_robot_frame.point.y = float(y)
            point_in_robot_frame.point.z = float(z)

            try:
                point_in_map_frame = tf2_geometry_msgs.do_transform_point(point_in_robot_frame, self.current_transform)
            except Exception as e:
                self.get_logger().error(f"Ошибка преобразования точки: {e}")
                continue

            # Индексы карты
            mx = int((point_in_map_frame.point.x - map_origin.position.x) / map_resolution)
            my = int((point_in_map_frame.point.y - map_origin.position.y) / map_resolution)

            if 0 <= mx < self.current_map.info.width and 0 <= my < self.current_map.info.height:
                index = my * self.current_map.info.width + mx
                if 0 <= index < len(self.buffered_map_data):
                    self.buffered_map_data[index] = self.class_intensity[int(class_id)]
                else:
                    self.get_logger().warn(f"Индекс {index} вне границ массива карты.")
            else:
                self.get_logger().warn(f"Точка ({point_in_map_frame.point.x}, {point_in_map_frame.point.y}) вне границ карты.")

        expected_size = self.current_map.info.width * self.current_map.info.height
        if len(self.buffered_map_data) != expected_size:
            self.get_logger().error(f"Размер данных карты ({len(self.buffered_map_data)}) не соответствует ожиданиям ({expected_size}). Перезапуск буфера.")
            self.buffered_map_data = np.full(expected_size, -1, dtype=np.int8)
            return

        colored_map = OccupancyGrid()
        colored_map.header = self.current_map.header
        colored_map.header.frame_id = "map"
        colored_map.info = self.current_map.info
        colored_map.data = self.buffered_map_data.tolist()

        self.map_pub.publish(colored_map)
        self.get_logger().info("Обновленная карта опубликована.")


def main(args=None):
    rclpy.init(args=args)
    node = MapOverlayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
