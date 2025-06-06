import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import tf2_geometry_msgs


class MapOverlayNode(Node):
    def __init__(self):
        super().__init__('map_overlay_node')

        self.draw_distance = self.declare_parameter('draw_distance', 3.0).value

        # Подписки
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_points = self.create_subscription(PointCloud2, '/colored_lidar_points', self.pointcloud_callback, 10)
        self.subscription_transform = self.create_subscription(TransformStamped, '/velodyne_fusion_transform', self.transform_callback, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, '/colored_map', 10)

        # Карта и буферы
        self.current_map = None
        self.buffered_map_data = None
        self.old_width = None
        self.old_height = None
        self.old_origin_x = None
        self.old_origin_y = None

        self.current_transform = None
        self.new_transform = False
        self.new_pointcloud = False

        self.class_intensity = np.array([300, 50, 200, 20, 120, 5, 180], dtype=np.int8)
        self.latest_pointcloud = None
        self.get_logger().info('map_overlay_started')

    def map_callback(self, msg):
        self.get_logger().info("Карта получена.")
        total_cells = msg.info.width * msg.info.height

        # Обновление, если карта сменилась
        if (self.buffered_map_data is None or
            self.old_width != msg.info.width or
            self.old_height != msg.info.height or
            self.old_origin_x != msg.info.origin.position.x or
            self.old_origin_y != msg.info.origin.position.y):

            self.get_logger().info("Обновление карты с переносом старых данных...")
            new_data = np.full(total_cells, -1, dtype=np.int8)

            if self.buffered_map_data is not None:
                dx = int((self.old_origin_x - msg.info.origin.position.x) / msg.info.resolution)
                dy = int((self.old_origin_y - msg.info.origin.position.y) / msg.info.resolution)

                for y in range(self.old_height):
                    for x in range(self.old_width):
                        old_idx = y * self.old_width + x
                        new_x = x + dx
                        new_y = y + dy
                        if 0 <= new_x < msg.info.width and 0 <= new_y < msg.info.height:
                            new_idx = new_y * msg.info.width + new_x
                            new_data[new_idx] = self.buffered_map_data[old_idx]

            self.buffered_map_data = new_data

            self.old_width = msg.info.width
            self.old_height = msg.info.height
            self.old_origin_x = msg.info.origin.position.x
            self.old_origin_y = msg.info.origin.position.y

        self.current_map = msg

    def transform_callback(self, msg):
        self.current_transform = msg
        self.new_transform = True
        self.process_data_if_ready()

    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg
        self.new_pointcloud = True
        self.process_data_if_ready()

    def process_data_if_ready(self):
        if not self.new_transform or not self.new_pointcloud or self.current_map is None:
            return

        self.new_transform = False
        self.new_pointcloud = False

        field_names = [f.name for f in self.latest_pointcloud.fields]
        if 'class_id' not in field_names:
            self.get_logger().error("PointCloud2 missing 'class_id'.")
            return

        points = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z", "class_id"), skip_nans=True)
        res = self.current_map.info.resolution
        origin = self.current_map.info.origin

        for point in points:
            x, y, z, class_id = point
            distance = math.sqrt(x**2 + y**2 + z**2)
            if distance > self.draw_distance or z > 2.0:
                continue
            if z > 1.0:
                class_id = 6
            if x < 0 or not (0 <= class_id < len(self.class_intensity)):
                continue

            local_point = tf2_geometry_msgs.PointStamped()
            local_point.header.frame_id = 'velodyne'
            local_point.point.x = float(x)
            local_point.point.y = float(y)
            local_point.point.z = float(z)

            try:
                global_point = tf2_geometry_msgs.do_transform_point(local_point, self.current_transform)
            except Exception as e:
                self.get_logger().error(f"TF error: {e}")
                continue

            mx = int((global_point.point.x - origin.position.x) / res)
            my = int((global_point.point.y - origin.position.y) / res)
            if 0 <= mx < self.current_map.info.width and 0 <= my < self.current_map.info.height:
                index = my * self.current_map.info.width + mx
                if 0 <= index < len(self.buffered_map_data):
                    self.buffered_map_data[index] = self.class_intensity[int(class_id)]

        msg_out = OccupancyGrid()
        msg_out.header = self.current_map.header
        msg_out.header.frame_id = "map"
        msg_out.info = self.current_map.info
        msg_out.data = self.buffered_map_data.tolist()
        self.map_pub.publish(msg_out)
        self.get_logger().info("Опубликована обновленная карта")


def main(args=None):
    rclpy.init(args=args)
    node = MapOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
