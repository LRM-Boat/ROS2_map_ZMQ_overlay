import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from message_filters import Subscriber, Cache
from message_filters import ApproximateTimeSynchronizer
from geometry_msgs.msg import TransformStamped

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        
        self.image_sub = Subscriber(self, Image, '/ximea_frames_calibrated')
        self.lidar_sub = Subscriber(self, PointCloud2, '/velodyne_points')

        self.image_cache = Cache(self.image_sub, cache_size=10)

        self.ts = ApproximateTimeSynchronizer(
            [self.lidar_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.callback)

        self.image_pub = self.create_publisher(Image, '/synced/ximea_frames_raw', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/synced/velodyne_points', 10)
        

    def callback(self, pointcloud):
        #self.get_logger().info('PointCloud2 received, looking for the latest Image')
        
        # Получение последнего сообщения из кеша
        image = self.image_cache.getLast()
        
        if image is None:
            self.get_logger().warn('No image available in cache')
            return

        #self.get_logger().info('Synchronized messages received')

        # Логи для отладки
        #self.get_logger().info(f'Received Image at {image.header.stamp.sec}.{image.header.stamp.nanosec}')
        #self.get_logger().info(f'Received PointCloud2 at {pointcloud.header.stamp.sec}.{pointcloud.header.stamp.nanosec}')

        # Обновляем временную метку сообщений для синхронизации
        timestamp = self.get_clock().now().to_msg()
        image.header.stamp = timestamp
        pointcloud.header.stamp = timestamp

        # Публикуем синхронизированные сообщения
        self.image_pub.publish(image)
        self.lidar_pub.publish(pointcloud)

        # Логи для подтверждения публикации
        #self.get_logger().info(f'Published synchronized Image at {timestamp.sec}.{timestamp.nanosec}')
        #self.get_logger().info(f'Published synchronized PointCloud2 at {timestamp.sec}.{timestamp.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
