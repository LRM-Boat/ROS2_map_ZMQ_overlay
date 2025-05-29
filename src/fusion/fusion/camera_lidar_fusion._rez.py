import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped


class LidarColorProjector(Node):
    def __init__(self):
        super().__init__('lidar_color_projector')
        
        # Подписки на топики
    
        self.transform_subscription_saved = self.create_subscription(
            TransformStamped,
            '/velodyne_saved_transform',
            self.saved_transform_callback,
            10
        )


        self.subscription_pc = self.create_subscription(
            PointCloud2,
            '/saved_lidar_points',
            self.pointcloud_callback,
            10
        )

        self.subscription_img = self.create_subscription(
            Image,
            '/segmented_image',
            self.image_callback,
            10
        )




        # Публикация цветного облака точек
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/colored_lidar_points', 10)
        self.fusion_transform_pub = self.create_publisher(TransformStamped, '/velodyne_fusion_transform', 10)




        self.bridge = CvBridge()
        self.counter=0
        self.old_counter=0
        # Буфер для текущего изображения, облака точек и меток времени
        self.current_image = None
        self.current_image_timestamp = None
        self.current_pointcloud = None
        self.last_processed_image_timestamp = None  # Метка времени последнего обработанного изображения
        self.current_saved_transform = None
        self.buffer_transform = None
        self.new_image_received = False
        # Параметры камеры
        self.camera_matrix = np.array([
            [1.64808815e+03, 0.00000000e+00, 1.04233398e+03],
            [0.00000000e+00, 1.64743756e+03, 7.84344865e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
        
        self.dist_coeffs = np.array([-0.368, -0.20684, -0.00025, -0.0005429, -0.07773], dtype=np.float32)
        self.tvec = np.array([-0.051539, 0.060649, 0.058266], dtype=np.float32)
        self.rvec_x = np.array([np.pi/2, 0, 0], dtype=np.float32)
        self.rvec_z = np.array([0, -np.pi/2, 0], dtype=np.float32)

        self.get_logger().info("Lidar Color Projector Node Initialized")



    
    def saved_transform_callback(self, msg):
        """Обрабатываем трансформацию из /velodyne_saved_transform."""
        
        self.current_saved_transform = msg
        self.get_logger().info("Transform data received.")


    
    def image_callback(self, msg):
        self.buffer_transform= self.current_saved_transform
       
        if not msg:
            self.get_logger().warn("No image")
            return

        


        # Обновляем текущее изображение и его временную метку
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


        self.current_image_timestamp = msg.header.stamp
        self.get_logger().info(f"Received Image with timestamp: {self.current_image_timestamp.sec}.{self.current_image_timestamp.nanosec}")
        print("Counter:")
        print(self.counter)
        self.counter=self.counter+1
        self.process_and_publish_colored_pointcloud()
        self.fusion_transform_pub.publish(self.buffer_transform)

    def pointcloud_callback(self, msg):
        # Проверяем, есть ли новое изображение для обработки
        
        
        # Сохраняем облако точек и обрабатываем
        self.current_pointcloud = msg
        # Логируем временную метку облака точек
        pointcloud_time_sec = msg.header.stamp.sec
        pointcloud_time_nsec = msg.header.stamp.nanosec
        self.get_logger().info(f"Received PointCloud2 with timestamp: {pointcloud_time_sec}.{pointcloud_time_nsec}")

        

        # Обновляем метку времени последнего обработанного изображения
        self.last_processed_image_timestamp = self.current_image_timestamp

    def process_and_publish_colored_pointcloud(self):
        # Преобразование точек лидара в массив
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(self.current_pointcloud, field_names=("x", "y", "z"), skip_nans=True)]).T

        # Повороты на 90 градусов вокруг X и Z осей
        points_rotated_x = self.rotate_points(points[:3, :], self.rvec_x)
        points_rotated_xz = self.rotate_points(points_rotated_x, self.rvec_z)

        # Проекция точек на изображение
        projected_points, _ = cv2.projectPoints(points_rotated_xz.T, np.zeros(3), self.tvec, self.camera_matrix, self.dist_coeffs)

        # Создание нового облака точек с цветовой информацией
        colored_points = []
        for i, point in enumerate(points.T):
            x_proj, y_proj = projected_points[i][0][0], projected_points[i][0][1]
            if np.isfinite(x_proj) and np.isfinite(y_proj):
                x, y = int(x_proj), int(y_proj)
                if 0 <= x < self.current_image.shape[1] and 0 <= y < self.current_image.shape[0]:
                    color = self.current_image[y, x]
                    r, g, b = int(color[2]), int(color[1]), int(color[0])
                else:
                    r, g, b = 0, 0, 0  # Если точка вне изображения, то чёрный цвет
            else:
                r, g, b = 0, 0, 0  # Если проекция не удалась, то черный цвет

            colored_points.append([point[0], point[1], point[2], (r << 16) | (g << 8) | b])

        # Определение полей для цветного облака точек
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1)
        ]

        # Создание сообщения PointCloud2 с цветными точками
        header = Header()
        header.frame_id = self.current_pointcloud.header.frame_id
        header.stamp = self.current_pointcloud.header.stamp
        colored_cloud_msg = pc2.create_cloud(header, fields, colored_points)

        # Публикация облака точек с цветами
        self.pointcloud_pub.publish(colored_cloud_msg)
        self.get_logger().info("Published colored lidar points")

    def rotate_points(self, points, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        rotated_points = np.dot(rotation_matrix, points)
        return rotated_points

def main(args=None):
    rclpy.init(args=args)
    node = LidarColorProjector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()