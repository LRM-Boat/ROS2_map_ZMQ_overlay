import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointField
import sensor_msgs.msg


class LidarColorProjector(Node):
    def __init__(self):
        super().__init__('lidar_color_projector')
        
        # Подписки на топики
    
        self.transform_subscription_saved = self.create_subscription(
            TransformStamped,
            '/sync_transform',
            self.saved_transform_callback,
            30
        )


        self.subscription_pc = self.create_subscription(
            PointCloud2,
            '/sync_points',
            self.pointcloud_callback,
            10
        )

        self.subscription_img = self.create_subscription(
            Image,
            '/camera/external_sync',
            self.image_callback,
            10
        )


        self._win_name = "Lidar-on-Image"
        cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL) 

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
            [2.49967857e+03, 0.00000000e+00, 9.89135886e+02],
            [0.00000000e+00, 2.50556976e+03, 7.71851614e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
        
        self.dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
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
        # --- 1. Преобразуем PointCloud2 в numpy-массив (3 x N)
        pts_xyz = np.array(
            [[p[0], p[1], p[2]]
             for p in pc2.read_points(
                 self.current_pointcloud,
                 field_names=('x', 'y', 'z'),
                 skip_nans=True)],
            dtype=np.float32
        ).T  # (3, N)

        if pts_xyz.size == 0 or self.current_image is None:
            return

        # --- 2. Повороты для согласования координат
        R_x, _ = cv2.Rodrigues(self.rvec_x)
        R_z, _ = cv2.Rodrigues(self.rvec_z)
        pts_cam = R_z @ (R_x @ pts_xyz)  # (3, N)

        # --- 3. Проекция точек на изображение
        img_pts, _ = cv2.projectPoints(
            pts_cam.T,
            np.zeros(3),  # rvec = 0
            self.tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        img_pts = img_pts.reshape(-1, 2)  # (N, 2)

        h, w = self.current_image.shape[:2]
        colored_points = []
        class_ids = []

        # Палитра и сопоставление цветов к классам (BGR)
        palette = [
            [196, 123, 55],   # background
            [65, 72, 74],     # gravel
            [69, 115, 153],   # dirt
            [173, 179, 184],  # asphalt
            [126, 194, 46],   # grass
            [184, 179, 173],  # penablock
            [2,   2,   3]     # wall
        ]

        def get_class_by_color(bgr):
            for idx, color in enumerate(palette):
                if (bgr[0] == color[0] and bgr[1] == color[1] and bgr[2] == color[2]):
                    return idx
            return 0  # default to background если не найдено

        for i, (u, v) in enumerate(img_pts):
            if np.isfinite(u) and np.isfinite(v):
                u_i, v_i = int(round(u)), int(round(v))
                if 0 <= u_i < w and 0 <= v_i < h:
                    bgr = self.current_image[v_i, u_i]
                    r, g, b = int(bgr[2]), int(bgr[1]), int(bgr[0])
                    class_id = get_class_by_color(bgr)
                else:
                    r = g = b = 0
                    class_id = 0
            else:
                r = g = b = 0
                class_id = 0

            rgb_uint32 = (r << 16) | (g << 8) | b
            colored_points.append([pts_xyz[0, i], pts_xyz[1, i], pts_xyz[2, i], rgb_uint32, class_id])

        # --- 4. Определяем поля PointCloud2, включая class_id
        fields = [
            sensor_msgs.msg.PointField(name='x', offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
            sensor_msgs.msg.PointField(name='y', offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
            sensor_msgs.msg.PointField(name='z', offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
            sensor_msgs.msg.PointField(name='rgb', offset=12, datatype=sensor_msgs.msg.PointField.UINT32, count=1),
            sensor_msgs.msg.PointField(name='class_id', offset=16, datatype=sensor_msgs.msg.PointField.UINT8, count=1),
        ]

        # --- 5. Создаём облако точек с новым полем
        header = self.current_pointcloud.header
        cloud_msg = pc2.create_cloud(header, fields, colored_points)

        # --- 6. Публикуем облако
        self.pointcloud_pub.publish(cloud_msg)
        self.get_logger().info("Published colored lidar points with class IDs")

        # --- 7. Показываем наложение
        overlay = self.current_image.copy()
        for (u, v) in img_pts.astype(int):
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(overlay, (u, v), 1, (0, 255, 0), -1)
        cv2.imshow(self._win_name, cv2.resize(overlay, (1024, 720)))
        cv2.waitKey(1)

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
