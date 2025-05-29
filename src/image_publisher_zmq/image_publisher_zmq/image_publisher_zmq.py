import rclpy
from rclpy.node import Node
import zmq
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import struct

import rclpy, tf2_ros
from geometry_msgs.msg import TransformStamped

from rclpy.duration import Duration

TF_RATE_HZ = 10.0
TF_PERIOD  = 1.0 / TF_RATE_HZ          # 0.1 с
TF_TIMEOUT = Duration(seconds=0.2)

class ImagePublisherZMQ(Node):
    def __init__(self):
        super().__init__('image_publisher_zmq')


        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            TransformStamped,
            '/velodyne_saved_transform',   # куда публикуем
            10)
        

        # Настройка ZeroMQ для отправки изображений
        zmq_pub_ip = "127.0.0.1"
        zmq_pub_port = "5556"
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{zmq_pub_ip}:{zmq_pub_port}")

        # Подписка на ROS2 топик с изображениями
        self.image_subscription = self.create_subscription(
            Image,
            '/synced/ximea_frames_raw',
            self.image_callback,
            10
        )

        
        self.last_pub   = self.get_clock().now()        # запоминаем «последний раз»
        self.create_timer(TF_PERIOD, self.pub_tf)
        self.latest_tf: TransformStamped | None = None 

        self.bridge = CvBridge()
        self.get_logger().info("ZeroMQ Image Publisher Initialized")
        


    def pub_tf(self):
        now = self.get_clock().now()

        # публикуем раз в 0.1 с
        if (now - self.last_pub).nanoseconds < TF_PERIOD * 1e9:
            return
        self.last_pub = now

        try:
            # «последний доступный» трансформ
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='velodyne',
                time          = rclpy.time.Time(),
                timeout       = TF_TIMEOUT)

            self.pub.publish(tf_msg)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):     # ← ДОБАВЛЕНО
            self.get_logger().debug("TF map→velodyne недоступен – повторю")

    def image_callback(self, msg):
        # Преобразуем ROS2 Image в OpenCV изображение
        sec  = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        timestamp = f"{sec}.{nsec:09d}" 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Кодируем изображение в формат JPEG
        _, buffer = cv2.imencode('.jpg', cv_image)
        hdr = struct.pack('>II', sec, nsec) 
        payload = hdr + buffer.tobytes() 
        self.socket.send(payload)
        self.get_logger().info("Image sent via ZeroMQ")

        # Очищаем буфер отправленных сообщений, чтобы не накапливались старые изображения
        self.clear_buffer()
        #if self.latest_tf is not None:
           # self.tf_pub.publish(self.latest_tf)

    def clear_buffer(self):
        """Очищает буфер отправленных сообщений в ZeroMQ."""
        self.socket.setsockopt(zmq.LINGER, 0)  # Немедленно очищает буфер, не дожидаясь отправки старых сообщений.


        # Очистка очереди сообщений перед запуском

def main(args=None):
    rclpy.init(args=args)
    
    image_publisher = ImagePublisherZMQ()
    # Запускаем оба узла параллельно
    rclpy.spin(image_publisher)


    rclpy.shutdown()


if __name__ == '__main__':
    main()

