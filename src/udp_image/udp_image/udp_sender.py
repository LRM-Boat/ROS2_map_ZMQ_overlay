#!/usr/bin/env python3

import socket
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import struct
import time
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

###############################################################################

TF_RATE_HZ = 10.0
TF_PERIOD = 1.0 / TF_RATE_HZ  # 0.1 с
TF_TIMEOUT = Duration(seconds=0.2)


UDP_IP   = '192.168.107.161'       # адрес получателя
UDP_PORT = 5005
CHUNK    = 1400  


class UdpImageSender(Node):
    def __init__(self):
        super().__init__("udp_image_sender")
                    
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        

        self.subscription = self.create_subscription(
            Image,
            '/synced/ximea_frames_raw',
            self.image_callback,
            10  # QoS-профиль (глубина очереди)
        )

        self.pub = self.create_publisher(
            TransformStamped,
            '/velodyne_saved_transform',   # куда публикуем
        10)
        
        self.last_pub   = self.get_clock().now()        # запоминаем «последний раз»
        self.create_timer(TF_PERIOD, self.pub_tf)
        self.latest_tf: TransformStamped | None = None 

        # Инициализируем UDP-сокет
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_address = (UDP_IP , UDP_PORT)  # Замените на нужный IP и порт
        self.last_pub = self.get_clock().now()
       
        # Для конвертации ROS Image -> OpenCV
        self.bridge = CvBridge()
        
        self.get_logger().info("Image UDP sender node started!")


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
        try:

            sec = msg.header.stamp.sec
            nsec = msg.header.stamp.nanosec
            timestamp = struct.pack('>II', sec, nsec)
            # Конвертируем ROS Image в OpenCV (numpy-массив)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            res=cv2.resize(cv_image,(1000, 512), interpolation=cv2.INTER_LINEAR)
            # Сжимаем изображение в JPEG (для уменьшения размера)
            _, buffer = cv2.imencode('.jpg', res, [cv2.IMWRITE_JPEG_QUALITY, 70])
            packet =timestamp+ buffer.tobytes()
           
            
            # Отправляем по UDP
            self.udp_socket.sendto(packet, self.udp_address)
            self.get_logger().info(
                f"Sent image (size: {len(packet)} ",
                
                )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


###############################################################################


def main(args=None):
    rclpy.init(args=args)
    node = UdpImageSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
