#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import collections

SYNC_TOL = 0.30  # сек – допуск для синхронизации
BUF_LIFE = 2.0   # сколько храним сообщения, сек
BUF_MAX = 200

class SimpleUdpReceiver(Node):
    def __init__(self):
        super().__init__('udp_image_receiver')
        
        # Параметры UDP
        self.declare_parameter('udp_ip', '0.0.0.0') 
        self.declare_parameter('udp_port', 5006)
        
        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = self.get_parameter('udp_port').value
    
        # Настройка UDP сокета
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.udp_ip, self.udp_port))
        self.socket.settimeout(0.1)  # Таймаут 100 мс
        
        # ROS публикаторы
        self.pub_img = self.create_publisher(Image, '/udp_received_image_sync', 10)
        self.pub_sync = self.create_publisher(Image, '/udp_received_image', 10)
        self.pub_pc = self.create_publisher(PointCloud2, '/sync_points', 10)
        self.pub_tf = self.create_publisher(TransformStamped, '/sync_transform', 10)
        
        # Буферы для синхронизации
        self.pc_buf = collections.deque(maxlen=BUF_MAX)
        self.tf_buf = collections.deque(maxlen=BUF_MAX)
       
        # Подписки
        self.create_subscription(
            PointCloud2,
            '/synced/velodyne_points',
            self.pc_cb,
            10
        )
        
        self.create_subscription(
            TransformStamped,
            '/velodyne_saved_transform',
            self.tf_cb,
            10
        )

        self.bridge = CvBridge()
        self.create_timer(0.01, self.udp_callback)  # Таймер 100 Гц
        self.get_logger().info(f"Listening on {self.udp_ip}:{self.udp_port}")

    def pc_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pc_buf.append((t, msg))

    def tf_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tf_buf.append((t, msg))

    def udp_callback(self):
        try:
            data, _ = self.socket.recvfrom(65536)
            
            if len(data) < 8:
                self.get_logger().warn("Too small packet received")
                return
                
            # Извлекаем timestamp (первые 8 байт)
            sec, nsec = struct.unpack('>II', data[:8])
            jpeg_data = data[8:]
            ts_img = sec + nsec * 1e-9
            
            # Декодируем JPEG
            img = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().error("Failed to decode JPEG")
                return
                
            # Конвертируем в ROS сообщение
            try:
                img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                img_msg.header.stamp.sec = sec
                img_msg.header.stamp.nanosec = nsec
                img_msg.header.frame_id = 'external'
            except CvBridgeError as e:
                self.get_logger().error(f"CV Bridge error: {e}")
                return
            
            # Публикуем изображение
            self.pub_img.publish(img_msg)
            
            # Синхронизация с другими данными
            pc_msg = self._closest(self.pc_buf, ts_img)
            tf_msg = self._closest(self.tf_buf, ts_img)

            if (pc_msg is not None and tf_msg is not None and
                abs(pc_msg[0] - ts_img) <= SYNC_TOL and
                abs(tf_msg[0] - ts_img) <= SYNC_TOL):
                
                self.pub_sync.publish(img_msg)
                self.pub_pc.publish(pc_msg[1])
                self.pub_tf.publish(tf_msg[1])
                
                self.get_logger().info(
                    f"Synced: Δpc={abs(pc_msg[0]-ts_img)*1e3:.1f}ms "
                    f"Δtf={abs(tf_msg[0]-ts_img)*1e3:.1f}ms",
                    throttle_duration_sec=1.0
                )
                
                # Удаляем использованные сообщения
                self.pc_buf.remove(pc_msg)
                self.tf_buf.remove(tf_msg)
                
        except socket.timeout:
            # Это нормальное поведение при отсутствии данных
            pass
        except Exception as e:
            self.get_logger().error(f"UDP processing error: {e}")

    @staticmethod
    def _closest(buf, t_ref):
        """Найти ближайшее по времени сообщение в буфере"""
        return min(buf, key=lambda it: abs(it[0] - t_ref)) if buf else None

def main(args=None):
    rclpy.init(args=args)
    node = SimpleUdpReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()