import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class RGBExtractorNode(Node):
    def __init__(self):
        super().__init__('rgb_extractor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/external_sync',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Конвертируем ROS Image в OpenCV BGR numpy array
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Конвертируем BGR в RGB
        rgb_img = cv_img[:, :, ::-1]

        # Переводим в 2D массив, где каждая строка — один пиксель (R,G,B)
        pixels = rgb_img.reshape(-1, 3)

        # Получаем уникальные цвета (RGB)
        unique_colors = np.unique(pixels, axis=0)

        self.get_logger().info(f"Unique RGB colors count: {len(unique_colors)}")
        self.get_logger().info(f"Unique colors sample (up to 10): {unique_colors[:10]}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
