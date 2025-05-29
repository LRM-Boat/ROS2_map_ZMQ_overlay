#!/usr/bin/env python3
"""
zmq_sync_node.py
– JPEG+timestamp   ← ZeroMQ
– PointCloud2      ← /synced/velodyne_points
– TransformStamped ← /velodyne_saved_transform

Накапливаем кольцевые буферы LiDAR и TF.
При каждом кадре ищем ближайшие по времени точки и трансформ;
если |Δt| ≤ SYNC_TOL → публикуем тройку.
"""
import time, struct, collections, zmq, cv2, numpy as np
import rclpy
from rclpy.node         import Node
from rclpy.duration     import Duration
from sensor_msgs.msg    import Image, PointCloud2
from geometry_msgs.msg  import TransformStamped
from cv_bridge          import CvBridge, CvBridgeError
from sensor_msgs_py     import point_cloud2 as pc2   # только ради подсказок IDE

# --------------------------------------------------------------------------- #
ZMQ_ADDR   = "tcp://127.0.0.1:5557"
SYNC_TOL   = 0.10                     # сек – допуск для синхронизации
BUF_LIFE   = 2.0                      # сколько храним сообщения, сек
BUF_MAX    = 200                      # макс. размер буфера
# --------------------------------------------------------------------------- #

# --------------------------------------------------------------------------- #
#парметры конвертации картинки модели
IMG_HEIGHT = 1401
IMG_WIDTH  = 1943
PALETTE = np.array([
    [196, 123, 55],   # background
    [65, 72, 74],     # gravel
    [69, 115, 153],   # dirt
    [173, 179, 184],  # asphalt
    [126, 194, 46],   # grass
    [184, 179, 173],  # penablock
    [2, 2, 3],        # wall
], dtype=np.uint8)


class ZMQSyncNode(Node):

    def __init__(self):
        super().__init__('zmq_sync_node')
        self.bridge = CvBridge()

        # --- паблишеры -----------------------------------------------------
        self.pub_img  = self.create_publisher(Image,           '/camera/external_image', 10)
        self.pub_sync = self.create_publisher(Image,           '/camera/external_sync',  10)
        self.pub_pc   = self.create_publisher(PointCloud2,     '/sync_points',           10)
        self.pub_tf   = self.create_publisher(TransformStamped,'/sync_transform',        10)

        # --- буферы --------------------------------------------------------
        self.pc_buf = collections.deque(maxlen=BUF_MAX)  # (t, PointCloud2)
        self.tf_buf = collections.deque(maxlen=BUF_MAX)  # (t, TransformStamped)

        # --- подписки ------------------------------------------------------
        self.create_subscription(PointCloud2,
                                 '/synced/velodyne_points',
                                 self.pc_cb, 10)

        self.create_subscription(TransformStamped,
                                 '/velodyne_saved_transform',
                                 self.tf_cb, 10)

        # --- ZeroMQ --------------------------------------------------------
        ctx = zmq.Context.instance()
        self.sub = ctx.socket(zmq.SUB)
        self.sub.connect(ZMQ_ADDR)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt(zmq.LINGER,   0)

        # --- таймеры -------------------------------------------------------
        self.create_timer(0.02, self.poll_zmq)      # опрос кадров
        self.create_timer(1.0,  self.prune_buffers) # чистка буферов

        self.get_logger().info("ZMQ-Sync node initialised")

    # =================================================================== #
    # callbacks
    # ------------------------------------------------------------------- #
    def pc_cb(self, msg: PointCloud2):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pc_buf.append((t, msg))

    def tf_cb(self, msg: TransformStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tf_buf.append((t, msg))

    # ------------------------------------------------------------------- #
    def prune_buffers(self):
        """Удаляем устаревшие сообщения из буферов."""
        now = time.time()
        for buf in (self.pc_buf, self.tf_buf):
            while buf and (now - buf[0][0] > BUF_LIFE):
                buf.popleft()

    # ------------------------------------------------------------------- #
    def poll_zmq(self):
        try:
            data = self.sub.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        if len(data) < 8:
            return

        # Извлекаем timestamp
        sec, nsec = struct.unpack(">II", data[:8])
        ts_img = sec + nsec * 1e-9
        mask_data = data[8:]

        # Восстанавливаем маску классов (uint8) из байтов
        expected_size = IMG_HEIGHT * IMG_WIDTH
        if len(mask_data) != expected_size:
            self.get_logger().warn(f"Invalid mask size: expected {expected_size}, got {len(mask_data)}")
            return

        mask = np.frombuffer(mask_data, dtype=np.uint8).reshape((IMG_HEIGHT, IMG_WIDTH))

        # Преобразуем маску в цветное изображение
        color_image = PALETTE[mask]  # (H, W) → (H, W, 3)

        # Преобразуем в ROS Image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Bridge error: {e}")
            return

        img_msg.header.stamp.sec = sec
        img_msg.header.stamp.nanosec = nsec
        img_msg.header.frame_id = "external"

        # Публикуем как обычное изображение
        self.pub_img.publish(img_msg)

        # --- синхронизация с LiDAR и TF ---
        pc_msg = self._closest(self.pc_buf, ts_img)
        tf_msg = self._closest(self.tf_buf, ts_img)

        if (pc_msg is not None and tf_msg is not None and
                abs(pc_msg[0] - ts_img) <= SYNC_TOL and
                abs(tf_msg[0] - ts_img) <= SYNC_TOL):
            self.pub_sync.publish(img_msg)
            self.pub_pc.publish(pc_msg[1])
            self.pub_tf.publish(tf_msg[1])
            self.get_logger().info(
                f"Sync Δpc={abs(pc_msg[0]-ts_img)*1e3:.1f} ms, "
                f"Δtf={abs(tf_msg[0]-ts_img)*1e3:.1f} ms")
            self.pc_buf.remove(pc_msg)
            self.tf_buf.remove(tf_msg)

    # =================================================================== #
    @staticmethod
    def _closest(buf, t_ref):
        """Вернуть элемент буфера с минимальным |Δt| (или None)."""
        return min(buf, key=lambda it: abs(it[0] - t_ref)) if buf else None


# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = ZMQSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
