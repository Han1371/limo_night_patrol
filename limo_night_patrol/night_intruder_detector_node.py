import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge


class NightIntruderDetectorNode(Node):
    def __init__(self):
        super().__init__('night_intruder_detector_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.pub_intruder = self.create_publisher(Bool, 'night_intruder', 10)

        self.declare_parameter('intruder_distance_max', 2.0)  # 2m 이내
        self.declare_parameter('intruder_pixel_threshold', 500)  # 최소 픽셀 수

        self.get_logger().info('NightIntruderDetectorNode started')

    def depth_callback(self, msg: Image):
        try:
            if msg.encoding in ['32FC1', '32FC']:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                depth_array = np.array(depth, dtype=np.float32)
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_array = np.array(depth, dtype=np.float32) / 1000.0
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')
            return

        h, w = depth_array.shape
        h1, h2 = int(h * 1/3), int(h * 2/3)
        w1, w2 = int(w * 1/3), int(w * 2/3)
        roi = depth_array[h1:h2, w1:w2]

        valid = np.isfinite(roi) & (roi > 0.0)
        valid_depths = roi[valid]

        if valid_depths.size == 0:
            intruder = False
        else:
            dist_max = self.get_parameter('intruder_distance_max').value
            close_pixels = valid_depths < dist_max
            num_close = int(np.sum(close_pixels))
            intruder_pixel_th = self.get_parameter('intruder_pixel_threshold').value
            intruder = num_close > intruder_pixel_th

        msg_out = Bool()
        msg_out.data = intruder
        self.pub_intruder.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = NightIntruderDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
