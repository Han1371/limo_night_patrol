import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge


class DepthObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('depth_obstacle_detector_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.pub_obstacle = self.create_publisher(Bool, 'depth_obstacles', 10)

        # 파라미터
        self.declare_parameter('obstacle_distance_threshold', 0.8)  # [m]
        self.declare_parameter('obstacle_ratio_threshold', 0.02)    # 2% 이상 픽셀이 가까우면 장애물

        self.get_logger().info('DepthObstacleDetectorNode started')

    def depth_callback(self, msg: Image):
        try:
            # depth 인코딩이 32FC1 또는 16UC1일 수 있음
            if msg.encoding in ['32FC1', '32FC']:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                depth_array = np.array(depth, dtype=np.float32)
            else:
                # 보통 16UC1 (mm)
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_array = np.array(depth, dtype=np.float32) / 1000.0  # m 로 변환
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')
            return

        # 중앙 영역만 사용 (이미지 가운데 1/3x1/3 영역)
        h, w = depth_array.shape
        h1, h2 = int(h * 1/3), int(h * 2/3)
        w1, w2 = int(w * 1/3), int(w * 2/3)
        roi = depth_array[h1:h2, w1:w2]

        # invalid 값 제거 (0 또는 NaN)
        valid = np.isfinite(roi) & (roi > 0.0)
        valid_depths = roi[valid]
        if valid_depths.size == 0:
            obstacle = False
        else:
            dist_th = self.get_parameter('obstacle_distance_threshold').value
            ratio_th = self.get_parameter('obstacle_ratio_threshold').value

            close_pixels = valid_depths < dist_th
            close_ratio = float(np.sum(close_pixels)) / float(valid_depths.size)

            obstacle = close_ratio > ratio_th

        msg_out = Bool()
        msg_out.data = obstacle
        self.pub_obstacle.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
