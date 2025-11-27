import os
import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraCaptureServerNode(Node):
    def __init__(self):
        super().__init__('camera_capture_server_node')
        self.bridge = CvBridge()
        self.latest_image = None

        self.declare_parameter('save_dir', '/tmp/limo_captures')

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.srv = self.create_service(
            Trigger,
            'camera/capture',
            self.capture_callback
        )

        self.get_logger().info('CameraCaptureServerNode started')

    def image_callback(self, msg: Image):
        self.latest_image = msg

    def capture_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet'
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            response.success = False
            response.message = f'cv_bridge error: {e}'
            return response

        save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        os.makedirs(save_dir, exist_ok=True)

        now = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(save_dir, f'night_capture_{now}.png')

        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Saved capture to {filename}')

        response.success = True
        response.message = filename
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
