import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager_node')
        self.publisher_ = self.create_publisher(String, 'patrol_mode', 10)
        self.declare_parameter('mode', 'NIGHT')  # 기본값 NIGHT

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ModeManagerNode started')

    def timer_callback(self):
        msg = String()
        msg.data = self.get_parameter('mode').get_parameter_value().string_value
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
