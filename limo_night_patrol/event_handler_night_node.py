import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class EventHandlerNightNode(Node):
    def __init__(self):
        super().__init__('event_handler_night_node')

        self.sub_depth_obstacle = self.create_subscription(
            Bool,
            'depth_obstacles',
            self.depth_obstacle_callback,
            10
        )
        self.sub_intruder = self.create_subscription(
            Bool,
            'night_intruder',
            self.intruder_callback,
            10
        )

        self.pub_event = self.create_publisher(String, 'patrol_event', 10)

        self.cli_capture = self.create_client(Trigger, 'camera/capture')
        while not self.cli_capture.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for camera/capture service...')

        # 쿨다운용 플래그
        self.last_capture_time = self.get_clock().now()
        self.cooldown_sec = 10.0

        self.get_logger().info('EventHandlerNightNode started')

    def depth_obstacle_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Obstacle detected by depth. Trigger capture.')
            self.handle_event('NIGHT_OBSTACLE')

    def intruder_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Night intruder detected. Trigger capture.')
            self.handle_event('NIGHT_INTRUDER')

    def handle_event(self, event_type: str):
        now = self.get_clock().now()
        dt = (now - self.last_capture_time).nanoseconds * 1e-9
        if dt < self.cooldown_sec:
            # 너무 자주 찍지 않도록
            self.get_logger().info(f'Skipping capture due to cooldown ({dt:.1f}s)')
            return

        self.last_capture_time = now

        # 1) 사진 촬영 서비스 호출
        req = Trigger.Request()
        future = self.cli_capture.call_async(req)

        def _done(fut):
            try:
                res = fut.result()
                if res.success:
                    self.get_logger().info(f'Capture OK: {res.message}')
                else:
                    self.get_logger().warn(f'Capture failed: {res.message}')
            except Exception as e:
                self.get_logger().error(f'Capture service call failed: {e}')

        future.add_done_callback(_done)

        # 2) 이벤트 토픽 퍼블리시
        msg = String()
        msg.data = event_type
        self.pub_event.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EventHandlerNightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
