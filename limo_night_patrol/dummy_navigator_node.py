# limo_night_patrol/limo_night_patrol/dummy_navigator_node.py
import rclpy
import asyncio
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose


class DummyNavigatorNode(Node):
    def __init__(self):
        super().__init__('dummy_navigator_node')

        # /cmd_vel/nav2 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/nav2', 10)

        # navigate_to_pose 액션 서버
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback
        )

        self.get_logger().info('DummyNavigatorNode started (no real Nav2)')

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request.pose
        self.get_logger().info(
            f'Received goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )

        # 3초 동안 앞으로 가는 cmd_vel/nav2 퍼블리시
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0

        period = 0.1
        steps = int(3.0 / period)
        for _ in range(steps):
            self.cmd_pub.publish(twist)
            await asyncio.sleep(period)

        # 정지
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

        self.get_logger().info('DummyNavigatorNode: goal “성공 처리”')

        result = NavigateToPose.Result()
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DummyNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
