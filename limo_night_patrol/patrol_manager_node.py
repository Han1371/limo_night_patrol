import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class PatrolManagerNode(Node):
    def __init__(self):
        super().__init__('patrol_manager_node')

        # 모드 / 이벤트 구독 (지금은 로그용)
        self.sub_mode = self.create_subscription(
            String,
            'patrol_mode',
            self.mode_callback,
            10
        )
        self.sub_event = self.create_subscription(
            String,
            'patrol_event',
            self.event_callback,
            10
        )

        # Nav2 액션 클라이언트
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        # 간단한 waypoint 리스트 (예시) – 나중에 YAML 파라미터로 교체 가능
        self.waypoints = self.create_default_waypoints()
        self.current_index = 0
        self.current_goal_handle = None
        self.is_sending_goal = False

        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('PatrolManagerNode started')

    def create_default_waypoints(self):
        # map 좌표계에서 (x, y, yaw) 리스트 – 프로젝트 환경에 맞게 수정
        coords = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi/2),
            (0.0, 1.0, math.pi),
        ]
        waypoints = []
        for x, y, yaw in coords:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            # yaw -> quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            waypoints.append(ps)
        return waypoints

    def mode_callback(self, msg: String):
        # NIGHT 모드인지 확인 (지금은 로그만)
        self.get_logger().info(f'Patrol mode: {msg.data}')

    def event_callback(self, msg: String):
        # 밤 모드 이벤트 수신 – 로그만 남김
        self.get_logger().info(f'Patrol event received: {msg.data}')

    def timer_callback(self):
        if self.is_sending_goal:
            # 이미 목표를 향해 가는 중
            return

        if not self.waypoints:
            return

        # 다음 waypoint로 이동
        goal_pose = self.waypoints[self.current_index]
        self.send_goal(goal_pose)

        # 다음 인덱스로
        self.current_index = (self.current_index + 1) % len(self.waypoints)

    def send_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        self.is_sending_goal = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_sending_goal = False
            return

        self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Goal result received. status={status}, result={result}')
        self.is_sending_goal = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 필요하면 여기서 거리, 진행률 등 로그 찍을 수 있음
        # self.get_logger().info(f'Current pose: {feedback.current_pose.pose.position.x:.2f}, ...')
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
