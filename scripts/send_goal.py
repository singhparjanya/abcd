#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self.namespaces = {}
        self.action_clients = {}
        self.goal_handles = {}

        # Subscribe to the final_goal topic
        self.create_subscription(String, 'final_goal', self.final_goal_callback, 10)

    def final_goal_callback(self, msg):
        # Assume the message format is 'namespace x y theta'
        command = msg.data.strip()
        parts = command.split()

        if len(parts) == 4:
            namespace, x, y, theta = parts
            x, y, theta = float(x), float(y), float(theta)
            self.set_goal(namespace, x, y, theta)
        else:
            self.get_logger().error(f"Invalid command format: {command}")

    def set_goal(self, namespace, x, y, theta):
        if namespace not in self.action_clients:
            self.action_clients[namespace] = ActionClient(self, NavigateToPose, f'/{namespace}/navigate_to_pose')

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta  # Assuming theta is given as yaw
        pose.pose.orientation.w = 1.0  # This assumes no roll/pitch, only yaw

        goal_msg.pose = pose

        self.action_clients[namespace].wait_for_server()
        send_goal_future = self.action_clients[namespace].send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, namespace))

    def goal_response_callback(self, future, namespace):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected for {namespace}')
            return

        self.goal_handles[namespace] = goal_handle
        self.get_logger().info(f'Goal accepted for {namespace}')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def cancel_goal_callback(self, request, response):
        for namespace, goal_handle in self.goal_handles.items():
            if goal_handle:
                self.get_logger().info(f'Cancelling goal for {namespace}')
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_response_callback)
        response.success = True
        response.message = "Goals cancelled"
        return response

    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info('Goal cancelled successfully')
        else:
            self.get_logger().info('Failed to cancel goal')


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseClient()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

