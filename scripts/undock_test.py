#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from lidar_auto_docking.action import Undock

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class UndockClient(Node):
    def __init__(self):
        super().__init__("undock_client")
        self._action_client = ActionClient(self, Undock, "Undock")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(str("Test"))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            print(str(result.undocked))
        else:
            self.get_logger().info("Goal failed with status: {0}".format(status))
            self.get_logger().info(str(result.undocked))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        goal_msg = Undock.Goal()
        goal_msg.rotate_in_place = True
        # fill up the rest later
        self.get_logger().info("Sending goal request...")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = UndockClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
