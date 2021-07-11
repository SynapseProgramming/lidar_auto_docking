#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class goto_pose:
    def __init__(self, action_client):
        self._action_client = action_client

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal Rejected")
            return

        print("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Goal Succeeded!")
        else:
            print("Goal Failed!")

    def send_goal(self, goal_pose):
        print("Waiting for action server")
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = goal_pose["x"]
        goal_msg.pose.pose.position.y = goal_pose["y"]
        goal_msg.pose.pose.orientation.z = goal_pose["z"]
        goal_msg.pose.pose.orientation.w = goal_pose["w"]
        # fill up the rest later
        print("Sending goal request")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


class UndockClient(Node):
    def __init__(self):
        super().__init__("nav2_client")
        self.goto_pose_ = goto_pose(
            ActionClient(self, NavigateToPose, "navigate_to_pose")
        )

    def send_goal(self):
        goal = {}
        goal["x"] = -1.5149
        goal["y"] = 0.962
        goal["z"] = -0.753
        goal["w"] = 0.6577
        self.goto_pose_.send_goal(goal)


def main(args=None):
    rclpy.init(args=args)

    action_client = UndockClient()

    action_client.send_goal()

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
