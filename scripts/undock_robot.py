#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from lidar_auto_docking.action import Undock

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class undocking_client:
    def __init__(self, action_client):
        self._action_client = action_client
        self.goal_status = False
        self.goal_accept_status = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal Rejected")
            self.goal_accept_status = False
            return

        print("Goal accepted")
        self.goal_accept_status = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Goal Succeeded!")
            self.goal_status = True
            print(str(result.undocked))
        else:
            print("Goal Failed!")
            self.goal_status = False

    def send_goal(self):
        print("Waiting for action server")
        self._action_client.wait_for_server()
        goal_msg = Undock.Goal()
        goal_msg.rotate_in_place = True
        print("Sending goal request")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_status(self):
        status = {}
        status["gs"] = self.goal_status
        status["gas"] = self.goal_accept_status
        return status

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False


class UndockClient(Node):
    def __init__(self):
        super().__init__("undock_client")
        self.undocking_client_ = undocking_client(ActionClient(self, Undock, "Undock"))

    def send_goal(self):
        self.undocking_client_.send_goal()


def main(args=None):
    rclpy.init(args=args)

    action_client = UndockClient()

    action_client.send_goal()

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
