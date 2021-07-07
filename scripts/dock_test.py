#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from lidar_auto_docking.action import Dock

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class docking_client:
    def __init__(self, action_client):
        self._action_client = action_client

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("goal rejected")
            return

        print("goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        print("feedback received")

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("goal succeeded")
        else:
            print("goal failed ")

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        print("waiting for action server")
        self._action_client.wait_for_server()
        # wrt base_linkx: 0.9966 y: -1.205732 z: -1.32998 w: -1.99998

        # wrt odomx: 0.92997 y: -1.215420 z: -1.808215 w: -1.999966
        goal_msg = Dock.Goal()
        goal_msg.dock_pose.pose.position.x = float(-1.5467)
        goal_msg.dock_pose.pose.position.y = float(-0.20825)
        goal_msg.dock_pose.pose.orientation.z = float(0.005627)
        goal_msg.dock_pose.pose.orientation.w = float(0.9999)
        goal_msg.dock_pose.header.frame_id = "map"
        # fill up the rest later
        print("Sending goal request")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)


class MinimalActionClient(Node):
    def __init__(self):
        super().__init__("minimal_action_client")
        self.docking_client_ = docking_client(ActionClient(self, Dock, "Dock"))

    def send_goal(self):
        self.docking_client_.send_goal()


def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
