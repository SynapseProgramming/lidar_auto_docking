#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from lidar_auto_docking.action import Dock

import rclpy
import json
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

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("goal succeeded")
        else:
            print("goal failed ")

    def send_goal(self, dock_pose):
        print("waiting for action server")
        self._action_client.wait_for_server()
        goal_msg = Dock.Goal()
        goal_msg.dock_pose.pose.position.x = dock_pose["x"]
        goal_msg.dock_pose.pose.position.y = dock_pose["y"]
        goal_msg.dock_pose.pose.orientation.z = dock_pose["z"]
        goal_msg.dock_pose.pose.orientation.w = dock_pose["w"]
        goal_msg.dock_pose.header.frame_id = "map"
        # fill up the rest later
        print("Sending goal request")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


class dock(Node):
    def __init__(self):
        super().__init__("minimal_action_client")
        self.docking_client_ = docking_client(ActionClient(self, Dock, "Dock"))
        self.declare_parameter("load_file_path")
        self.dock_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        # load in dock coordinates
        with open(self.dock_file_path) as outfile:
            self.initial_dock_pose = json.load(outfile)

    def send_goal(self):
        self.docking_client_.send_goal(self.initial_dock_pose)


def main(args=None):
    rclpy.init(args=args)

    action_client = dock()

    action_client.send_goal()

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
