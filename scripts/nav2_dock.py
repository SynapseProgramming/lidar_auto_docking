#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class goto_pose:
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
        else:
            print("Goal Failed!")
            self.goal_status = False

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

    def get_status(self):
        status = {}
        status["gs"] = self.goal_status
        status["gas"] = self.goal_accept_status
        return status

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False


class MainLogic(Node):
    def __init__(self):
        super().__init__("dock_logic")
        self.dock_cmd = 0
        self.timer = self.create_timer(0.5, self.timed_callback)
        self.subscription = self.create_subscription(
            Int32, "dock_cmd", self.update_cmd, 10
        )
        self.goto_pose_ = goto_pose(
            ActionClient(self, NavigateToPose, "navigate_to_pose")
        )

    def update_cmd(self, msg):
        self.dock_cmd = msg.data

    def timed_callback(self):
        nav2_status = self.goto_pose_.get_status()
        print("dock_cmd: " + str(self.dock_cmd))
        print("goal status: " + str(nav2_status["gs"]))
        print("goal accept status: " + str(nav2_status["gas"]))
        if nav2_status["gs"] == True and nav2_status["gas"] == True:
            print("Resetting goal status!")
            self.goto_pose_.reset_status()

    def send_goal(self):
        goal = {}
        goal["x"] = -1.5149
        goal["y"] = 0.962
        goal["z"] = -0.753
        goal["w"] = 0.6577
        self.goto_pose_.send_goal(goal)


def main(args=None):
    rclpy.init(args=args)

    action_client = MainLogic()

    # action_client.send_goal()

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
