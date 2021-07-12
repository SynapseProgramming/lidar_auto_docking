#!/usr/bin/env python3
...


from action_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from lidar_auto_docking.action import Dock
from lidar_auto_docking.action import Undock
from nav2_msgs.action import NavigateToPose

import json
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


class docking_client:
    def __init__(self, action_client):
        self._action_client = action_client
        self.goal_status = False
        self.goal_accept_status = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("goal rejected")
            self.goal_accept_status = False
            return

        print("goal accepted")
        self.goal_accept_status = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("goal succeeded")
            self.goal_status = True
        else:
            print("goal failed ")
            self.goal_status = False

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

    def get_status(self):
        status = {}
        status["gs"] = self.goal_status
        status["gas"] = self.goal_accept_status
        return status

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False


class goto_pose:
    def __init__(self, action_client):
        self._action_client = action_client
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("nav2 Goal Rejected")
            self.goal_accept_status = False
            return

        print("nav2 Goal accepted")
        self.goal_accept_status = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("navigation succeeded")
            self.goal_status = True
        else:
            print("navigation failed!")
            self.goal_status = False
            self.failure_flag = True

    def send_goal(self, goal_pose):
        print("Waiting for action server")
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = goal_pose["bx"]
        goal_msg.pose.pose.position.y = goal_pose["by"]
        goal_msg.pose.pose.orientation.z = goal_pose["bz"]
        goal_msg.pose.pose.orientation.w = goal_pose["bw"]
        # fill up the rest later
        print("Sending goal request")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_status(self):
        status = {}
        status["gs"] = self.goal_status
        status["gas"] = self.goal_accept_status
        status["f_flag"] = self.failure_flag
        return status

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False


class MainLogic(Node):
    def __init__(self):
        super().__init__("dock_logic")
        self.dock_cmd = 0
        self.dock_stat = 0
        self.timer = self.create_timer(0.1, self.timed_callback)
        self.subscription = self.create_subscription(
            Int32, "dock_cmd", self.update_cmd, 10
        )
        self.status_publisher = self.create_publisher(Int32, "dock_status", 10)
        self.goto_pose_ = goto_pose(
            ActionClient(self, NavigateToPose, "navigate_to_pose")
        )
        self.docking_client_ = docking_client(ActionClient(self, Dock, "Dock"))
        self.declare_parameter("load_file_path")

        self.undocking_client_ = undocking_client(ActionClient(self, Undock, "Undock"))
        self.dock_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        # load in dock coordinates
        with open(self.dock_file_path) as outfile:
            self.initial_poses = json.load(outfile)

    def update_cmd(self, msg):
        self.dock_cmd = msg.data

    def timed_callback(self):
        # update status of all classes
        nav2_status = self.goto_pose_.get_status()
        dock_status = self.docking_client_.get_status()
        undock_status = self.undocking_client_.get_status()

        # check failure flags
        if nav2_status["f_flag"] == True and self.dock_stat != 5:
            print("Docking failure!")
            self.dock_stat = 5
        # Docking command received. Navigate to initial goal pose first
        elif self.dock_cmd == 1 and self.dock_stat == 0:

            self.goto_pose_.send_goal(self.initial_poses)
            self.dock_stat = 1
        # Engage in docking sequence once the robot has reached the goal
        elif (
            self.dock_stat == 1
            and nav2_status["gs"] == True
            and nav2_status["gas"] == True
        ):
            print("docking robot!")
            self.docking_client_.send_goal(self.initial_poses)
            self.dock_stat = 2
        # once the robot has docked, we will wait for an input to undock.
        elif (
            self.dock_stat == 2
            and dock_status["gs"] == True
            and dock_status["gas"] == True
        ):
            print("Waiting for undock command!")
            self.dock_stat = 3
        # Undock command received. Will initiate undocking sequence.
        elif self.dock_stat == 3 and self.dock_cmd == 2:
            print("undocking")
            self.undocking_client_.send_goal()
            self.dock_stat = 4
        # Once undocking sequence is complete, reset all boolean flags.
        elif (
            self.dock_stat == 4
            and undock_status["gs"] == True
            and undock_status["gas"] == True
        ):
            print("Docking Sequence Complete")
            self.dock_stat = 0
            self.undocking_client_.reset_status()
            self.docking_client_.reset_status()
            self.goto_pose_.reset_status()
        elif self.dock_stat == 5 and self.dock_cmd == -1:
            print("Resetting docking state!")
            self.dock_stat = 0
            self.undocking_client_.reset_status()
            self.docking_client_.reset_status()
            self.goto_pose_.reset_status()

        self.pub_dock_status(self.dock_stat)

    def pub_dock_status(self, stat):
        msg = Int32()
        msg.data = stat
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = MainLogic()

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
