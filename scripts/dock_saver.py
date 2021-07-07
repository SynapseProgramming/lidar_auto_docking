#!/usr/bin/env python3
...

import rclpy
import tkinter
import json
from rclpy.node import Node

from std_msgs.msg import String
from lidar_auto_docking.msg import Initdock


class gui(object):
    def __init__(self):

        self.top_ = tkinter.Tk()
        self.top_.geometry("600x500")

    def get_selected_goal(self):
        return self.selected_goal_.get()

    # callback_function is the address of the function to callback when the button is pressed.

    def create_button(self, posx, posy, button_name, callback_function, button_colour):
        self.send_goal_button_ = tkinter.Button(
            self.top_, text=button_name, command=callback_function, bg=button_colour
        )
        self.send_goal_button_.place(x=posx, y=posy)

    def update_tk(self):
        self.top_.update_idletasks()
        self.top_.update()


class dock_pose_subscriber(Node):
    def __init__(self):
        super().__init__("dock_subscriber")
        self.subscription = self.create_subscription(
            Initdock, "init_dock", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.obj_gui_ = gui()
        self.obj_gui_.create_button(
            posx=100,
            posy=300,
            button_name="save_dock_pose",
            button_colour="green",
            callback_function=self.save_dock_callback,
        )

    def save_dock_callback(self):
        print(self.x_pos)
        print(self.y_pos)
        print(self.z_pos)
        print(self.w_pos)
        initial_dock_pose = {}
        initial_dock_pose["x"] = self.x_pos
        initial_dock_pose["y"] = self.y_pos
        initial_dock_pose["z"] = self.z_pos
        initial_dock_pose["w"] = self.w_pos
        # TODO: the filepath should be loaded from a parameter (specified in the launch file)
        with open(
            "/home/ro/dev_ws/src/lidar_auto_docking/initial_dock_pose/init_dock.json",
            "w",
        ) as outfile:
            json.dump(initial_dock_pose, outfile)

    def listener_callback(self, msg):
        # we should update our tkinter gui with the current dock coordinates here
        self.x_pos = msg.x
        self.z_pos = msg.z
        self.y_pos = msg.y
        self.w_pos = msg.w
        # refresh tkinter
        self.obj_gui_.update_tk()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = dock_pose_subscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
