#!/usr/bin/env python3
...

import rclpy
import tkinter
from tkinter import messagebox
from tkinter import Label
import json
from rclpy.node import Node

from std_msgs.msg import String
from lidar_auto_docking_messages.msg import Initdock
import math
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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

    def show_message(self, msg):
        messagebox.showinfo(title="Save Status", message=msg)

    def show_variable(self, msg, posx, posy):
        self.label = Label(self.top_, text=str(msg))
        self.label.place(x=posx, y=posy)


class dock_pose_subscriber(Node):
    def __init__(self):
        super().__init__("dock_subscriber")
        self.subscription = self.create_subscription(
            Initdock, "init_dock", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.declare_parameter("load_file_path")
        self.dock_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # Try to get the transform every second.
        # If the transform is unavailable the timer callback will wait for it.
        self._output_timer = self.create_timer(0.1, self.on_timer)
        self.obj_gui_ = gui()
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.obj_gui_.create_button(
            posx=250,
            posy=300,
            button_name="save dock and bot pose",
            button_colour="green",
            callback_function=self.save_dock_callback,
        )

    async def on_timer(self):
        from_frame = "base_link"
        to_frame = "map"

        # Get latest available
        when = rclpy.time.Time()

        try:
            # Suspends callback until transform becomes available
            self.robot_pose = await self._tf_buffer.lookup_transform_async(
                to_frame, from_frame, when
            )
            self.bot_x = self.robot_pose.transform.translation.x
            self.bot_y = self.robot_pose.transform.translation.y
            self.bot_z = self.robot_pose.transform.rotation.z
            self.bot_w = self.robot_pose.transform.rotation.w
            self.dock_x_diff = abs(self.bot_x - self.x_pos)
            self.dock_y_diff = abs(self.bot_y - self.y_pos)
            self.dist_to_dock = math.sqrt(
                self.dock_x_diff * self.dock_x_diff
                + self.dock_y_diff * self.dock_y_diff
            )
            self.obj_gui_.show_variable(
                posx=250, posy=250, msg="Distance to dock: " + str(self.dist_to_dock)
            )
        except LookupException as e:
            self.get_logger().error("failed to get transform {}".format(repr(e)))

    def save_dock_callback(self):
        print(self.x_pos)
        print(self.y_pos)
        print(self.z_pos)
        print(self.w_pos)
        output_dict = {}
        output_dict["x"] = self.x_pos
        output_dict["y"] = self.y_pos
        output_dict["z"] = self.z_pos
        output_dict["w"] = self.w_pos

        output_dict["bx"] = self.bot_x
        output_dict["by"] = self.bot_y
        output_dict["bz"] = self.bot_z
        output_dict["bw"] = self.bot_w
        with open(
            self.dock_file_path,
            "w",
        ) as outfile:
            json.dump(output_dict, outfile)
        self.obj_gui_.show_message("Dock and robot Coordinates have been saved!")

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
