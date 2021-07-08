#!/usr/bin/env python3
...

import rclpy
import tkinter
from tkinter import messagebox
import json
from rclpy.node import Node

from std_msgs.msg import String
from lidar_auto_docking.msg import Initdock
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
        self.obj_gui_.create_button(
            posx=250,
            posy=300,
            button_name="save_dock_pose",
            button_colour="green",
            callback_function=self.save_dock_callback,
        )

    async def on_timer(self):
        from_frame = "map"
        to_frame = "base_link"

        self.get_logger().info(
            "Waiting for transform from {} to {}".format(from_frame, to_frame)
        )

        # Get latest available
        when = rclpy.time.Time()

        try:
            # Suspends callback until transform becomes available
            self.transform = await self._tf_buffer.lookup_transform_async(
                to_frame, from_frame, when
            )
            self.get_logger().info("Got {}".format(repr(self.transform)))
        except LookupException as e:
            self.get_logger().error("failed to get transform {}".format(repr(e)))

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
        with open(
            self.dock_file_path,
            "w",
        ) as outfile:
            json.dump(initial_dock_pose, outfile)
        self.obj_gui_.show_message("Dock Coordinates have been saved!")

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
