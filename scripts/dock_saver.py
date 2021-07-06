#!/usr/bin/env python3
...

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from lidar_auto_docking.msg import Initdock


class dock_pose_subscriber(Node):
    def __init__(self):
        super().__init__("dock_subscriber")
        self.subscription = self.create_subscription(
            Initdock, "init_dock", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('x: "%s"' % str(msg.x))
        self.get_logger().info('y: "%s"' % str(msg.y))
        self.get_logger().info('z: "%s"' % str(msg.z))
        self.get_logger().info('w: "%s"' % str(msg.w))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = dock_pose_subscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
