#ifndef LIDAR_AUTO_DOCKING_TF2LISTENER_H
#define LIDAR_AUTO_DOCKING_TF2LISTENER_H

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

class tf2_listener {
 public:
  tf2_listener(rclcpp::Clock::SharedPtr clock) : buffer_(clock), rate(10) {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }
  // would block code till the transform is received.
  void waitTransform(std::string origin, std::string destination);

  // would return the transfomation from origin to destination as a transform
  // stamped
  geometry_msgs::msg::TransformStamped getTransform(std::string origin,
                                                    std::string destination);

  // would tranform the input pose to be with reference to the tracking frame.
  void transformPose(std::string tracking_frame,
                     geometry_msgs::msg::PoseStamped &input_pose,
                     geometry_msgs::msg::PoseStamped &output_pose);

 private:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::Rate rate;
};

#endif  // TF2LISTENER_H
