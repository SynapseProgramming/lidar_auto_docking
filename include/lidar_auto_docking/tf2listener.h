#ifndef LIDAR_AUTO_DOCKING_TF2LISTENER_H
#define LIDAR_AUTO_DOCKING_TF2LISTENER_H

#include <geometry_msgs/msg/transform_stamped.h>
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

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class tf2_listener {
 public:
  tf2_listener(rclcpp::Clock::SharedPtr clock) : buffer_(clock), rate(1) {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }
  // init_listener initialises node_ptr_, which allows the class to access node
  // functions.
  // also initialises the other members which require node functions
  // void init_listener(std::shared_ptr<rclcpp::Node> node_ptr);

  void waitTransform(std::string source, std::string target);

 private:
  //  std::shared_ptr<tf2_ros::Buffer> buffer_;
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  // local node_ptr
  std::shared_ptr<rclcpp::Node> node_ptr_;
  //  rclcpp::Clock::SharedPtr clock;
  rclcpp::Rate rate;

  std::string source_frameid;
  std::string target_frameid;
};

#endif  // TF2LISTENER_H
