/*
 * Copyright 2015 Fetch Robotics Inc
 * Author: Michael Ferguson
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIDAR_AUTO_DOCKING_CONTROLLER_H
#define LIDAR_AUTO_DOCKING_CONTROLLER_H

#include <tf2/utils.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include "lidar_auto_docking/tf2listener.h"

class BaseController {
 public:
  // explicit BaseController(ros::NodeHandle& nh);

  /**
   * @brief Implements something loosely based on "A Smooth Control Law for
   * Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments"
   * by Park and Kuipers, ICRA 2011
   * @returns true if base has reached goal.
   */
  bool approach(const geometry_msgs::msg::PoseStamped& target);

  /**
   * @brief Back off dock, then rotate. Robot is first reversed by the
   *        prescribed distance, and then rotates with respect to it's current
   *        orientation.
   * @param distance        Distance in meters to backup.
   * @param rotate_distance Amount of angle in radians for the robot to yaw.
   */
  bool backup(double distance, double rotate_distance);

  /**
   * @brief Get the last command sent
   */
  bool getCommand(geometry_msgs::msg::Twist& command);

  /** @brief send stop command to robot base */
  void stop();

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  tf2_listener listener_;
  geometry_msgs::msg::Twist command_;
  /*
   * Parameters for approach controller
   */

  double k1_;  // ratio in change of theta to rate of change in r
  double k2_;  // speed at which we converge to slow system
  double min_velocity_;
  double max_velocity_;
  double max_angular_velocity_;
  double beta_;    // how fast velocity drops as k increases
  double lambda_;  // ??
  double dist_;    // used to create the tracking line

  /*
   * Parameters for backup controller
   */

  geometry_msgs::msg::PoseStamped start_;
  bool ready_;
  bool turning_;
};

#endif  // FETCH_AUTO_DOCK_CONTROLLER_H
