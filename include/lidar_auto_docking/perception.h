/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Michael Ferguson, Sriramvarun Nidamarthy
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

#ifndef LIDAR_AUTO_DOCKING_PERCEPTION_H
#define LIDAR_AUTO_DOCKING_PERCEPTION_H

#include <geometry_msgs/msg/pose_stamped.h>
#include <lidar_auto_docking/dock_candidate.h>
#include <lidar_auto_docking/laser_processor.h>
#include <lidar_auto_docking/linear_pose_filter_2d.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

class DockPerception {
 public:
  //  explicit DockPerception(ros::NodeHandle& nh);

  /**
   * @brief Start dock detection.
   * @param pose The initial estimate of dock pose
   */
  // bool start(const geometry_msgs::PoseStamped& pose);

  /** @brief Stop tracking the dock. */
  // bool stop();

  /** @brief Get the pose of the dock. */
  // bool getPose(geometry_msgs::PoseStamped& pose, std::string frame = "");

 private:
  /** @brief Callback to process laser scans */
  //  void callback(const sensor_msgs::LaserScanConstPtr& scan);

  /**
   * @brief Extract a DockCandidate from a cluster, filling in the
   *        lengths and slopes of each line found using ransac.
   * @param cluster The pointer to the cluster to extract from.
   */
  // DockCandidatePtr extract(laser_processor::SampleSet* cluster);

  /**
   * @brief Try to fit a dock to candidate
   * @param candidate The candidate to fit to.
   * @param pose The fitted pose, if successful.
   * @returns Fitness score (>0 if successful)
   */
  // double fit(const DockCandidatePtr& candidate, geometry_msgs::Pose& pose);

  /**
   * @brief Method to check if the quaternion is valid.
   * @param q Quaternion to check.
   * @return True if quaternion is valid.
   */
  // static bool isValid(const tf::Quaternion& q);
  /*
    ros::Subscriber scan_sub_;
    tf::TransformListener listener_;

    bool running_;  // Should we be trying to find the dock
    bool debug_;    // Should we output debugging info

    LinearPoseFilter2DPtr
        dock_pose_filter_;  /// Low pass filter object for filtering dock poses.
//TODO: import the private member functions
    // TF frame to track dock within
    std::string tracking_frame_;
    // Best estimate of where the dock is
    geometry_msgs::PoseStamped dock_;
    // Mutex on dock_
    boost::mutex dock_mutex_;
    // If true, then dock_ is based on actual sensor data
    bool found_dock_;
    // Last time that dock pose was updated
    ros::Time dock_stamp_;
    // Maximum allowable error between scan and "ideal" scan
    double max_alignment_error_;

    // Publish visualization of which points are detected as dock
    ros::Publisher debug_points_;

    // The ideal cloud, located at origin of dock frame
    std::vector<geometry_msgs::Point> ideal_cloud_;
    // The ideal cloud (when only front is visible)
    std::vector<geometry_msgs::Point> front_cloud_;
    */
};

#endif  // LIDAR_AUTO_DOCKING_PERCEPTION_H
