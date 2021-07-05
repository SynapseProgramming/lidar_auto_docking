
#include <lidar_auto_docking/perception.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "lidar_auto_docking/msg/initdock.hpp"
#include "lidar_auto_docking/tf2listener.h"

using namespace std::chrono_literals;

class DockCoordinates : public rclcpp::Node {
 public:
  DockCoordinates()
      : Node("perception_test"), tf2_listen(this->get_clock()), rate(10) {
    tbr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // the init_objects function would return a shared_ptr to this class. It will
  // be stored in new_ptr before being passed to
  // the init_node function of Sepclass to initialise its memeber functions.
  void init_objects() {
    std::shared_ptr<rclcpp::Node> new_ptr = shared_ptr_from_this();
    perception_ptr = std::make_shared<DockPerception>(new_ptr);
  }

  // shared_ptr_from_this would return a shared pointer of the current class
  std::shared_ptr<rclcpp::Node> shared_ptr_from_this() {
    return shared_from_this();
  }

  void main_test() {
    // we will assume that the dock is placed 1m in front of the robot
    tf2_listen.waitTransform("map", "base_link");
    geometry_msgs::msg::TransformStamped robot_pose;
    robot_pose = tf2_listen.getTransform("map", "base_link");
    auto translation = robot_pose.transform.translation;
    auto rotation = robot_pose.transform.rotation;
    // we will set the dock to be 1m in front of the robot
    init_dock_pose.pose.position.x = translation.x + 1;
    init_dock_pose.pose.position.y = translation.y;
    init_dock_pose.pose.position.z = translation.z;
    init_dock_pose.pose.orientation = rotation;
    init_dock_pose.header.frame_id = "map";
    perception_ptr->start(init_dock_pose);
    timer_ = this->create_wall_timer(20ms, [this]() {
      // if no dock is found yet, call start function with init_dock_pose to let
      // perception assume the dock is 1m ahead of the bot.
      if (this->perception_ptr->getPose(this->dock_pose, "map") == false &&
          this->found_dockk == false) {
        std::cout << "still finding dock!\n";

        this->perception_ptr->start(this->init_dock_pose);

      } else {
        this->found_dockk = true;
        std::cout << "wrt map ";
        std::cout << "x: " << this->dock_pose.pose.position.x
                  << " y: " << this->dock_pose.pose.position.y;
        std::cout << " z: " << this->dock_pose.pose.orientation.z
                  << " w: " << this->dock_pose.pose.orientation.w << "\n";

        // Also, send the transform for visualisation
        this->time_now = rclcpp::Clock().now();

        this->transformStamped.header.stamp = this->time_now;
        this->transformStamped.header.frame_id = "map";
        this->transformStamped.child_frame_id = "dock";
        this->transformStamped.transform.translation.x =
            this->dock_pose.pose.position.x;
        this->transformStamped.transform.translation.y =
            this->dock_pose.pose.position.y;
        this->transformStamped.transform.translation.z = 0.0;
        this->transformStamped.transform.rotation.x = 0;
        this->transformStamped.transform.rotation.y = 0;
        this->transformStamped.transform.rotation.z =
            this->dock_pose.pose.orientation.z;
        this->transformStamped.transform.rotation.w =
            this->dock_pose.pose.orientation.w;

        this->tbr->sendTransform(this->transformStamped);
      }
    });
  }

 private:
  std::shared_ptr<DockPerception> perception_ptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tbr;
  tf2_listener tf2_listen;

  rclcpp::Time time_now;
  geometry_msgs::msg::TransformStamped transformStamped;
  geometry_msgs::msg::PoseStamped dock_pose;
  // init_dock_pose has an empty quaternion. This would force perception to
  // assume that the dock 1m directly ahead of the robot.
  geometry_msgs::msg::PoseStamped init_dock_pose;
  bool found_dockk = false;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Rate rate;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DockCoordinates> min_ptr =
      std::make_shared<DockCoordinates>();

  min_ptr->init_objects();
  min_ptr->main_test();
  rclcpp::spin(min_ptr);
  rclcpp::shutdown();
  return 0;
}
