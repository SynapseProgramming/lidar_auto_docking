
#include <lidar_auto_docking/perception.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "lidar_auto_docking/msg/initdock.hpp"
#include "lidar_auto_docking/tf2listener.h"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class DockCoordinates : public rclcpp::Node {
 public:
  DockCoordinates() : Node("dock_coordinates"), tf2_listen(this->get_clock()) {
    tbr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    publisher_ = this->create_publisher<lidar_auto_docking::msg::Initdock>(
        "init_dock", 10);
    this->declare_parameter<int>("reset_goal_button", 3);
    this->get_parameter("reset_goal_button", reset_goal_button);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
          std::vector<int> pressed_buttons = msg->buttons;

          if (pressed_buttons[reset_goal_button]) {
            RCLCPP_INFO(this->get_logger(), "Resetting initial dock estimate");
            this->found_dockk = false;
            this->perception_ptr->stop();
            update_init_dock(init_dock_pose);
            perception_ptr->start(init_dock_pose);
          }
        });
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
  // debugging function to print out the initial dock pose
  void print_idp(geometry_msgs::msg::PoseStamped idp) {
    std::cout << "init x: " << idp.pose.position.x << "\n";
    std::cout << "init y: " << idp.pose.position.y << "\n";
    std::cout << "init z: " << idp.pose.orientation.z << "\n";
    std::cout << "init w: " << idp.pose.orientation.w << "\n";
  }

  // this function would update the initial dock pose to be 1m from robot.
  // wrt map frame.
  void update_init_dock(geometry_msgs::msg::PoseStamped& idp) {
    tf2_listen.waitTransform("map", "base_link");
    geometry_msgs::msg::PoseStamped fake_dock;
    // take it that the fake dock is 1m in front of the robot.
    fake_dock.header.frame_id = "base_link";
    fake_dock.pose.position.x = 1;
    // we will transform fake_dock wrt map
    tf2_listen.transformPose("map", fake_dock, idp);
  }

  void main_test() {
    update_init_dock(init_dock_pose);
    perception_ptr->start(init_dock_pose);
    timer_ = this->create_wall_timer(20ms, [this]() {
      // if no dock is found yet, call start function with init_dock_pose to let
      // perception assume the dock is 1m ahead of the bot.
      if (this->perception_ptr->getPose(this->dock_pose, "map") == false &&
          this->found_dockk == false) {
        std::cout << "still finding dock!\n";
        this->update_init_dock(this->init_dock_pose);
        this->perception_ptr->start(this->init_dock_pose);

      } else {
        this->found_dockk = true;

        // publish the transformations of the dock
        auto dock_pose_msg = lidar_auto_docking::msg::Initdock();
        dock_pose_msg.x = this->dock_pose.pose.position.x;
        dock_pose_msg.y = this->dock_pose.pose.position.y;
        dock_pose_msg.z = this->dock_pose.pose.orientation.z;
        dock_pose_msg.w = this->dock_pose.pose.orientation.w;
        publisher_->publish(dock_pose_msg);
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
  int reset_goal_button;
  std::shared_ptr<DockPerception> perception_ptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tbr;
  rclcpp::Publisher<lidar_auto_docking::msg::Initdock>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  tf2_listener tf2_listen;

  rclcpp::Time time_now;
  geometry_msgs::msg::TransformStamped transformStamped;
  geometry_msgs::msg::PoseStamped dock_pose;
  geometry_msgs::msg::PoseStamped init_dock_pose;
  bool found_dockk = false;
  rclcpp::TimerBase::SharedPtr timer_;
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
