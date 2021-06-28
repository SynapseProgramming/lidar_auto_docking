
#include <lidar_auto_docking/perception.h>

#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("perception_test"), rate(10) {}

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
    perception_ptr->start(init_dock_pose);
    timer_ = this->create_wall_timer(50ms, [this]() {
      // if no dock is found yet, call start function with init_dock_pose to let
      // perception assume the dock is 1m ahead of the bot.
      if (this->perception_ptr->getPose(this->dock_pose, "base_link") ==
              false &&
          this->found_dockk == false) {
        std::cout << "still finding dock!\n";

        this->perception_ptr->start(this->init_dock_pose);

      } else {
        this->found_dockk = true;
        std::cout << "wrt base_link";
        std::cout << "x: " << this->dock_pose.pose.position.x
                  << " y: " << this->dock_pose.pose.position.y;
        std::cout << " z: " << this->dock_pose.pose.orientation.z
                  << " w: " << this->dock_pose.pose.orientation.w << "\n";
      }
    });
  }

 private:
  std::shared_ptr<DockPerception> perception_ptr;
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
  std::shared_ptr<MinimalPublisher> min_ptr =
      std::make_shared<MinimalPublisher>();

  min_ptr->init_objects();
  min_ptr->main_test();
  rclcpp::spin(min_ptr);
  rclcpp::shutdown();
  return 0;
}