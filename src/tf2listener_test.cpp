#include "lidar_auto_docking/tf2listener.h"

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("ros_publisher"), obj(this->get_clock()) {}

  void wait_for_transform() {
    obj.waitTransform(origin, destination);
    std::cout << "DO STUFF!\n";
  }

 private:
  tf2_listener obj;
  std::string origin = "odom";
  std::string destination = "base_link";
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MinimalPublisher> min_ptr =
      std::make_shared<MinimalPublisher>();

  min_ptr->wait_for_transform();
  rclcpp::spin(min_ptr);
  rclcpp::shutdown();
  return 0;
}
