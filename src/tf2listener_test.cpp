#include "lidar_auto_docking/tf2listener.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("ros_publisher"), obj(this->get_clock()) {}

  void wait_for_transform() {
    obj.waitTransform(origin, destination);
    std::cout << "GOT TRANSFORMATIONS";
    geometry_msgs::msg::TransformStamped echo_transform;
    echo_transform = obj.getTransform(origin, destination);

    auto translation = echo_transform.transform.translation;
    auto rotation = echo_transform.transform.rotation;
    std::cout << "- Translation: [" << translation.x << ", " << translation.y
              << ", " << translation.z << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << rotation.x << ", "
              << rotation.y << ", " << rotation.z << ", " << rotation.w << "]"
              << std::endl;
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
