#include <lidar_auto_docking/tf2listener.h>

void tf2_listener::waitTransform(std::string origin, std::string destination) {
  std::string warning_msg;
  while (rclcpp::ok() &&
         !buffer_.canTransform(origin, destination, tf2::TimePoint(),
                               &warning_msg)) {
    std::cout << "waiting: " << warning_msg << "\n";
    rate.sleep();
  }
}

geometry_msgs::msg::TransformStamped tf2_listener::getTransform(
    std::string origin, std::string destination) {
  geometry_msgs::msg::TransformStamped echo_transform;
  echo_transform =
      buffer_.lookupTransform(origin, destination, tf2::TimePoint());

  return echo_transform;
}
