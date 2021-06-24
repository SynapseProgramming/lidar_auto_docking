#include <lidar_auto_docking/tf2listener.h>

void tf2_listener::waitTransform(std::string origin, std::string destination) {
  std::string warning_msg;
  while (rclcpp::ok() &&
         !buffer_.canTransform(destination, origin, tf2::TimePoint(),
                               &warning_msg)) {
    std::cout << "waiting: " << warning_msg << "\n";
    rate.sleep();
  }
}
