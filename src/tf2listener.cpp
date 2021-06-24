#include <lidar_auto_docking/tf2listener.h>

void tf2_listener::waitTransform(std::string source, std::string target) {
  std::string warning_msg;
  while (rclcpp::ok() && !buffer_.canTransform(source, target, tf2::TimePoint(),
                                               &warning_msg)) {
    std::cout << "waiting: " << warning_msg << "\n";
    rate.sleep();
  }
}
