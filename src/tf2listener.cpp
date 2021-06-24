#include <lidar_auto_docking/tf2listener.h>

/*
void tf2_listener::init_listener(std::shared_ptr<rclcpp::Node> node_ptr) {
  node_ptr_ = node_ptr;
  clock = node_ptr_->get_clock();
  buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_.get());
  //  tf2_ros::Buffer buffer_(clock);
}
*/

void tf2_listener::waitTransform(std::string source, std::string target) {
  std::string warning_msg;
  while (rclcpp::ok() && !buffer_.canTransform(source, target, tf2::TimePoint(),
                                               &warning_msg)) {
    //  RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *clock, 1000,
    //                     "Waiting for transform %s ->  %s: %s",
    //                     source.c_str(),
    //                   target.c_str(), warning_msg.c_str());
    std::cout << "waiting " << warning_msg << "\n";
    rate.sleep();
  }
  std::cout << "FOUND TRANSFORM";
}
