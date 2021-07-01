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

void tf2_listener::transformPose(std::string tracking_frame,
                                 geometry_msgs::msg::PoseStamped &input_pose,
                                 geometry_msgs::msg::PoseStamped &output_pose) {
  // firstly, get the correct desired transformation.
  geometry_msgs::msg::TransformStamped corrective_transform;
  corrective_transform =
      getTransform(tracking_frame, input_pose.header.frame_id);
  // transform the input pose to be referenced to the main tracking frame.
  tf2::doTransform(input_pose, output_pose, corrective_transform);
}
