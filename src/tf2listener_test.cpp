#include "lidar_auto_docking/tf2listener.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("ros_publisher"), obj(this->get_clock()) {}

  void print_transformations(
      geometry_msgs::msg::TransformStamped echo_transform) {
    auto translation = echo_transform.transform.translation;
    auto rotation = echo_transform.transform.rotation;
    std::cout << "- Translation: [" << translation.x << ", " << translation.y
              << ", " << translation.z << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << rotation.x << ", "
              << rotation.y << ", " << rotation.z << ", " << rotation.w << "]"
              << std::endl;
  }

  void wait_for_transform() {
    obj.waitTransform(origin, destination);
    std::cout << "GOT TRANSFORMATIONS\n";
    print_transformations(obj.getTransform(origin, destination));
  }

  void print_pose_stamped(geometry_msgs::msg::PoseStamped posee) {
    auto position = posee.pose.position;
    auto orientation = posee.pose.orientation;
    std::cout << "px: " << position.x << " py: " << position.y << "\n";
    std::cout << "- Rotation: in Quaternion [" << orientation.x << ", "
              << orientation.y << ", " << orientation.z << ", " << orientation.w
              << "]" << std::endl;
  }

  void tpose_test() {
    geometry_msgs::msg::PoseStamped dock;
    dock.header.frame_id = "fake_frame";
    rclcpp::Time time_now = rclcpp::Clock().now();
    dock.header.stamp = time_now;
    dock.pose.position.x = 5;
    dock.pose.position.y = 0;
    dock.pose.orientation.w = 1;
    std::cout << "BEFORE TRANSFORMATION aka still wrt fake_frame\n ";
    print_pose_stamped(dock);
    // wait for transformation from map to fake_frame
    obj.waitTransform("map", "fake_frame");
    // once we can confirm that the frames exist, then we can test out the
    // transformation
    obj.transformPose("map", dock, dock);

    std::cout << "AFTER TRANSFORMATION should be  wrt map\n ";
    print_pose_stamped(dock);
  }

 private:
  tf2_listener obj;
  std::string origin = "map";
  std::string destination = "fake_frame";
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MinimalPublisher> min_ptr =
      std::make_shared<MinimalPublisher>();

  // min_ptr->wait_for_transform();
  min_ptr->tpose_test();
  rclcpp::spin(min_ptr);
  rclcpp::shutdown();
  return 0;
}
