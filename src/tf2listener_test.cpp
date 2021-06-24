#include "lidar_auto_docking/tf2listener.h"

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("ros_publisher"), obj(this->get_clock()) {}

  // the init_objects function would return a shared_ptr to this class. It will
  // be stored in new_ptr before being passed to
  // the init_node function of Sepclass to initialise its memeber functions.
  void init_objects() {
    std::shared_ptr<rclcpp::Node> new_ptr = shared_ptr_from_this();
    // TODO: declare tf2_listener here
    //  obj.init_listener(new_ptr);
    // test frames here;
    //  obj.waitTransform(origin, destination);
  }
  // clock = node_ptr_->get_clock();
  // shared_ptr_from_this would return a shared pointer of the current class
  std::shared_ptr<rclcpp::Node> shared_ptr_from_this() {
    return shared_from_this();
  }

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

  //  min_ptr->init_objects();
  min_ptr->wait_for_transform();
  rclcpp::spin(min_ptr);
  rclcpp::shutdown();
  return 0;
}
