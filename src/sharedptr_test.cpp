#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;

class SepPublisher {
 public:
  SepPublisher(std::shared_ptr<rclcpp::Node> main_ptr) {
    // copy over the main_ptr to a local shared_ptr
    main_ptr_ = main_ptr;
    // init the publisher pointer
    publisher_ =
        main_ptr_->create_publisher<std_msgs::msg::Int32>("/sepclass_int", 10);
    // we would still have to pass (this) pointer to the lamda function to allow
    // it to access all memeber functions inside the class
    timer_ = main_ptr_->create_wall_timer(500ms, [this]() {
      // create string obj.
      auto message = std_msgs::msg::Int32();
      message.data = this->count;
      //  RCLCPP_INFO(main_ptr_->get_logger(), "Publishing: '%d'",
      //  message.data);

      std::cout << "Sep Class : " << message.data << "\n";
      // publish message
      publisher_->publish(message);
      this->count += 10;
    });
  }

  // This function would initialise
  // void init_node(std::shared_ptr<rclcpp::Node> main_ptr) {}

 private:
  int count = 0;
  // shared ptr of a timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::Node> main_ptr_;
};

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("ros_publisher") {
    publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/mainclass_int", 10);

    timer_ = this->create_wall_timer(500ms, [this]() {
      // create string obj.
      auto message = std_msgs::msg::Int32();
      message.data = this->count;
      //  RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      std::cout << "main class: " << message.data << "\n";
      // publish message
      publisher_->publish(message);
      this->count++;
    });
  }

  // the init_objects function would return a shared_ptr to this class. It will
  // be stored in new_ptr before being passed to
  // the init_node function of Sepclass to initialise its memeber functions.
  void init_objects() {
    std::shared_ptr<rclcpp::Node> new_ptr = shared_ptr_from_this();
    obj_ptr = std::make_shared<SepPublisher>(new_ptr);
  }

  // shared_ptr_from_this would return a shared pointer of the current class
  std::shared_ptr<rclcpp::Node> shared_ptr_from_this() {
    return shared_from_this();
  }

 private:
  int count = 0;
  // shared ptr of a timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std::shared_ptr<SepPublisher> obj_ptr;
  // SepPublisher obj;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MinimalPublisher> min_ptr =
      std::make_shared<MinimalPublisher>();

  min_ptr->init_objects();

  rclcpp::spin(min_ptr);
  rclcpp::shutdown();
  return 0;
}
