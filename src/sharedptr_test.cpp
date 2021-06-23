#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;

class SepPublisher {
 public:
  SepPublisher() {}

  void init_node(std::shared_ptr<rclcpp::Node> main_ptr) {
    main_ptr_ = main_ptr;

    publisher_ =
        main_ptr_->create_publisher<std_msgs::msg::Int32>("/sepclass_int", 10);

    timer_ = main_ptr_->create_wall_timer(500ms, [this]() {
      // create string obj.
      auto message = std_msgs::msg::Int32();
      message.data = this->count;
      //  RCLCPP_INFO(main_ptr_->get_logger(), "Publishing: '%d'",
      //  message.data);
      // publish message
      publisher_->publish(message);
      this->count += 10;
    });
  }

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
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      // publish message
      publisher_->publish(message);
      this->count++;
    });
  }
  void init_objects() {
    std::shared_ptr<rclcpp::Node> new_ptr = shared_ptr_from_this();
    obj.init_node(new_ptr);
  }

  std::shared_ptr<rclcpp::Node> shared_ptr_from_this() {
    return shared_from_this();
  }

 private:
  int count = 0;
  // shared ptr of a timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  SepPublisher obj;
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
