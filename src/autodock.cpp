#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <string>
#include <thread>

#include "lidar_auto_docking/action/dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DockingServer : public rclcpp::Node {
 public:
  //  using Fibonacci = example_interfaces::action::Fibonacci;
  using Dock = lidar_auto_docking::action::Dock;
  //  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;

  explicit DockingServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("docking_server", options) {
    using namespace std::placeholders;
    // initialise the action server object

    this->action_server_ = rclcpp_action::create_server<Dock>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "Dock",
        std::bind(&DockingServer::handle_goal, this, _1, _2),
        std::bind(&DockingServer::handle_cancel, this, _1),
        std::bind(&DockingServer::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::Server<Dock>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Dock::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request!");
    (void)uuid;
    // Let's reject sequences that are over 9000
    // if (goal->order > 9000) {
    //  return rclcpp_action::GoalResponse::REJECT;
    //}
    // TODO: maybe code in a function to reject some invalid dock goal
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleDock> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleDock> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Dock::Feedback>();
    auto result = std::make_shared<Dock::Result>();
    // for our implementation, we would just count up for dock_pose and command.

    // count up from 1 to 20
    for (int i = 1; i <= 20 && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->docked = false;
        result->dock_id = "HEHEE";
        // return the current result
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // TODO: add in result as numerical base
      feedback->dock_pose.pose.position.x = i;
      feedback->dock_pose.pose.position.y = i + 1;
      feedback->dock_pose.pose.position.z = i + 2;
      feedback->command.linear.x = i;
      feedback->command.linear.y = i + 1;
      feedback->command.linear.z = i + 2;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      // result->sequence = sequence;
      result->docked = true;
      result->dock_id = "YAY";
      // return succeeded
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&DockingServer::execute, this, _1), goal_handle}
        .detach();
  }
};  // class DockingServer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<DockingServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
