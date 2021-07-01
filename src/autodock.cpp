#include <lidar_auto_docking/controller.h>
#include <lidar_auto_docking/perception.h>

#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <string>
#include <thread>

#include "lidar_auto_docking/action/dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std::chrono_literals;

class DockingServer : public rclcpp::Node {
 public:
  using Dock = lidar_auto_docking::action::Dock;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;

  explicit DockingServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("docking_server", options), NUM_OF_RETRIES_(5) {
    using namespace std::placeholders;
    // initialise the action server object

    // the string is the action topic
    this->action_server_ = rclcpp_action::create_server<Dock>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "Dock",
        std::bind(&DockingServer::handle_goal, this, _1, _2),
        std::bind(&DockingServer::handle_cancel, this, _1),
        std::bind(&DockingServer::handle_accepted, this, _1));
  }

  void init_objects() {
    std::shared_ptr<rclcpp::Node> new_ptr = shared_ptr_from_this();
    perception_ = std::make_shared<DockPerception>(new_ptr);
    controller_ = std::make_shared<BaseController>(new_ptr);
  }

  // shared_ptr_from_this would return a shared pointer of the current class
  std::shared_ptr<rclcpp::Node> shared_ptr_from_this() {
    return shared_from_this();
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

  // this function initialises the dock timeout.
  void initDockTimeout() {
    // get current time and fill up the header
    rclcpp::Time time_now = rclcpp::Clock().now();
    // TODO: Change duration to 120 once it is stable.
    deadline_docking_ = time_now + rclcpp::Duration(5s);
    num_of_retries_ = NUM_OF_RETRIES_;
  }

  bool isDockingTimedOut() {
    // Have we exceeded our deadline or tries?
    rclcpp::Time time_now = rclcpp::Clock().now();
    if (time_now > deadline_docking_ || !num_of_retries_) {
      return true;
    }
    return false;
  }

  // main function which is called when a goal is received
  void execute(const std::shared_ptr<GoalHandleDock> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Dock::Feedback>();
    auto result = std::make_shared<Dock::Result>();
    // start perception
    perception_->start(goal->dock_pose);

    // TODO: TEST TIMEOUT FUNCTION
    initDockTimeout();
    while (!isDockingTimedOut()) {
      std::cout << "Docking not timed out yet!\n";
      loop_rate.sleep();
    }
    std::cout << "DOCKING TIMED OUT!\n";
    /*
        // count up from 1 to 10
        for (int i = 1; i <= 10 && rclcpp::ok(); ++i) {
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
          feedback->dock_pose.pose.position.x = i +
       goal->dock_pose.pose.position.x; feedback->dock_pose.pose.position.y
       = i
       + 1; feedback->dock_pose.pose.position.z = i + 2;
          feedback->command.linear.x = i;
          feedback->command.linear.y = i + 1;
          feedback->command.linear.z = i + 2;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Publish Feedback");

          loop_rate.sleep();
        }
    */
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

  // Configuration Constants.
  int NUM_OF_RETRIES_;  // Number of times the robot gets to attempt
  double DOCK_CONNECTOR_CLEARANCE_DISTANCE_;  // The amount to back off in order
                                              // to clear the dock connector.
  double DOCKED_DISTANCE_THRESHOLD_;  // Threshold distance that indicates that
                                      // the robot might be docked

  std::shared_ptr<DockPerception> perception_;
  std::shared_ptr<BaseController> controller_;
  // determine if charging
  bool charging_;

  // Failure detection
  double abort_distance_;    // Distance below which to check abort criteria.
  double abort_threshold_;   // Y-offset that triggers abort.
  double abort_angle_;       // Angle offset that triggers abort.
  double correction_angle_;  // Yaw correction angle the robot should use to
                             // line up with the dock.
  double
      backup_limit_;  // Maximum distance the robot will backup when trying to
                      // retry. Based on range of initial dock pose estimate.
  bool aborting_;  // If the robot realizes it won't be sucessful, it needs to
                   // abort.
  int num_of_retries_;   // The number of times the robot gets to abort before
                         // failing. This variable will count down.
  bool cancel_docking_;  // Signal that docking has failed and the action server
                         // should abort the goal.
  rclcpp::Time deadline_docking_;       // Time when the docking times out.
  rclcpp::Time deadline_not_charging_;  // Time when robot gives up on the
  // charge
  // state and retries docking.
  bool charging_timeout_set_;  // Flag to indicate if the deadline_not_charging
                               // has been set.

};  // class DockingServer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<DockingServer>();
  action_server->init_objects();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
