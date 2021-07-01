#include <angles/angles.h>
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
  // TODO: ADD IN RECOFIGURABLE PARAMETERS FOR DOCKED DISTANCE THRESHOLD
  explicit DockingServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("docking_server", options),
        NUM_OF_RETRIES_(5),
        DOCKED_DISTANCE_THRESHOLD_(0.55) {
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
    // TODO: change the timeout back to 120
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

  bool continueDocking(std::shared_ptr<Dock::Result> result,
                       const std::shared_ptr<GoalHandleDock> goal_handle) {
    // If charging, stop and return success.

    if (charging_) {
      result->docked = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "DOCK REACHED!");
      return false;
    }

    // Timeout on time or retries.
    else if (isDockingTimedOut() || cancel_docking_) {
      result->docked = false;
      // return the current result
      RCLCPP_INFO(this->get_logger(), "Docking Cancelled");
      return false;
    }

    else if (goal_handle->is_canceling()) {
      return false;
    }

    return true;
  }

  bool backupDistance() {
    // Initialized to 1.0 meter as our basic backup amount.
    double distance = 1.0;

    // Distance should be proportional to the amount of yaw correction.
    // The constants are purely arbitrary because they seemed good at the time.
    // distance *= 3.5*fabs(correction_angle_);
    distance *= 1.5 * fabs(correction_angle_);
    // We should backup more the more times we try. This function should range
    // from 1 to 2. num_of_retries is initially equal to NUM_OF_RETRIES and
    // decrements as the robot retries.
    double retry_constant =
        2 - static_cast<float>(num_of_retries_) / NUM_OF_RETRIES_;
    retry_constant = std::max(1.0, std::min(2.0, retry_constant));
    distance *= retry_constant;

    // Cap the backup limit to 1 meter, just in case.
    backup_limit_ = std::min(1.0, backup_limit_);
    // Threshold distance.
    distance = std::max(0.2, std::min(backup_limit_, distance));

    return distance;
  }

  // method to reverse the robot when the abort flag is set.
  void executeBackupSequence(rclcpp::Rate& r) {
    RCLCPP_ERROR(this->get_logger(), "Poor Approach! Backing up!");
    // Get off of the dock. Try to straighten out.
    while (!controller_->backup(DOCK_CONNECTOR_CLEARANCE_DISTANCE_,
                                correction_angle_)) {
      if (isDockingTimedOut()) {
        return;
      }
      r.sleep();  // Sleep the rate control object.
    }
    // Move to recovery pose.
    while (!controller_->backup(backupDistance(), 0.0)) {
      if (isDockingTimedOut()) {
        return;
      }
      r.sleep();
    }
  }

  bool isApproachBad(double& dock_yaw) {
    // Grab the dock pose in the base_link so we can evaluate it wrt the robot.
    geometry_msgs::msg::PoseStamped dock_pose_base_link;
    perception_->getPose(dock_pose_base_link, "base_link");

    dock_yaw = angles::normalize_angle(
        tf2::getYaw(dock_pose_base_link.pose.orientation));

    // If we are close to the dock but not quite docked, check other approach
    // parameters.
    if (dock_pose_base_link.pose.position.x < abort_distance_ &&
        dock_pose_base_link.pose.position.x > DOCKED_DISTANCE_THRESHOLD_) {
      // Check to see if we are too far side-to-side or at a bad angle.
      if (fabs(dock_pose_base_link.pose.position.y) > abort_threshold_ ||
          fabs(dock_yaw) > abort_angle_) {
        // Things are bad, abort.
        return true;
      }
    }
    // Everything is ok.
    return false;
  }

  // main function which is called when a goal is received
  void execute(const std::shared_ptr<GoalHandleDock> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(50);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Dock::Feedback>();
    auto result = std::make_shared<Dock::Result>();

    // Reset flags.
    result->docked = false;
    aborting_ = false;
    charging_timeout_set_ = false;
    cancel_docking_ = false;
    charging_ = false;

    // start perception
    perception_->start(goal->dock_pose);
    // start the timeout counter
    initDockTimeout();
    // get the first dock pose wrt base link.
    geometry_msgs::msg::PoseStamped dock_pose_base_link;
    RCLCPP_INFO(this->get_logger(), "Finding Dock");

    while (!perception_->getPose(dock_pose_base_link, "base_link")) {
      if (!continueDocking(result, goal_handle)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Docking failed: Initial dock not found.");
        break;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Pre-orienting the bot!");
    double dock_yaw = angles::normalize_angle(
        tf2::getYaw(dock_pose_base_link.pose.orientation));
    if (!std::isfinite(dock_yaw)) {
      RCLCPP_ERROR(this->get_logger(), "Dock yaw is invalid.");
      std::cout << "The invalid yaw value is: " << dock_yaw << std::endl;
      cancel_docking_ = true;
    } else if (rclcpp::ok() && continueDocking(result, goal_handle)) {
      // Set backup limit to be the initial dock range.
      backup_limit_ =
          std::sqrt(std::pow(dock_pose_base_link.pose.position.x, 2) +
                    std::pow(dock_pose_base_link.pose.position.y, 2));
      // Shorten up the range a bit.
      backup_limit_ *= 0.9;

      // Preorient the robot towards the dock.
      while (!controller_->backup(0.0, dock_yaw) &&
             continueDocking(result, goal_handle) && rclcpp::ok()) {
        loop_rate.sleep();  // Sleep the rate control object.
      }
    }
    // Make sure controller is ready
    controller_->stop();

    // main control which brings the bot to the dock
    while (rclcpp::ok() && continueDocking(result, goal_handle)) {
      // Update perception
      if (perception_->getPose(feedback->dock_pose)) {
        if (aborting_) {
          // backup
          executeBackupSequence(loop_rate);
          // Reset abort flag.
          aborting_ = false;
          // Decrement the number of retries.
          num_of_retries_--;
        } else {
          if (isApproachBad(correction_angle_)) {
            // Not on target, abort, abort, abort!
            controller_->stop();
            aborting_ = true;
          } else {
            // do stuff here
            geometry_msgs::msg::PoseStamped dock_x_distance;
            perception_->getPose(dock_x_distance, "base_link");

            std::cout << "Distance to dock: " << dock_x_distance.pose.position.x
                      << std::endl;
            // remember to replace 0.55 with a reconfigurable parameter
            if (dock_x_distance.pose.position.x <= DOCKED_DISTANCE_THRESHOLD_) {
              charging_ = true;
            } else {
              charging_ = false;
            }
            // Update control
            controller_->approach(feedback->dock_pose);
            // Are we on the dock? Check charging timeout.
            // TODO: port over checkDockChargingConditions(maybe)
            // checkDockChargingConditions();
          }
        }
        // feedback
        controller_->getCommand(feedback->command);
        goal_handle->publish_feedback(feedback);
      }
      loop_rate.sleep();
    }
    // stop all movements once we are done docking
    controller_->stop();
    perception_->stop();

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
    /*
        if (goal_handle->is_canceling()) {
          result->docked = false;
          result->dock_id = "HEHEE";
          // return the current result
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal Canceled");
          return;
        }
    */
    // Check if goal is done
    /* NOTE: if this function is not returned, then it would default to a
       failure.
        if (rclcpp::ok()) {
          // result->sequence = sequence;
          result->docked = true;
          result->dock_id = "YAY";
          // return succeeded
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
        */
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up
    // a new thread
    std::thread{std::bind(&DockingServer::execute, this, _1), goal_handle}
        .detach();
  }

  // Configuration Constants.
  int NUM_OF_RETRIES_;  // Number of times the robot gets to attempt
  double DOCK_CONNECTOR_CLEARANCE_DISTANCE_;  // The amount to back off in order
                                              // to clear the dock connector.
  double DOCKED_DISTANCE_THRESHOLD_;  // Threshold distance that indicates
                                      // that the robot might be docked

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
  bool cancel_docking_;  // Signal that docking has failed and the action
                         // server should abort the goal.
  rclcpp::Time deadline_docking_;       // Time when the docking times out.
  rclcpp::Time deadline_not_charging_;  // Time when robot gives up on the
  // charge
  // state and retries docking.
  bool charging_timeout_set_;  // Flag to indicate if the
                               // deadline_not_charging has been set.
};                             // class DockingServer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<DockingServer>();
  action_server->init_objects();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
