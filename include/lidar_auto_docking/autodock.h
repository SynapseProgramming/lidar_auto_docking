#ifndef LIDAR_AUTO_DOCKING_AUTODOCK_H
#define LIDAR_AUTO_DOCKING_AUTODOCK_H

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
#include "lidar_auto_docking/action/undock.hpp"
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
        DOCKED_DISTANCE_THRESHOLD_(0.30),
        abort_distance_(0.32),
        abort_angle_(0.090),
        abort_threshold_(0.04) {
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

  // init_objects function creates instances of helper classes.
  void init_objects();

  // shared_ptr_from_this would return a shared pointer of the current class
  std::shared_ptr<rclcpp::Node> shared_ptr_from_this();

 private:
  rclcpp_action::Server<Dock>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Dock::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleDock> goal_handle);

  // function which is called to spin a new thread to run execute function
  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle);

  /**
   * @brief Method sets the docking deadline and number of retries.
   */
  void initDockTimeout();

  /**
   * @brief Method checks to see if we have run out of time or retries.
   * @return True if we are out of time or tries.
   */
  bool isDockingTimedOut();

  /**
   * @brief Method that checks success or failure of docking.
   * @param result Dock result message used to set the dock action server state.
   * @return True if we have neither succeeded nor failed to dock.
   */
  bool continueDocking(std::shared_ptr<Dock::Result> result,
                       const std::shared_ptr<GoalHandleDock> goal_handle);

  /**
   * @brief Method to compute the distance the robot should backup when
   * attemping a docking correction. Method uses a number of state variables in
   * the class to compute distance. TODO(enhancement): Should these be
   * parameterized instead?
   * @return Distance for robot to backup in meters.
   */
  bool backupDistance();

  // method to reverse the robot when the abort flag is set.
  void executeBackupSequence(rclcpp::Rate& r);

  /**
   * @brief Method to check approach abort conditions. If we are close to the
   * dock but the robot is too far off side-to-side or at a bad angle, it should
   *        abort. Method also returns through the parameter the orientation of
   * the dock wrt the robot for use in correction behaviors.
   * @param dock_yaw Yaw angle of the dock wrt the robot in radians.
   * @return True if the robot should abort the approach.
   */
  bool isApproachBad(double& dock_yaw);

  // main function which is called when a goal is received
  void execute(const std::shared_ptr<GoalHandleDock> goal_handle);

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

class UndockingServer : public rclcpp::Node {
 public:
  using Undock = lidar_auto_docking::action::Undock;
  using GoalHandleUndock = rclcpp_action::ServerGoalHandle<Undock>;
  // TODO: ADD IN RECOFIGURABLE PARAMETERS FOR DOCKED DISTANCE THRESHOLD
  explicit UndockingServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("Undocking_server", options),
        DOCK_CONNECTOR_CLEARANCE_DISTANCE_(0.4) {
    using namespace std::placeholders;
    // initialise the action server object
    // the string is the action topic
    /*
    this->undock_action_server_ = rclcpp_action::create_server<Unock>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "Undock",
        std::bind(&UndockingServer::handle_goal, this, _1, _2),
        std::bind(&UndockingServer::handle_cancel, this, _1),
        std::bind(&UndockingServer::handle_accepted, this, _1));
      */
  }

 private:
  rclcpp_action::Server<Undock>::SharedPtr undock_action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Undock::Goal> goal);
  /*
      rclcpp_action::CancelResponse handle_cancel(
          const std::shared_ptr<GoalHandleUndock> goal_handle);

      // function which is called to spin a new thread to run execute function
      void handle_accepted(const std::shared_ptr<GoalHandleUndock> goal_handle);

      // main function which is called when a goal is received
      void execute(const std::shared_ptr<GoalHandleUndock> goal_handle);
    */
  double DOCK_CONNECTOR_CLEARANCE_DISTANCE_;  // The amount to back off in order
                                              // to clear the dock
};

#endif
