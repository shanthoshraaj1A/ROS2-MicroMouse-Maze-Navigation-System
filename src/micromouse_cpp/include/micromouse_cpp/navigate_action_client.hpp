#pragma once
/**
 * @file navigate_action_client.hpp
 * @brief Action Client for MicroMouse Navigation
 *
 * Demonstrates how to:
 * - Send goals to an action server
 * - Receive feedback during execution
 * - Handle results (success/failure/cancellation)
 * - Cancel goals with Ctrl+C
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "micromouse_interfaces/action/navigate_to_goal.hpp"

namespace micromouse {

using NavigateToGoal = micromouse_interfaces::action::NavigateToGoal;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToGoal>;

/**
 * @brief Action Client Node for MicroMouse Navigation
 *
 * Usage:
 *   ros2 run micromouse_cpp navigate_action_client --ros-args -p goal_x:=7 -p
 * goal_y:=7
 *
 * Press Ctrl+C to cancel navigation while in progress.
 */
class NavigateActionClient : public rclcpp::Node {
 public:
  /**
   * @brief Constructor - creates action client and sets up signal handler
   */
  NavigateActionClient();

  /**
   * @brief Send navigation goal to the action server
   */
  void send_goal();

  /**
   * @brief Cancel the current goal (called by signal handler)
   *
   * Sends a cancel request to the action server if a goal is active.
   * If no goal is active, shuts down the node.
   */
  void cancel_goal();

 private:
  rclcpp_action::Client<NavigateToGoal>::SharedPtr action_client_;

  /// Store goal handle for cancellation support
  GoalHandleNavigate::SharedPtr goal_handle_;

  /**
   * @brief Callback when goal is accepted or rejected
   * @param goal_handle Handle to the goal
   */
  void goal_response_callback(const GoalHandleNavigate::SharedPtr& goal_handle);

  /**
   * @brief Callback for feedback during navigation
   * @param goal_handle Handle to the goal
   * @param feedback Current navigation feedback
   */
  void feedback_callback(
      GoalHandleNavigate::SharedPtr goal_handle,
      const std::shared_ptr<const NavigateToGoal::Feedback> feedback);

  /**
   * @brief Callback when navigation completes
   * @param result Final result of navigation
   */
  void result_callback(const GoalHandleNavigate::WrappedResult& result);
};

}  // namespace micromouse
