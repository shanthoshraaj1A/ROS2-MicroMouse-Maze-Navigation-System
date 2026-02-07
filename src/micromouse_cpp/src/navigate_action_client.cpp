/**
 * @file navigate_action_client.cpp
 * @brief Implementation of Action Client for MicroMouse Navigation
 *
 * @details
 * This file demonstrates how to:
 * - Create a ROS2 action client
 * - Send goals to an action server
 * - Handle feedback, results, and cancellation
 *
 * @authors
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#include "micromouse_cpp/navigate_action_client.hpp"

#include <csignal>

using namespace std::chrono_literals;

namespace micromouse {

// =============================================================================
// Global for signal handler (PROVIDED)
// =============================================================================
// Signal handlers cannot be member functions, so we need a global pointer
// to the client instance to forward the cancel request.

NavigateActionClient* g_client_instance = nullptr;

/**
 * @brief Signal handler for SIGINT (Ctrl+C) (PROVIDED)
 *
 * When Ctrl+C is pressed, this handler forwards the cancel request
 * to the action client instead of immediately terminating the process.
 */
void signal_handler(int signum) {
  (void)signum;  // Suppress unused parameter warning
  if (g_client_instance) {
    g_client_instance->cancel_goal();
  }
}

// =============================================================================
// Constructor (PROVIDED)
// =============================================================================

NavigateActionClient::NavigateActionClient() : Node("navigate_action_client") {
  // Declare parameters
  this->declare_parameter("goal_x", 7);
  this->declare_parameter("goal_y", 7);

  // Create action client
  action_client_ =
      rclcpp_action::create_client<NavigateToGoal>(this, "/navigate_to_goal");

  // Set up signal handler for Ctrl+C cancellation
  g_client_instance = this;
  std::signal(SIGINT, signal_handler);

  RCLCPP_INFO(this->get_logger(),
              "Action client created for /navigate_to_goal");
}

// =============================================================================
// send_goal
// =============================================================================
/**
 * @brief Function for sending goal
 * 
 * 1. Wait for action server (10s timeout)
 * 2. Create goal message from parameters.
 * 3. Sets up goal options with callbacks.
 * 4. Sends the goal message. 
 * 
 */
void NavigateActionClient::send_goal() {
  // Wait for action server
  if (!action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server /navigate_to_goal not available after 10s");
    rclcpp::shutdown();
    return;
  }

  // -------------------------------------------------------------------------
  // Goal message and set goal coordinates
  // -------------------------------------------------------------------------

  auto goal_msg   = NavigateToGoal::Goal();
  goal_msg.goal_x = this->get_parameter("goal_x").as_int();
  goal_msg.goal_y = this->get_parameter("goal_y").as_int();

  RCLCPP_INFO(this->get_logger(), "Sending goal: navigate to (%d, %d)",
              goal_msg.goal_x, goal_msg.goal_y);

  // -------------------------------------------------------------------------
  // Goal options with callbacks
  // -------------------------------------------------------------------------
  auto send_goal_options =
      rclcpp_action::Client<NavigateToGoal>::SendGoalOptions();

  send_goal_options.goal_response_callback =
                std::bind(&NavigateActionClient::goal_response_callback,
                this,
                std::placeholders::_1);

  send_goal_options.feedback_callback =
                std::bind(&NavigateActionClient::feedback_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2);

  send_goal_options.result_callback =
                std::bind(&NavigateActionClient::result_callback,
                this,
                std::placeholders::_1);


  // Send the goal
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

// =============================================================================
// cancel_goal (PROVIDED)
// =============================================================================

void NavigateActionClient::cancel_goal() {
  if (goal_handle_) {
    RCLCPP_INFO(this-> get_logger(), "Cancelling goal...");
    action_client_-> async_cancel_goal(goal_handle_);
  } else {
    RCLCPP_WARN(this-> get_logger(), "No active goal to cancel");
    rclcpp::shutdown();
  }
}

// =============================================================================
// goal_response_callback
// =============================================================================
  /**
   * @brief Callback function for goal
   * 
   * 1. Check if goal_handle is null (goal rejected)
   *    - If null: log error and call rclcpp::shutdown(), then return
   * 2. Store goal_handle in goal_handle_ member for cancellation support
   * 3. Log that goal was accepted
   * 
   * @param goal_handle 
   */
  void NavigateActionClient::goal_response_callback(
      const GoalHandleNavigate::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                  "Goal was rejected by action server");
      rclcpp::shutdown();
      return;
    }

  goal_handle_ = goal_handle;
  RCLCPP_INFO(this->get_logger(), "Goal accepted by action server");
  RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to cancel navigation");
}


// =============================================================================
// feedback_callback
// =============================================================================
  /**
   * @brief Callback to log feedback information.
   * 
   * Use RCLCPP_INFO to log the feedback fields:
   *   - current_x, current_y (position)
   *   - direction
   *   - elapsed_seconds
   */
  void NavigateActionClient::feedback_callback(
    GoalHandleNavigate::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const NavigateToGoal::Feedback> feedback) {
              RCLCPP_INFO(this->get_logger(),
              "Feedback - Position: (%d, %d), Direction: %d, Elapsed: %.2fs",
              feedback->current_x,
              feedback->current_y,
              static_cast<int>(feedback->direction),
              feedback->elapsed_seconds);
}

// =============================================================================
// result_callback
// =============================================================================
/**
 * @brief Callback to log results
 * 
 * 1. Clear goal handle.
 * 2. Log result code.
 * 3. Shutdown node.
 * 
 * @param result 
 */
void NavigateActionClient::result_callback(
    const GoalHandleNavigate::WrappedResult& result) {
  // Clear goal handle since navigation is complete
  goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "=== NAVIGATION SUCCEEDED ===");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "=== NAVIGATION ABORTED ===");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "=== NAVIGATION CANCELED ===");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "=== UNKNOWN RESULT ===");
      break;
  }

  RCLCPP_INFO(this->get_logger(), "Result:");
  RCLCPP_INFO(this->get_logger(), "  Success: %s",
              result.result->success ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Total steps: %d",
              result.result->total_steps);
  RCLCPP_INFO(this->get_logger(), "  Total time: %.2fs",
              result.result->total_time);
  RCLCPP_INFO(this->get_logger(), "  Message: %s",
              result.result->message.c_str());

  rclcpp::shutdown();
}

}  // namespace micromouse

// =============================================================================
// Main
// =============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<micromouse::NavigateActionClient>();

  // Send goal after node is initialized
  node->send_goal();

  // Spin to process callbacks
  rclcpp::spin(node);
}
