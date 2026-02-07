/**
 * @file get_status_client.cpp
 * @brief Implementation of Service Client for Robot Status
 *
 * @details
 * This file demonstrates how to:
 * - Create a ROS2 service client
 * - Wait for a service to become available
 * - Send asynchronous service requests
 * - Handle service responses with callbacks
 *
 * @authors
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#include "micromouse_cpp/get_status_client.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace micromouse {

// =============================================================================
// Constructor
// =============================================================================

GetStatusClient::GetStatusClient() : Node("get_status_client") {
  // =========================================================================
  // Create service client
  // =========================================================================
  // Create a service client for GetRobotStatus on "/get_robot_status"
  // Store in client_ member variable.
  // =========================================================================
  
  client_ = this->create_client<GetRobotStatus>("/get_robot_status");

  RCLCPP_INFO(this->get_logger(),
              "Service client created for /get_robot_status");
}


// =============================================================================
// send_request
// =============================================================================
  /**
   * @brief Create and send service request
   * 
   * 1. Create an empty request: std::make_shared<GetRobotStatus::Request>()
   * 2. Send the request asynchronously with callback:
   *     client_->async_send_request(request, callback)
   *     where callback is bound to response_callback method
   */
  void GetStatusClient::send_request() {
    // Wait for service to be available
    RCLCPP_INFO(this->get_logger(), "Waiting for service...");

    if (!client_->wait_for_service(5s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Service /get_robot_status not available after 5 seconds");
      RCLCPP_WARN(this->get_logger(),
                  "Make sure micromouse_node is running in MMS simulator");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Service available, sending request...");

    auto request = std::make_shared<GetRobotStatus::Request>();

    auto callback =
        std::bind(&GetStatusClient::response_callback, this, _1);

    client_->async_send_request(request, callback);
  }

  // =========================================================================
  // Handle service response
  // =========================================================================
  /**
   * @brief Handle service response
   * 
   * 1. Get the response from the future: future.get()
   * 2. Log all 9 response fields using RCLCPP_INFO:
   *     - position_x, position_y
   *     - direction (string)
   *     - steps_taken
   *     - steps_to_goal_estimate
   *     - elapsed_seconds
   *     - is_running (bool -> "true"/"false")
   *     - success (bool -> "true"/"false")
   *     - message (string)
   * 3. Call rclcpp::shutdown() to exit
   * 
   * @param future 
   */
  void GetStatusClient::response_callback(
    rclcpp::Client<GetRobotStatus>::SharedFuture future) {
    auto response = future.get();

    RCLCPP_INFO(this->get_logger(), "Robot Status:");
    RCLCPP_INFO(this->get_logger(), "  Position: (%d, %d)",
                response->position_x, response->position_y);
    RCLCPP_INFO(this->get_logger(), "  Direction: %s",
                response->direction.c_str());
    RCLCPP_INFO(this->get_logger(), "  Steps taken: %d",
                response->steps_taken);
    RCLCPP_INFO(this->get_logger(), "  Steps-to-goal estimate: %d",
                response->steps_to_goal_estimate);
    RCLCPP_INFO(this->get_logger(), "  Elapsed time: %.2fs",
                response->elapsed_seconds);
    RCLCPP_INFO(this->get_logger(), "  Is running: %s",
                response->is_running ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Success: %s",
                response->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Message: %s",
                response->message.c_str());

    rclcpp::shutdown();
  }

// }

}  // namespace micromouse

// =============================================================================
// Main
// =============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<micromouse::GetStatusClient>();

  // Send request after node is initialized
  node->send_request();

  // Spin to process callback
  rclcpp::spin(node);
}
