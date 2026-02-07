#pragma once
/**
 * @file get_status_client.hpp
 * @brief Service Client for querying MicroMouse Robot Status
 *
 * This node demonstrates ROS2 service client patterns:
 * - Creating a service client
 * - Waiting for service availability
 * - Sending asynchronous requests
 * - Handling responses via callbacks
 *
 * Usage:
 *   ros2 run micromouse_cpp get_status_client
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "micromouse_interfaces/srv/get_robot_status.hpp"

namespace micromouse {

using GetRobotStatus = micromouse_interfaces::srv::GetRobotStatus;

/**
 * @brief Service Client Node for Robot Status
 *
 * Queries the /get_robot_status service to retrieve current robot state
 * including position, direction, steps taken, and navigation status.
 */
class GetStatusClient : public rclcpp::Node {
public:
    /**
     * @brief Constructor - creates service client
     */
    GetStatusClient();

    /**
     * @brief Send status request to the service server
     *
     * Waits for service availability, then sends an asynchronous request.
     * The response is handled in response_callback().
     */
    void send_request();

private:
    /// Service client for /get_robot_status
    rclcpp::Client<GetRobotStatus>::SharedPtr client_;

    /**
     * @brief Callback when service response is received
     * @param future Future containing the service response
     *
     * Logs all response fields and shuts down the node.
     */
    void response_callback(rclcpp::Client<GetRobotStatus>::SharedFuture future);
};

}  // namespace micromouse