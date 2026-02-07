// micromouse_node.hpp

#pragma once
/**
 * @file micromouse_node.hpp
 * @brief MicroMouse ROS2 Node Class Declaration
 *
 * This file declares the MicroMouseNode class which implements:
 * - Action server for navigation
 * - Service for robot status
 * - Publisher for robot position
 * 
 * Demonstrates ROS2 patterns:
 * - Declaring and retrieving parameters
 * - Creating action servers and service servers
 * - Publishing messages
 * 
 * Usage:
 *  ros2 run micromouse_cpp micromouse_node
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "micromouse_interfaces/action/navigate_to_goal.hpp"
#include "micromouse_interfaces/srv/get_robot_status.hpp"
#include "micromouse_cpp/maze_control_api.hpp"
#include "micromouse_interfaces/action/navigate_to_goal.hpp"


using NavigateToGoal     = micromouse_interfaces::action::NavigateToGoal;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToGoal>;
using GetRobotStatus     = micromouse_interfaces::srv::GetRobotStatus;


namespace micromouse {

    /**
     * @brief MicroMouse ROS2 Node
     *
     * Provides:
     * - Action server: /navigate_to_goal
     * - Service: /get_robot_status
     * - Publisher: /robot_position
     *
     * Can run in standalone mode (immediate navigation) or wait for action client.
     */
    class MicroMouseNode : public rclcpp::Node {
      public:
        MicroMouseNode() : Node("micromouse_node") {
            // =====================================================================
            // Declare and retrieve parameters
            // =====================================================================
            // Declare 5 parameters with default values:
            //   - "goal_x" (int, default: 7)
            //   - "goal_y" (int, default: 7)
            //   - "path_color" (string, default: "c")
            //   - "goal_color" (string, default: "g")
            //   - "standalone_mode" (bool, default: true)
            //
            // Then retrieve each parameter into the corresponding member variable.
            // For string parameters, get the first character: .as_string()[0]
            // =====================================================================
            this->declare_parameter<int>("goal_x", 7);
            this->declare_parameter<int>("goal_y", 7);
            this->declare_parameter<std::string>("path_color", "c");
            this->declare_parameter<std::string>("goal_color", "g");
            this->declare_parameter<bool>("standalone_mode", true);

            goal_x_ = this->get_parameter("goal_x").as_int();
            goal_y_ = this->get_parameter("goal_y").as_int();

            auto path_color_str = this->get_parameter("path_color").as_string();
            auto goal_color_str = this->get_parameter("goal_color").as_string();
            if (!path_color_str.empty()) {
            path_color_ = path_color_str[0];
            }
            if (!goal_color_str.empty()) {
            goal_color_ = goal_color_str[0];
            }

            standalone_mode_ = this->get_parameter("standalone_mode").as_bool();


            // =====================================================================
            // Create publisher for robot position
            // =====================================================================
            // Create a publisher for geometry_msgs::msg::Point on topic
            // "/robot_position" with a queue size of 10. Store in robot_position_pub_.
            // =====================================================================
            robot_position_pub_ =
            this->create_publisher<geometry_msgs::msg::Point>("/robot_position", 10);


            // =====================================================================
            // Create service server for robot status
            // =====================================================================
            // Create a service server for GetRobotStatus on "/get_robot_status".
            // Bind to get_status_callback method.
            // Store in status_srv_.
            // =====================================================================
            status_srv_ = this->create_service<GetRobotStatus>(
            "/get_robot_status",
            std::bind(&MicroMouseNode::get_status_callback, this,
                        std::placeholders::_1, std::placeholders::_2));


            // =====================================================================
            // Create action server for navigation
            // =====================================================================
            // Create an action server for NavigateToGoal on "/navigate_to_goal".
            // Bind three callbacks:
            //   - handle_goal (goal request callback)
            //   - handle_cancel (cancel request callback)
            //   - handle_accepted (goal accepted callback)
            // Store in action_server_.
            // =====================================================================
            action_server_ = rclcpp_action::create_server<NavigateToGoal>(
                this,
                "/navigate_to_goal",
                std::bind(&MicroMouseNode::handle_goal, 
                        this,
                        std::placeholders::_1, 
                        std::placeholders::_2
                        ),
                std::bind(&MicroMouseNode::handle_cancel, 
                        this,
                        std::placeholders::_1
                        ),
                std::bind(&MicroMouseNode::handle_accepted, 
                        this,
                        std::placeholders::_1)
            );

            log("Action server ready: /navigate_to_goal");
        }

    /**
     * @brief Run navigation immediately (standalone MMS mode)
     */
    void run_navigation();

    /**
     * @brief Check if running in standalone mode
     * @return true if standalone mode (immediate navigation), false if waiting
     * for action
     */
    bool is_standalone_mode() const { return standalone_mode_;}

    private:
        // Maze dimensions
        int W_{16};
        int H_{16};
        int goal_x_{7};
        int goal_y_{7};

        // Visualization colors
        char path_color_{'c'};
        char goal_color_{'g'};

        // Mode
        bool standalone_mode_{true};

        // Internal wall representation: walls_[x][y][dir] = true if wall exists
        std::vector<std::vector<std::array<bool, 4>>> walls_;

        // Track explored cells
        std::set<Cell> explored_cells_;

        // Robot state
        Cell robot_{0, 0};
        Dir facing_{Dir::North};
        int steps_{0};

        // Timing
        std::chrono::steady_clock::time_point nav_start_time_;
        bool is_running_{false};
        std::mutex state_mutex_;

        // ROS2 interfaces
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_position_pub_;
        rclcpp::Service<GetRobotStatus>::SharedPtr status_srv_;
        rclcpp_action::Server<NavigateToGoal>::SharedPtr action_server_;

        // =========================================================================
        // Utility Methods (PROVIDED)
        // =========================================================================

        void log(const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
            std::cerr << "[LOG] " << msg << std::endl;
        }

        bool in_bounds(const Cell& c) const {
            return c.x >= 0 && c.y >= 0 && c.x < W_ && c.y < H_;
        }

        // =========================================================================
        // Wall Management (PROVIDED)
        // =========================================================================

        /**
         * @brief Initialize internal wall storage and set perimeter walls
         */
        void init_walls();

        /**
         * @brief Mark wall on both sides (current cell and neighbor) 
         */
        void mark_wall(const Cell& c, Dir d);

        /**
         * @brief Sense walls and update internal map
         */
        void sense_and_update(const Cell& pos, Dir facing);

        /**
         * @brief Check if edge between cell and direction is free (no wall)
         */
        bool edge_free(const Cell& c, Dir d) const;

        // =========================================================================
        // Movement (PROVIDED)
        // =========================================================================

        /**
         * @brief Turn robot to face desired direction
         */
        void face_direction(Dir want);


        /**
         * @brief Compute path using Depth-First Search
         *
         * Uses iterative DFS with a stack. Explores neighbors in order:
         * North, East, South, West. Does NOT guarantee shortest path.
         *
         * @param start Starting cell
         * @param goal Target cell
         * @return Path from start to goal, or nullopt if no path exists
         */
        std::optional<std::vector<Cell>> dfs_plan(const Cell& start, const Cell& goal);


        /**
         * @brief Visualize path in maze
         * 
         * @param path Vector of Cells representing the path
         * @param goal Goal Cell to color differently
         */
        void color_path(const std::vector<Cell>& path, const Cell& goal);

        /**
         * @brief Execute navigation with dynamic replanning
         *
         * Algorithm:
         * 1. Sense walls at current position
         * 2. Plan path using DFS
         * 3. Follow path, sensing before each move
         * 4. If wall blocks path, replan from current position
         * 5. Repeat until goal reached or no path exists
         *
         * @param start Starting cell
         * @param goal Target cell
         * @param goal_handle Optional goal handle for action server (nullptr default)
         * @return true if goal reached, false if no path exists or cancelled
         */
        bool execute_with_replanning(
            const Cell& start, 
            const Cell& goal,
            std::shared_ptr<GoalHandleNavigate> goal_handle = nullptr); 
        
        /**
         * @brief Function to publish position
         * 
         */
        void publish_position();


        /**
         * @brief Get the elapsed seconds object
         * 
         * @return double 
         */
        double get_elapsed_seconds() const;
        

        /**
         * @brief Callback function to get status
         * 
         */
        void get_status_callback(
            const std::shared_ptr<GetRobotStatus::Request> /*request*/,
            std::shared_ptr<GetRobotStatus::Response> response
            ); 
        

        /**
         * @brief Callback function for goal handling (action server)
         * 
         */
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& /*uuid*/,
            std::shared_ptr<const NavigateToGoal::Goal> goal);


        /**
         * @brief Callback function for goal canceling (action server)
         * 
         */    
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleNavigate> /*goal_handle*/) ;
        

        /**
         * @brief Callback function for accepted goals (action server)
         * 
         * @param goal_handle 
         */
        void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

        
        /**
         * @brief Execute navigation action
         * 
         * @param goal_handle 
         */
        void execute_action(const std::shared_ptr<GoalHandleNavigate> goal_handle);

     };
};