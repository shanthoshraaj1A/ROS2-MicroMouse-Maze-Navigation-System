cmake_minimum_required(VERSION 3.22)
project(micromouse_cpp)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(micromouse_interfaces REQUIRED)

# Include directories
include_directories(include)

# =============================================================================
# LIBRARIES
# =============================================================================

# MazeControlAPI library (MMS communication)
add_library(maze_control_api_lib
  src/maze_control_api.cpp
)
ament_target_dependencies(maze_control_api_lib rclcpp)

# =============================================================================
# SINGLE EXECUTABLE - MicroMouse Node (for MMS simulator)
# =============================================================================
# This executable communicates with MMS via stdin/stdout.

add_executable(micromouse_node
  src/micromouse_node.cpp
)
target_link_libraries(micromouse_node maze_control_api_lib)
ament_target_dependencies(micromouse_node
  rclcpp
  rclcpp_action
  geometry_msgs
  micromouse_interfaces
)

# =============================================================================
# ACTION CLIENT - For external goal sending and monitoring
# =============================================================================
# Use this to send goals and receive feedback from micromouse_node

add_executable(navigate_action_client
  src/navigate_action_client.cpp
)
ament_target_dependencies(navigate_action_client
  rclcpp
  rclcpp_action
  micromouse_interfaces
)

# Service Client - queries robot status
add_executable(get_status_client
  src/get_status_client.cpp
)
ament_target_dependencies(get_status_client
  rclcpp
  micromouse_interfaces
)

# =============================================================================
# INSTALL
# =============================================================================

install(TARGETS
  micromouse_node
  navigate_action_client
  get_status_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# =============================================================================
# TESTING
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
