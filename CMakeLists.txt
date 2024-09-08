################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_multi_robot)

################################################################################
# Default to C++14
################################################################################
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

################################################################################
# Include directories
################################################################################
include_directories(
  include
)

################################################################################
# Add executable
################################################################################
add_executable(behavior_tree_node src/behavior_trees/tree.cpp)

################################################################################
# Link libraries
################################################################################
ament_target_dependencies(behavior_tree_node
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

################################################################################
# Install
################################################################################
install(TARGETS behavior_tree_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models rviz urdf worlds params
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

################################################################################
# Ament package
################################################################################
ament_package()

