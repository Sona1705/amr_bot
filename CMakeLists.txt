cmake_minimum_required(VERSION 3.5)
project(amr_bot)

# Default to C++14 if no C++ standard is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Declare your executable here
add_executable(amr_bot_node src/amr_bot_node.cpp)  # Make sure this path is correct

# Link against ROS 2 libraries
ament_target_dependencies(amr_bot_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  controller_manager
  gazebo_ros
  robot_state_publisher
)

install(TARGETS
  amr_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
