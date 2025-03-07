cmake_minimum_required(VERSION 3.8)
project(hesai_ros_driver)

# Set C++ 17 as default (if not specified).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find necessary dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Optional: if you have any headers in an include/ folder
include_directories(include)

# ----------------------------------------------------------------------------
# Build one executable from the node source file.
# The code includes main(), so no separate library or main.cpp is needed.
add_executable(${PROJECT_NAME}_node
  src/hesai_ros_driver_node.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ----------------------------------------------------------------------------
# Install the executable so ROS2 can find and run it via ros2 run
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
