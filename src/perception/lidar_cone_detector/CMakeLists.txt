cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find necessary dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include headers from the include/ folder.
include_directories(include)

# Create the executable target from the source files.
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/lidar_cone_detector_node.cpp
)

# Link the necessary dependencies to the target.
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  autonomous_msgs
  visualization_msgs
)

# Install the executable so ROS2 can run it via ros2 run.
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally install the launch directory.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
