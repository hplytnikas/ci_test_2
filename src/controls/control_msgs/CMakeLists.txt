set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.5)
project(control_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Define message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerRef.msg"
  "msg/ReferenceState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
