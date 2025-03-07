cmake_minimum_required(VERSION 3.8)
project(vcu_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarCommand.msg"
  "msg/EbsPressures.msg"
  "msg/FourWheelCarCommand.msg"
  "msg/LlcLogging.msg"
  "msg/LlcParameters.msg"
  "msg/LvBattery.msg"
  "msg/MissionSelect.msg"
  "msg/ResState.msg"
  "msg/StateMachine.msg"
  "msg/TorqueData.msg"
  "msg/VelocityEstimation.msg"
  DEPENDENCIES std_msgs geometry_msgs autonomous_msgs
)

ament_package()
