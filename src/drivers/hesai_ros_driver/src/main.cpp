/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2025 Author:
 *   - Fred Defokue   <fdefokue@ethz.ch>
 *   
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include <rclcpp/rclcpp.hpp>
#include "hesai_ros_driver/hesai_ros_driver_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyHesaiDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
