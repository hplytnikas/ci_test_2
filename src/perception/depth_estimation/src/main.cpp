/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - František Kmječ     <frantisek.kmjec@gmail.com>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include <rclcpp/rclcpp.hpp>
#include "depth_estimation.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<depth_estimation::DepthEstimation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
