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
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "camera_detector.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_detector::CameraDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
