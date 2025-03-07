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
#include "lidar_cone_detector/lidar_cone_detector_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_cone_detector::LidarConeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
