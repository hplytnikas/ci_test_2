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
#ifndef LIDAR_CONE_DETECTOR_NODE_HPP_
#define LIDAR_CONE_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autonomous_msgs/msg/cone_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lidar_cone_detector {

class LidarConeDetectorNode : public rclcpp::Node
{
public:
  explicit LidarConeDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Subscriber for raw point cloud data
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;

  // Publishers for the different topics
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_compensated_publisher_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr cone_array_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_point_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_publisher_;
};

}  // namespace lidar_cone_detector

#endif  // LIDAR_CONE_DETECTOR_NODE_HPP_
