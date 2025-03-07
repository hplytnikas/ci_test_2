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
#include <functional>

namespace lidar_cone_detector {

LidarConeDetectorNode::LidarConeDetectorNode(const rclcpp::NodeOptions & options)
: Node("lidar_cone_detector_node", options)
{
  // 1) Subscription to raw LiDAR point cloud
  point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "lidar_points",  // Use the same topic name as last year's node
    1,
    std::bind(&LidarConeDetectorNode::pointCloudCallback, this, std::placeholders::_1)
  );

  // 2) Publishers

  // Compensated point cloud publisher
  point_cloud_compensated_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidar_points_compensated",  
    1
  );

  // Cone array publisher
  cone_array_publisher_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(
    "lidar_cone_detector/cone_array",  
    1
  );


  // Filtered point cloud publisher
  filtered_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidar_cone_detector/filtered_point_cloud",  
    1
  );

  // Centroid marker publisher
  centroid_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lidar_cone_detector/centroid_marker_array",
    1
  );

  RCLCPP_INFO(this->get_logger(), "Dummy LidarConeDetectorNode for 2025 is initialized.");
}

void LidarConeDetectorNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Log the incoming point cloud dimensions
  RCLCPP_INFO(this->get_logger(), "Received point cloud: width=%u, height=%u", msg->width, msg->height);

  // -------------------------------------------------------
  // 1) Publish a dummy "compensated" point cloud (copy header from input)
  sensor_msgs::msg::PointCloud2 compensated_cloud;
  compensated_cloud.header = msg->header;
  point_cloud_compensated_publisher_->publish(compensated_cloud);

  // -------------------------------------------------------
  // 2) Publish a dummy ConeArray
  autonomous_msgs::msg::ConeArray cone_array_msg;
  cone_array_msg.header = msg->header;
  cone_array_publisher_->publish(cone_array_msg);

  // -------------------------------------------------------
  // 3) Publish a dummy "filtered" point cloud
  sensor_msgs::msg::PointCloud2 filtered_cloud;
  filtered_cloud.header = msg->header;
  filtered_point_cloud_publisher_->publish(filtered_cloud);

  // -------------------------------------------------------
  // 4) Publish dummy centroid markers
  visualization_msgs::msg::MarkerArray centroid_markers;
  centroid_publisher_->publish(centroid_markers);
}

}  // namespace lidar_cone_detector
