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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autonomous_msgs/msg/cone_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/box_array.hpp>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <utility>

namespace depth_estimation {

using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<perception_msgs::msg::BoxArray, sensor_msgs::msg::Image>;
using Sync = message_filters::Synchronizer<SyncPolicy>;

using Pose = std::pair<double, double>;

class DepthEstimation : public rclcpp::Node {
public:
  DepthEstimation();

private:
  std::shared_ptr<Sync> sync_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // TF2 Listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                       // TF2 Buffer

  // Topics
  std::string topic_bbox_depth_;
  std::string topic_cone_array_;
  std::string intrinsics_server_name_;
  std::string topic_fw_cam_;
  std::string topic_fw_bbox_;
  std::string topic_img_overlay_;

  // Frame IDs
  std::string camera_frame_id_;
  std::string base_link_frame_id_;

  // Subscriber
  message_filters::Subscriber<perception_msgs::msg::BoxArray> sub_bbox_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::BoxArray>::SharedPtr pub_bbox_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_overlay_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr pub_cone_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cone_markers_;

  // Functions
  void loadParameters();
  void advertiseTopics();
  void subscribeTopics();
  void loadCalibrationParameters();
  void BboxDepth(const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                 const sensor_msgs::msg::Image::SharedPtr img_msg);
};

} // namespace depth_estimation
