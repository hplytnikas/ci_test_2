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

#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/box_array.hpp>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

namespace camera_detector {

using Pose = std::pair<float, float>;

struct Detection {
  cv::Rect box;
  float conf{};
  int classId{};
  float y_conf{};
  float b_conf{};
  float os_conf{};
  float ob_conf{};
};


class CameraDetector : public rclcpp::Node {
public:
  CameraDetector();

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // TF2 Listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                       // TF2 Buffer

  int buffer_size_; // Buffer size

  // int image_width_;         // Image Inference Width
  // int image_height_;        // Image Inference Height
  // int scaleFactor_;         // Downsampling factor
  // int brightness_;
  // float contrast_;

  // Functions
  void loadParameters();
  void subscribeTopics();
  void advertiseTopics();
  void DetectCones(const sensor_msgs::msg::Image::SharedPtr fw_image);
  void SetBoxesMessage(const std::vector<Detection> &detections, perception_msgs::msg::BoxArray &boxes);

  // Camera Status
  bool status_cone_array_;

  // Topic Names
  std::string topic_fw_cam_;

  std::string topic_runtime_;
  std::string topic_fw_bbox_;
  std::string topic_fw_overlay_;
  std::string topic_cone_array_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_fw_cam_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PipelineRuntime>::SharedPtr pub_runtime_;
  rclcpp::Publisher<perception_msgs::msg::BoxArray>::SharedPtr pub_fw_bbox_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_fw_overlay_;
  rclcpp::Publisher<perception_msgs::msg::BoundingBox>::SharedPtr pub_cone_array_;

  // Services for camera ptp
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ptp_client_;
};
} // namespace camera_detector
