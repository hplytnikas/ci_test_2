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
#include "depth_estimation.hpp"
#include <opencv2/opencv.hpp>

namespace depth_estimation {
void DepthEstimation::loadParameters() {
  this->declare_parameter<std::string>("topics.pub.bbox_depth", "/perception/depth_estimation/bbox_depth");
  this->declare_parameter<std::string>("topics.pub.cone_array", "/perception/camera/cone_array");

  this->declare_parameter<std::string>("topics.sub.fw_bbox", "/perception/yolo_camera_detector/forward_bbox");
  this->declare_parameter<std::string>("topics.pub.debug.img_overlay",
                                       "/perception/depth_estimation/debug/img_overlay");
  this->declare_parameter<std::string>("topics.sub.fw_cam", "/sensors/forward_camera/image_color");

  // Frame IDs parameters
  this->declare_parameter<std::string>("frame_ids.camera", "pylon_camera");
  this->declare_parameter<std::string>("frame_ids.base_link", "base_link");

  this->get_parameter("topics.pub.bbox_depth", topic_bbox_depth_);
  this->get_parameter("topics.pub.cone_array", topic_cone_array_);
  this->get_parameter("topics.sub.fw_bbox", topic_fw_bbox_);
  this->get_parameter("topics.pub.debug.img_overlay", topic_img_overlay_);
  this->get_parameter("topics.sub.fw_cam", topic_fw_cam_);

  this->get_parameter("frame_ids.camera", camera_frame_id_);
  this->get_parameter("frame_ids.base_link", base_link_frame_id_);
}

void DepthEstimation::advertiseTopics() {
  pub_bbox_depth_ = this->create_publisher<perception_msgs::msg::BoxArray>(topic_bbox_depth_, 1);
  pub_cone_array_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_cone_array_, 1);
  pub_img_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(topic_img_overlay_, 10);
  pub_cone_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/camera/cone_markers", 1);
}

void DepthEstimation::subscribeTopics() {
  sub_bbox_.subscribe(this, topic_fw_bbox_);
  sub_image_.subscribe(this, topic_fw_cam_);
  sync_.reset(new Sync(SyncPolicy(20), sub_bbox_, sub_image_));
  sync_->registerCallback(&depth_estimation::DepthEstimation::BboxDepth, this);
}

DepthEstimation::DepthEstimation() : Node("depth_estimation") {
  loadParameters();
  subscribeTopics();
  RCLCPP_INFO(this->get_logger(), "Depth Estimation has initialized.");
}

void DepthEstimation::BboxDepth(const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                                const sensor_msgs::msg::Image::SharedPtr img_msg) {
// compute & publish depth information here
}
}
