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

#include "camera_detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace camera_detector {

void CameraDetector::subscribeTopics() {
  sub_fw_cam_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_fw_cam_, buffer_size_, std::bind(&CameraDetector::DetectCones, this, std::placeholders::_1));
}

// Advertise yolo predictions - debugging purposes
void CameraDetector::advertiseTopics() {
  pub_runtime_ = this->create_publisher<perception_msgs::msg::PipelineRuntime>(topic_runtime_, 1);
  pub_fw_bbox_ = this->create_publisher<perception_msgs::msg::BoxArray>(topic_fw_bbox_, 1);
  pub_fw_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(topic_fw_overlay_, 1);
}

void CameraDetector::SetBoxesMessage(const std::vector<Detection> &detections, perception_msgs::msg::BoxArray &boxes) {
  for (int i = 0; i < detections.size(); i++) {
    auto box = detections[i].box;

    perception_msgs::msg::BoundingBox output_box;
    output_box.box_x_min = box.x;
    output_box.box_x_max = box.x + box.width;
    output_box.box_y_min = box.y;
    output_box.box_y_max = box.y + box.height;
    output_box.prob_cone = detections[i].conf;
    output_box.label = detections[i].classId;
    output_box.prob_type.blue = detections[i].b_conf;
    output_box.prob_type.yellow = detections[i].y_conf;
    output_box.prob_type.orange = detections[i].os_conf;
    output_box.prob_type.orange_big = detections[i].ob_conf;
    output_box.depth = 0.0; // Depth will be assgined downstream in the depth_estimation node.
    boxes.boxes.push_back(output_box);
  }
}

void CameraDetector::loadParameters() {
  // Subscriber Topic Names
  this->declare_parameter<std::string>("topics.sub.fw_cam", "/sensors/forward_camera/image_color");

  // Publisher Topic Names
  this->declare_parameter<std::string>("topics.pub.runtime", "/perception/runtimes");
  this->declare_parameter<std::string>("topics.pub.fw_bbox", "/perception/yolo_camera_detector/forward_bbox");
  this->declare_parameter<std::string>("topics.pub.debug.fw_overlay",
                                       "/perception/yolo_camera_detector/debug/forward_overlay");
  this->get_parameter("topics.sub.fw_cam", topic_fw_cam_);
  this->get_parameter("topics.pub.runtime", topic_runtime_);
  this->get_parameter("topics.pub.fw_bbox", topic_fw_bbox_);
  this->get_parameter("topics.pub.debug.fw_overlay", topic_fw_overlay_);
}

CameraDetector::CameraDetector() : Node("yolo_camera_detector") {
  loadParameters();
  subscribeTopics();
  advertiseTopics();
}

void CameraDetector::DetectCones(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  // rclcpp::Time img_msg_time = img_msg->header.stamp;
  // rclcpp::Time end_time = this->get_clock()->now();

  // Creates a copy of img_msg (is a ROS type image - sensors::msgs::Image) and
  // converts it to img_mat (is an OpenCV type image - cv::Mat). We do the
  // conversion to feed it to the detector.
  cv::Mat img_mat = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  // FYI - image tensor format [B, C, H , W]
  // To test different input shape -> Export yolov5s_amz.pt again using
  // ultralitics script and
  // --img-size x y --dynamic --include onnx

  const int DUMMY_DETECTIONS = 5;
  std::vector<Detection> detections;
  for (int i = 0; i < DUMMY_DETECTIONS; i++) {
    Detection det;
    detections.push_back(det);
  } 
  perception_msgs::msg::BoxArray boxes;
  boxes.header = img_msg->header;
  SetBoxesMessage(detections, boxes);
  pub_fw_bbox_->publish(boxes);
}
}
