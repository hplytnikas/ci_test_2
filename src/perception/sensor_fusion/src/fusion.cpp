/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Tristan Gabl <trgabl@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "fusion.hpp"

#include <iostream>

namespace sensor_fusion_baseline {
SensorFusion::SensorFusion() : Node("sensor_fusion") {
  LoadParameters();
  LoadCalibrationParameters();
  AdvertiseTopics();
  SubscribeTopics();
}


/**
 * Callback function of all the topics. The message_filters package calls this
 * function when all the messages arrive at (approximately) the same time.
 *
 * @param pc_msg    # Point cloud message
 * @param bbox_msg  # Bounding box message
 * @param img_msg   # Image message
 */
void SensorFusion::DetectCones(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg,
                               const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                               const sensor_msgs::msg::Image::SharedPtr img_msg) {
  
    autonomous_msgs::msg::ConeArray cone_array_msg_dummy;
    cone_array_msg_dummy.header.stamp = this->now();
    cone_array_msg_dummy.header.frame_id = "map";

    // Create dummy cones
    for (int i = 0; i < 3; ++i) {
      autonomous_msgs::msg::Cone cone;
      cone.position.x = static_cast<float>(i);
      cone.position.y = static_cast<float>(i * 2);
      cone.position.z = 0.0;

      cone_array_msg_dummy.cones.push_back(cone);
    }

    pub_cone_array_->publish(cone_array_msg_dummy);
    SensorFusion::PublishConeMarkers(cone_array_msg_dummy);
    
    // Create a dummy image with the same size as the input image
    cv::Mat image_mat_dummy = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
    cv::randn(image_mat_dummy, cv::Scalar::all(128), cv::Scalar::all(20));

    auto img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_mat_dummy).toImageMsg();

    pub_img_overlay_->publish(*img);

    perception_msgs::msg::BoxArrayDebug sf_boxes_dummy;
    for (int i = 0; i < 3; ++i) {
      perception_msgs::msg::BoundingBoxDebug box;
      box.x = static_cast<float>(i);
      box.y = static_cast<float>(i * 2);
      box.box_x_min = 0;
      box.box_y_min = 0;
      box.box_x_max = 1;
      box.box_y_max = 1;
      box.label = 0; // Assign an integer value to label
      sf_boxes_dummy.boxes.push_back(box);
    }

    pub_bbox_fusion_->publish(sf_boxes_dummy);

    perception_msgs::msg::PipelineRuntime runtime_msg_dummy;
    runtime_msg_dummy.runtime = 0.0;
    pub_runtime_->publish(runtime_msg_dummy);
}
} // namespace sensor_fusion_baseline
