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

#include "cone_fusion.hpp"

namespace cone_fusion {
    
ConeFusion::ConeFusion() : Node("cone_fusion") {
    LoadParameters();
    SubscribeTopics();
    AdvertiseTopics(); 
}

void ConeFusion::LoadParameters() {

    // Topic Names
    this->declare_parameter<std::string>("topics.sub.camera_cone", "/perception/camera/cone_array");
    this->declare_parameter<std::string>("topics.sub.lidar_cone", "/perception/lidar/cone_array");
    this->declare_parameter<std::string>("topics.sub.sf_cone", "/perception/fusion/cone_array");
    this->declare_parameter<std::string>("topics.pub.output_cone", "/perception/cone_array");
    this->declare_parameter<std::string>("topics.pub.output_cone_markers", "/perception/cone_markers");
    this->declare_parameter<std::string>("topics.pub.runtime", "/perception/runtimes");


    this->declare_parameter<std::string>("perception_mode", "sensor_fusion");

    // Topic Names
    this->get_parameter("topics.sub.camera_cone", topic_camera_cone_array_);
    this->get_parameter("topics.sub.lidar_cone", topic_lidar_cone_array_);
    this->get_parameter("topics.sub.sf_cone", topic_sf_cone_array_);
    this->get_parameter("topics.pub.output_cone", topic_output_cone_array_);
    this->get_parameter("topics.pub.output_cone_markers", topic_output_cone_markers_);
    this->get_parameter("topics.pub.runtime", topic_runtime_);

    std::string perception_mode = this->get_parameter("perception_mode").as_string();

    if (perception_mode == "camera_only")
        active_pipeline_ = camera_only;
    else if (perception_mode == "lidar_only")
        active_pipeline_ = lidar_only;
    else if (perception_mode == "sensor_fusion")
        active_pipeline_ = sensor_fusion;
}

void ConeFusion::SubscribeTopics() {
    sub_camera_cone_array_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
        topic_camera_cone_array_, 10, std::bind(&ConeFusion::PassCameraCones, this, std::placeholders::_1));
    sub_lidar_cone_array_ = this->create_subscription<autonomous_msgs::msg::ConeArray>(
        topic_lidar_cone_array_, 10, std::bind(&ConeFusion::PassLiDARCones, this, std::placeholders::_1));

    sub_lidar_cone_array2_.subscribe(this, topic_lidar_cone_array_);
    sub_sf_cone_array_.subscribe(this, topic_sf_cone_array_);
    sync_.reset(new Sync(SyncPolicy(20), sub_sf_cone_array_, sub_lidar_cone_array2_));
    sync_->registerCallback(&ConeFusion::ConcatenateLiDARtoFusion, this);
}


void ConeFusion::AdvertiseTopics() {
    pub_runtime_ = this->create_publisher<perception_msgs::msg::PipelineRuntime>(topic_runtime_, 1);
    pub_output_cone_array_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_output_cone_array_, 1);
    pub_cone_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_output_cone_markers_, 1);
}


void ConeFusion::PassCameraCones(const autonomous_msgs::msg::ConeArray::SharedPtr camera_cone_msg) {
    CommonPostProcessor(*camera_cone_msg); // dummy
}


void ConeFusion::PassLiDARCones(const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cones) {
    CommonPostProcessor(*lidar_cones); // dummy
}


void ConeFusion::ConcatenateLiDARtoFusion(const autonomous_msgs::msg::ConeArray::SharedPtr sf_cone_msg,
                                          const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cone_msg) {
    
    autonomous_msgs::msg::ConeArray output_cones = dummy_cones();

    pub_output_cone_array_->publish(output_cones);

    PublishConeMarkers(output_cones);
    PublishRuntime(sf_cone_msg->header.stamp, this->get_clock()->now());
}


void ConeFusion::CommonPostProcessor(const autonomous_msgs::msg::ConeArray &all_cones) {
    autonomous_msgs::msg::ConeArray output = dummy_cones();    

    pub_output_cone_array_->publish(output);

    PublishConeMarkers(output);
    PublishRuntime(output.header.stamp, this->get_clock()->now());
} 


void ConeFusion::PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array) {
    visualization_msgs::msg::MarkerArray markerArray;
    int markerId = 0;
    int max_markers = 50;

    // Remove markers from previous callback
    for (int i = 0; i < max_markers; ++i) { // assuming you won't have more than 50 markers
        visualization_msgs::msg::Marker deleteMarker;
        deleteMarker.header.frame_id = cone_array.header.frame_id;
        deleteMarker.header.stamp = cone_array.header.stamp;
        deleteMarker.ns = "cones";
        deleteMarker.id = i;
        deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
        markerArray.markers.push_back(deleteMarker);
    }

    for (const auto &cone : cone_array.cones) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = cone_array.header.frame_id;
        marker.header.stamp = cone_array.header.stamp;
        marker.ns = "cones";
        marker.id = markerId++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = cone.position.x;
        marker.pose.position.y = cone.position.y;
        marker.pose.position.z = 0.01; // Slightly above ground for visibility
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4; // Specify the size of the marker
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        if (cone.pipeline == 2) {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        } else if (cone.prob_type.blue > cone.prob_type.yellow && cone.prob_type.blue > cone.prob_type.orange &&
                cone.prob_type.blue > cone.prob_type.orange_big) {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        } else if (cone.prob_type.yellow > cone.prob_type.blue && cone.prob_type.yellow > cone.prob_type.orange &&
                cone.prob_type.yellow > cone.prob_type.orange_big) {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        } else if (cone.prob_type.orange > cone.prob_type.blue && cone.prob_type.orange > cone.prob_type.yellow &&
                cone.prob_type.orange > cone.prob_type.orange_big) {
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        } else if (cone.prob_type.orange_big > cone.prob_type.blue && cone.prob_type.orange_big > cone.prob_type.yellow &&
                cone.prob_type.orange_big > cone.prob_type.orange) {
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        } else {
        marker.color.r = 1.0;
        marker.color.g = 0.3;
        marker.color.b = 0.0;
        }

        marker.color.a = 1.0; // Make the marker opaque
        markerArray.markers.push_back(marker);
    }

    pub_cone_markers_->publish(markerArray);
}


void ConeFusion::PublishRuntime(const rclcpp::Time &start_time, const rclcpp::Time &end_time) {
  perception_msgs::msg::PipelineRuntime runtime_msg_dummy;
  switch (active_pipeline_) {
    case lidar_only:
        runtime_msg_dummy.node = "perception_pipeline/lidar_only";
        break;
    case camera_only:
        runtime_msg_dummy.node = "perception_pipeline/camera_only";
        break;
    case sensor_fusion:
        runtime_msg_dummy.node = "perception_pipeline/sensor_fusion";
        break;

    default:
        break;
  }

  runtime_msg_dummy.runtime = 0.0;
  pub_runtime_->publish(runtime_msg_dummy);
}

}