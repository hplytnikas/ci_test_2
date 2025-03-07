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

#pragma once
#define PCL_NO_RECOMPILE

#include "visualization_msgs/msg/marker_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autonomous_msgs/msg/cone_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_msgs/msg/bounding_box.hpp>
#include <perception_msgs/msg/bounding_box_debug.hpp>
#include <perception_msgs/msg/box_array.hpp>
#include <perception_msgs/msg/box_array_debug.hpp>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/types.hpp>
#include <string>
#include <vector>

namespace sensor_fusion_baseline {

// shorthands for message_filters synchronization
// policy (we use approximate time policy) types
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, perception_msgs::msg::BoxArray,
                                                    sensor_msgs::msg::Image>;
using Sync = message_filters::Synchronizer<SyncPolicy>;

class SensorFusion : public rclcpp::Node {
public:
  SensorFusion();

private:
  std::shared_ptr<Sync> sync_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF2 Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              // TF2 Buffer

  // Frame IDs
  std::string camera_frame_id_;
  std::string lidar_frame_id_;
  std::string base_link_frame_id_;
  std::string reference_frame_id_;

  // Image resolution
  int image_width_;
  int image_height_;

  // Subscribed Topics
  std::string topic_points_compensated_;
  std::string topic_fw_bbox_;
  std::string topic_fw_cam_;
  
  // Publisher Topic Names
  std::string topic_runtime_;
  std::string topic_img_overlay_;
  std::string topic_cone_array_;
  std::string topic_bbox_fusion_;
  std::string topic_cone_markers_;

  // Intrinsics
  std::string intrinsics_file_;

  // Subscribers
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud_;
  message_filters::Subscriber<perception_msgs::msg::BoxArray> sub_bbox_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PipelineRuntime>::SharedPtr pub_runtime_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_overlay_;
  rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr pub_cone_array_;
  rclcpp::Publisher<perception_msgs::msg::BoxArrayDebug>::SharedPtr pub_bbox_fusion_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cone_markers_;

  // Intrinsics & Extrinscs (MRH2Cam) for the active camera
  cv::Mat K_mat_ = cv::Mat(3, 3, CV_32F);
  cv::Mat D_mat_ = cv::Mat(1, 14, CV_32F);
  cv::Mat t_mat_ = cv::Mat(1, 3, CV_32F);
  cv::Mat rvecR_ = cv::Mat(3, 1, CV_32F);

  void LoadParameters();
  void LoadCalibrationParameters();
  void SubscribeTopics();
  void AdvertiseTopics();
  void LoadIntrinsics();

  void TryLoadStaticTransforms();

  /**
   * Callback function for the synchronized messages
   * @param pc_msg Point cloud message
   * @param bbox_msg Bounding box message
   * @param img_msg Image message
   */
  void DetectCones(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg,
                   const perception_msgs::msg::BoxArray::SharedPtr bbox_msg,
                   const sensor_msgs::msg::Image::SharedPtr img_msg);

  /**
   * Publishes the cone markers for better visualization in rviz2
   * @param cone_array The array of cones to be visualized
   */
  void PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array);

  /**
   * Resolves the filename from a 'package://' format to the actual file path in the package directory
   * Inspired by "CameraInfoManager::getPackageFileName" in
   * http://docs.ros.org/en/hydro/api/camera_info_manager/html/camera__info__manager_8cpp_source.html
   * @param filename The filename in 'package://' format
   * @return The actual file path
   */
  std::string GetPackageFilename(const std::string &filename);
};
} // namespace sensor_fusion_baseline
