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

namespace sensor_fusion_baseline {

void SensorFusion::LoadParameters() {
  // ------------------- declare Parameters -------------------
  // Subscriber Topic Names
  this->declare_parameter<std::string>("topics.sub.points_compensated", "/perception/lidar_motion_compensator/compensated_pc");
  this->declare_parameter<std::string>("topics.sub.fw_bbox", "/perception/depth_estimation/bbox_depth");
  this->declare_parameter<std::string>("topics.sub.debug.fw_cam", "/sensors/forward_camera/image_color");
  
  // Publisher Topic Names
  this->declare_parameter<std::string>("topics.pub.runtime", "/perception/runtimes");
  this->declare_parameter<std::string>("topics.pub.debug.img_overlay", "/perception/sensor_fusion_baseline/debug/img_overlay");
  this->declare_parameter<std::string>("topics.pub.cone_array", "/perception/fusion/cone_array");
  this->declare_parameter<std::string>("topics.pub.debug.bbox_fusion", "/perception/sensor_fusion_baseline/debug/bbox_depth");
  this->declare_parameter<std::string>("topics.pub.debug.cone_markers", "/perception/fusion/cone_markers");

  // Intrinsics
  this->declare_parameter<std::string>("intrinsics_file", "package://camera_launcher/calibration/forward_camera_castor.yaml");

  // Frame IDs parameters
  this->declare_parameter<std::string>("frame_ids.camera", "pylon_camera");
  this->declare_parameter<std::string>("frame_ids.base_link", "base_link");
  this->declare_parameter<std::string>("frame_ids.reference_frame", "odom");
  this->declare_parameter<std::string>("lidar_mode", "hesai");
  this->declare_parameter<std::string>("car_mode", "dufour");

  // Image resolution parameters
  this->declare_parameter<int>("image_width", 2592);
  this->declare_parameter<int>("image_height", 352);

  // ------------------- get Parameters -------------------
  // Subscriber Topic Names
  this->get_parameter("topics.sub.points_compensated", topic_points_compensated_);
  this->get_parameter("topics.sub.fw_bbox", topic_fw_bbox_);
  this->get_parameter("topics.sub.debug.fw_cam", topic_fw_cam_);

  // Publisher Topic Names
  this->get_parameter("topics.pub.runtime", topic_runtime_);
  this->get_parameter("topics.pub.debug.img_overlay", topic_img_overlay_);
  this->get_parameter("topics.pub.cone_array", topic_cone_array_);
  this->get_parameter("topics.pub.debug.bbox_fusion", topic_bbox_fusion_);
  this->get_parameter("topics.pub.debug.cone_markers", topic_cone_markers_);

  // Intrinsics
  this->get_parameter("intrinsics_file", intrinsics_file_);

  // ------------------- ------------------- -------------------

  // Frame IDs parameters
  this->get_parameter("frame_ids.camera", camera_frame_id_);
  this->get_parameter("frame_ids.base_link", base_link_frame_id_);
  this->get_parameter("frame_ids.reference_frame", reference_frame_id_);
  std::string lidar_mode_;
  this->get_parameter("lidar_mode", lidar_mode_);
  if (lidar_mode_ == "hesai") {
    lidar_frame_id_ = "hesai_lidar";
  } else if (lidar_mode_ == "ouster") {
    lidar_frame_id_ = "os_lidar";
  }

  std::string car_mode_;
  this->get_parameter("car_mode", car_mode_);
  std::string car_name = "CAR_NAME";
  size_t pos = intrinsics_file_.find(car_name);
  if (pos != std::string::npos) intrinsics_file_.replace(pos, car_name.length(), car_mode_);

  // Image resolution parameters
  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);


  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// Loads all camera's intrinsics from intrinsics server, and extrinsics
// (MRH2Camera) from /tf_static
void SensorFusion::LoadIntrinsics() {
  // Load camera intrinsics from intrinsics server
  try {
    std::string resolved_filename = GetPackageFilename(intrinsics_file_);
    YAML::Node intrinsics = YAML::LoadFile(resolved_filename);
    const std::vector<float> &data = intrinsics["camera_matrix"]["data"].as<std::vector<float>>();
    std::memcpy(K_mat_.data, data.data(), 9 * sizeof(float));
    const std::vector<float> &distortion = intrinsics["distortion_coefficients"]["data"].as<std::vector<float>>();
    std::memcpy(D_mat_.data, distortion.data(), 14 * sizeof(float));
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load intrinsics -> " << e.what());
  }
  RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded from file: %s", intrinsics_file_.c_str());
}

void SensorFusion::TryLoadStaticTransforms() {
  try {
    geometry_msgs::msg::TransformStamped tf_transform =
        tf_buffer_->lookupTransform(camera_frame_id_, lidar_frame_id_, tf2::TimePointZero, std::chrono::milliseconds(100));
    auto rotation =
        tf2::Matrix3x3(tf2::Quaternion(tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
                                      tf_transform.transform.rotation.z, tf_transform.transform.rotation.w));
    float r[] = {
        static_cast<float>(rotation[0].x()), static_cast<float>(rotation[0].y()), static_cast<float>(rotation[0].z()),
        static_cast<float>(rotation[1].x()), static_cast<float>(rotation[1].y()), static_cast<float>(rotation[1].z()),
        static_cast<float>(rotation[2].x()), static_cast<float>(rotation[2].y()), static_cast<float>(rotation[2].z())};
    cv::Mat r_mat(3, 3, CV_32F, r);
    cv::Rodrigues(r_mat, rvecR_);

    float t[] = {static_cast<float>(tf_transform.transform.translation.x),
                static_cast<float>(tf_transform.transform.translation.y),
                static_cast<float>(tf_transform.transform.translation.z)};
    std::memcpy(t_mat_.data, t, 3 * sizeof(float));
  } catch (tf2::TransformException &exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Exception during transform lookup from [" << lidar_frame_id_ << "] to ["
                                                                                      << camera_frame_id_
                                                                                      << "]: " << exception.what());
  }
}

void SensorFusion::LoadCalibrationParameters() {
  LoadIntrinsics();

  // Initialize the timer to check for static transform and call TryLoadStaticTransforms
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SensorFusion::TryLoadStaticTransforms, this));
}

void SensorFusion::SubscribeTopics() {
  sub_pointcloud_.subscribe(this, topic_points_compensated_);
  sub_bbox_.subscribe(this, topic_fw_bbox_);
  sub_image_.subscribe(this, topic_fw_cam_);

  // SyncPolicy(20): ApproximateTime sync with queue of 20 for all the listed
  // synchronised topics Additional parameters for efficient matching of topic
  // messages
  sync_.reset(new Sync(SyncPolicy(20), sub_pointcloud_, sub_bbox_, sub_image_));

  sync_->registerCallback(&sensor_fusion_baseline::SensorFusion::DetectCones, this);
}

void SensorFusion::AdvertiseTopics() {
  pub_runtime_ = this->create_publisher<perception_msgs::msg::PipelineRuntime>(topic_runtime_, 1);
  //if (debug_) {
    pub_img_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(topic_img_overlay_, 1);
    pub_bbox_fusion_ = this->create_publisher<perception_msgs::msg::BoxArrayDebug>(topic_bbox_fusion_, 1);
  //}
  pub_cone_array_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_cone_array_, 1);
  pub_cone_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_cone_markers_, 1);
}


void SensorFusion::PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array) {
  visualization_msgs::msg::MarkerArray markerArray;
  int markerId = 0;

  for (const auto &centroid : cone_array.cones) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_frame_id_; // Change to your point cloud's frame ID
    marker.header.stamp = cone_array.header.stamp; 
    marker.ns = "cones";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = centroid.position.x;
    marker.pose.position.y = centroid.position.y;
    marker.pose.position.z = centroid.position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2; // Specify the size of the marker
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0; // Color the marker red
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Make the marker opaque

    markerArray.markers.push_back(marker);
  }

  pub_cone_markers_->publish(markerArray);
}

std::string SensorFusion::GetPackageFilename(const std::string &filename) {
  // Scan filename from after "package://" until next '/' and extract
  // package name.
  size_t prefix_len = std::string("package://").length();
  size_t rest = filename.find('/', prefix_len);
  std::string package(filename.substr(prefix_len, rest - prefix_len));
  // Look up the ROS package path name.
  std::string pkgPath(ament_index_cpp::get_package_share_directory(package));
  if (pkgPath.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "unknown package: " << package << " (ignored)");
    return pkgPath;
  } else {
    // Construct file name from package location and remainder of URL.
    return pkgPath + filename.substr(rest);
  }
}

} // namespace sensor_fusion_baseline
