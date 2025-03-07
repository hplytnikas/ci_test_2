/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024-2025 Authors:
 *   - Yee Hsien Quek <yequek@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include "slam_common/utils.hpp"

namespace slam {

namespace utils {

geometry_msgs::msg::PoseStamped ZeroPose() {
  geometry_msgs::msg::PoseStamped pose;

  return pose;
}

geometry_msgs::msg::PoseStamped CreatePoseStamped(double x, double y, double theta, const rclcpp::Time &timestamp) {
  geometry_msgs::msg::PoseStamped pose;

  return pose;
}

gtsam::Pose2 PoseStampedToPose2(const geometry_msgs::msg::PoseStamped &pose_stamped) {
  gtsam::Pose2 pose;
  return pose;
}

geometry_msgs::msg::PoseStamped Pose2ToPoseStamped(const gtsam::Pose2 &pose, const rclcpp::Time &timestamp) {
  geometry_msgs::msg::PoseStamped pose_stamped;

  return pose_stamped;
}

geometry_msgs::msg::PoseStamped CalculatePoseDelta(const geometry_msgs::msg::PoseStamped &pose1,
                                                   const geometry_msgs::msg::PoseStamped &pose2) {
  geometry_msgs::msg::PoseStamped delta;
  return delta;
}

geometry_msgs::msg::PoseStamped ComposePoses(const geometry_msgs::msg::PoseStamped &pose1,
                                             const geometry_msgs::msg::PoseStamped &pose2) {
  geometry_msgs::msg::PoseStamped composed;
  return composed;
}

std::string PoseStampedToString(const geometry_msgs::msg::PoseStamped &pose_stamped) {

  return "";
}

tf2::Transform ZeroTfTransform() {
  tf2::Transform transform;
  return transform;
}

tf2::Transform PoseToTfTransform(const geometry_msgs::msg::Pose &pose) {
  tf2::Transform transform;
  return transform;
}

geometry_msgs::msg::Pose TfTransformToPose(const geometry_msgs::msg::Transform &transform) {
  geometry_msgs::msg::Pose pose;

  return pose;
}

geometry_msgs::msg::PoseStamped TransformStampedToPoseStamped(const geometry_msgs::msg::TransformStamped &transform_stamped) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  return pose_stamped;
}

ConeMap TransformMap(const ConeMap &map, const geometry_msgs::msg::Pose &pose) {
  ConeMap result = {};

  return result;
}

std::string GetCurrentTimeString() {
  return "";
}

} // end namespace utils

} // end namespace slam
