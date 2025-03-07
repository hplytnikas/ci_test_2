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
#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ctime>
#include <string>

#include "slam_common/aliases.hpp"

namespace slam {

namespace utils {

/*
 * Create PoseStamped message with zero pose and unit quaternion
 */
geometry_msgs::msg::PoseStamped ZeroPose();

/*
 * Create PoseStamped message with x, y, theta and timestamp
 */
geometry_msgs::msg::PoseStamped CreatePoseStamped(double x, double y, double theta, const rclcpp::Time &timestamp);

/*
 * Convert PoseStamped to gtsam::Pose2
 */
gtsam::Pose2 PoseStampedToPose2(const geometry_msgs::msg::PoseStamped &pose_stamped);

/*
 * Convert gtsam::Pose2 to PoseStamped
 */
geometry_msgs::msg::PoseStamped Pose2ToPoseStamped(const gtsam::Pose2 &pose, const rclcpp::Time &timestamp);

/*
 * Calculate delta between two poses, keeping the timestamp from the first one
 */
geometry_msgs::msg::PoseStamped CalculatePoseDelta(const geometry_msgs::msg::PoseStamped &pose1,
                                                   const geometry_msgs::msg::PoseStamped &pose2);

/*
 * Compose two poses, keeping the timestamp of the first one
 */
geometry_msgs::msg::PoseStamped ComposePoses(const geometry_msgs::msg::PoseStamped &pose1,
                                             const geometry_msgs::msg::PoseStamped &pose2);

/* 
 * String representation of PoseStamped
 */
std::string PoseStampedToString(const geometry_msgs::msg::PoseStamped &pose_stamped);

/* 
 * Create tf2::Transform with zero translation and unit quaternion
 */
tf2::Transform ZeroTfTransform();

/*
 * Convert pose to tf2 Transform object
 */
tf2::Transform PoseToTfTransform(const geometry_msgs::msg::Pose &pose);

/*
 * Convert tf geometry msg Transform object to pose
 */
geometry_msgs::msg::Pose TfTransformToPose(const geometry_msgs::msg::Transform &transform);

/* 
 * Convert transform stamped to psoe stamped
*/
geometry_msgs::msg::PoseStamped TransformStampedToPoseStamped(const geometry_msgs::msg::TransformStamped &transform_stamped);

/*
 * Transform map using transformation from a 2D pose (transform)
 */
ConeMap TransformMap(const ConeMap &map, const geometry_msgs::msg::Pose &pose);

/*
 * Get a string for the current datetime
 */
std::string GetCurrentTimeString();

} // end namespace utils

} // end namespace slam
