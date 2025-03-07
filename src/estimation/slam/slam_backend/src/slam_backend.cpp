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

#include "slam_backend/slam_backend.hpp"

namespace slam {

SlamBackend::SlamBackend(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer)
    : node_handle_(node_handle), visualizer_(visualizer), is_map_fixed_(false) {
  // Load throttling parameter
  log_throttle_ms_ = 1000 * node_handle_->GetParameter<double>("logging.throttle", 2.0);
}

SlamBackend::~SlamBackend() {}

// Getters
geometry_msgs::msg::PoseStamped SlamBackend::LatestPoseEstimate() const {
  if (pose_estimates_.size() > 0) {
    return pose_estimates_.back();
  }
  return utils::ZeroPose();
}

std::vector<geometry_msgs::msg::PoseStamped> SlamBackend::PoseEstimates() const { return pose_estimates_; }

size_t SlamBackend::ConeCount() const { return global_map_estimate_.size(); }

ConeMap SlamBackend::GlobalMapEstimate() const { return global_map_estimate_; }

} // end namespace slam
