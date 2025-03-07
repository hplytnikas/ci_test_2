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

#include "slam_frontend/global_map_tracker.hpp"

namespace slam {

GlobalMapTracker::GlobalMapTracker(std::shared_ptr<SlamNode> node_handle)
    : node_handle_(node_handle), mapping_mode_(true) {
  LoadParameters();
}

GlobalMapTracker::~GlobalMapTracker() {}

void GlobalMapTracker::LoadParameters() {
  probability_updates_enabled_ =
      node_handle_->GetParameter<bool>("global_map_tracker.probability_updates_enabled", false);
  cone_ignore_enabled_ = node_handle_->GetParameter<bool>("global_map_tracker.cone_ignore_enabled", false);
  log_throttle_ms_ = 1000 * node_handle_->GetParameter<double>("logging.throttle", 2.0);
  color_trust_close_threshold_ =
      node_handle_->GetParameter<double>("global_map_tracker.color_trust.close_threshold", 7.0);
  color_trust_far_threshold_ = node_handle_->GetParameter<double>("global_map_tracker.color_trust.far_threshold", 15.0);
  color_trust_min_value_ = node_handle_->GetParameter<double>("global_map_tracker.color_trust.min_value", 0.4);
  min_observation_count_ = node_handle_->GetParameter<int>("global_map_tracker.min_observation_count", 3);
  accepted_cone_observation_timeframe_s_ =
      node_handle_->GetParameter<int>("global_map_tracker.accepted_cone_observation_timeframe_s", 5);
  min_cone_probability_ = node_handle_->GetParameter<double>("global_map_tracker.min_cone_probability", 0.4);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Parameters loaded.");
}

void GlobalMapTracker::SetFixedMap(const ConeMap &fixed_map) {
  size_t global_cone_id = 0;
  for (const auto &cone : fixed_map) {
    global_map_.push_back(GlobalCone(cone));
    global_map_.back().cone.id_cone = global_cone_id++;
  }

  mapping_mode_ = false;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Fixed map set.");
}

void GlobalMapTracker::SwitchToLocalizationMode() {
  mapping_mode_ = false;
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "GlobalMapTracker: Switched to localization mode.");
}

ConeMap GlobalMapTracker::FullGlobalMap() {
  // Return a copy of the global map
  ConeMap result_map;
  
  return result_map;
}

ConeMap GlobalMapTracker::OnlineMap() {
  ConeMap result_map;
  
  return result_map;
}

void GlobalMapTracker::ProcessAssociations(const std::vector<Association> &associations,
                                           ConeMap &observations_global_frame, const geometry_msgs::msg::PoseStamped &pose_stamped) {
  EASY_FUNCTION(profiler::colors::Green); // Time this function
  return;
}

void GlobalMapTracker::AddNewCone(autonomous_msgs::msg::Cone &cone, const geometry_msgs::msg::PoseStamped &pose_stamped) {
  return;
}

void GlobalMapTracker::UpdateMap(const ConeMap &new_cones) {
  return;
}

// Probability updates
void GlobalMapTracker::UpdateIsConeProbability(autonomous_msgs::msg::Cone &global_map_cone,
                                               autonomous_msgs::msg::Cone &observed_cone) {
  return;
}

void GlobalMapTracker::UpdateConeColorProbabilities(GlobalCone &global_map_cone,
                                                    autonomous_msgs::msg::Cone &observed_cone,
                                                    const geometry_msgs::msg::PoseStamped &pose_stamped) {
  return;
}

double GlobalMapTracker::CalculateColorTrust(double range) const {
  return 0.0;
}

bool GlobalMapTracker::TrustColor(const autonomous_msgs::msg::Cone &cone) const {
  return false;
}

void GlobalMapTracker::IgnoreConeIfIrrelevant(GlobalCone &cone, const geometry_msgs::msg::PoseStamped &pose_stamped) {
  return;
}

} // end namespace slam
