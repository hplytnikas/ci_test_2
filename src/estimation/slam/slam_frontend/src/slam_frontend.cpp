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

#include "slam_frontend/slam_frontend.hpp"

#include <easy/profiler.h>

namespace slam {

SlamFrontend::SlamFrontend(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer,
                           std::unique_ptr<DataAssociation> data_association, std::unique_ptr<SlamBackend> slam_backend,
                           std::unique_ptr<GlobalMapTracker> global_map_tracker)
    : node_handle_(node_handle), visualizer_(visualizer), data_association_(std::move(data_association)),
      slam_backend_(std::move(slam_backend)), global_map_tracker_(std::move(global_map_tracker)) {
  LoadParameters();

  map_loader_ = std::make_unique<MapLoader>(node_handle_);
  LoadMapIfNeeded();

  InitSubscribers();

  // Initially publish zero pose
  node_handle_->BroadcastInitialPose(node_handle_->get_clock()->now());
  previous_pose_ = utils::ZeroPose();
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "SlamFrontend: Initialization done. First map->odom published.");
}

SlamFrontend::~SlamFrontend() {}

void SlamFrontend::InitSubscribers() {
  node_handle_->CreateConeCallbackSubscriber(std::bind(&SlamFrontend::ConeCallback, this,
                                                       std::placeholders::_1)); // Create subscribers
}

void SlamFrontend::LoadParameters() {
  mission_type_ = static_cast<MissionType>(
      node_handle_->GetParameter<int>("mission_type", static_cast<int>(MissionType::kAutocross)));
  mapping_mode_ = node_handle_->GetParameter<bool>("mapping_mode", true);
  save_map_on_mission_finished_ = node_handle_->GetParameter<bool>("save_map_on_mission_finished", true);
  save_trajectory_on_mission_finished_ = node_handle_->GetParameter<bool>("save_trajectory_on_mission_finished", true);
  log_throttle_ms_ = 1000 * node_handle_->GetParameter<double>("logging.throttle", 2.0);
  generate_acceleration_map_ = false;
  acceleration_map_generated_ = false;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "SlamFrontend: Parameters loaded - Mission type: "
                                                     << mission_type_ << ". Mapping mode: " << mapping_mode_);
}

void SlamFrontend::LoadMapIfNeeded() {
  return;
}

void SlamFrontend::ConeCallback(const autonomous_msgs::msg::ConeArray &msg) {
  EASY_FUNCTION(profiler::colors::Magenta); // Time this function

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_,
                              "SlamFrontend: Cone callback.");
}

std::vector<Landmark> SlamFrontend::AssociationsToLandmarks(const ConeMap &observations_local_frame,
                                                            const ConeMap &observations_global_frame,
                                                            const std::vector<Association> &associations) {
  std::vector<Landmark> landmarks;
  return landmarks;
}

void SlamFrontend::CheckMissionProgress() {
  return;
}

} // end namespace slam
