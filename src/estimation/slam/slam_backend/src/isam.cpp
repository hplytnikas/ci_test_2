/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024-2025 Authors:
 *   - Quek Yee Hsien <yequek@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "slam_backend/isam.hpp"

namespace slam {

/*
 ******************************************
 ******** INITIALIZATION AND SETUP ********
 ******************************************
 */

ISAM::ISAM(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer)
    : SlamBackend(node_handle, visualizer) {
  Init();
}

ISAM::~ISAM() {}

void ISAM::Init() {
  InitGraph();
  AddPosePrior(utils::ZeroPose());
}

void ISAM::InitGraph() {
  // Load parameters
  max_edges_ = node_handle_->GetParameter<size_t>("slam_backend.isam.max_edges_before_pruning", 10000);

  max_edges_after_pruning_ = node_handle_->GetParameter<size_t>("slam_backend.isam.max_edges_after_pruning", 8000);

  if (max_edges_after_pruning_ > max_edges_) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),
                        "ISAM: Max edges after pruning should be less than max edges before pruning. \r\nMax edges: "
                            << max_edges_ << ". Max edges after pruning: " << max_edges_after_pruning_);
    max_edges_after_pruning_ = max_edges_;
  }

  auto pose_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.isam.pose_symbol", "X");
  pose_symbol_ = pose_symbol_str.empty() ? 'X' : pose_symbol_str[0];

  auto landmark_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.isam.landmark_symbol", "L");
  landmark_symbol_ = landmark_symbol_str.empty() ? 'L' : landmark_symbol_str[0];

  // Load all noise models
  std::vector<double> noise;
  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam.pose_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM: Odometry noise model should have 3 params, but has " << noise.size());
    noise = {1, 1, 1};
  }
  pose_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam.pose_prior_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM: Pose prior noise model should have 3 params, but has " << noise.size());
    noise = {1, 1, 1};
  }
  pose_prior_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam.landmark_prior_noise",
                                                          std::vector<double>{0.02, 0.02});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM: Landmark prior noise model should have 2 params, but has " << noise.size());
    noise = {0.02, 0.02};
  }
  landmark_prior_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam.landmark_noise",
                                                          std::vector<double>{0.1, 0.7});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM: Landmark prior noise model should have 2 params, but has " << noise.size());
    noise = {0.1, 0.7};
  }
  landmark_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  relinearization_interval_ = node_handle_->GetParameter<int>("slam_backend.isam.relinearization_interval", 10);

  // Create visualization graph
  graph_visualization_ = std::make_shared<GraphVisualization>();
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "ISAM: Visualization graph initialized");

  // Create ISAM object
  isam_ = std::make_unique<gtsam::NonlinearISAM>(relinearization_interval_);

  enable_pruning_ = false;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "ISAM: Graph initialized and noise models set.");
}

void ISAM::AddPosePrior(const geometry_msgs::msg::PoseStamped &prior_pose) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function

}

void ISAM::AddLandmarkPriors() {
  EASY_FUNCTION(profiler::colors::Red); // Time this function

}

void ISAM::SetFixedMap(const ConeMap &fixed_map) {
  is_map_fixed_ = true;
}

void ISAM::SwitchToLocalizationMode() {

}

/*
 ******************************************
 ************** GRAPH UPDATES *************
 ******************************************
 */

void ISAM::Update(const geometry_msgs::msg::PoseStamped pose_difference, const std::vector<Landmark> &landmarks) {
  EASY_FUNCTION(profiler::colors::Yellow); // Time this function

}

void ISAM::AddPose(const geometry_msgs::msg::PoseStamped &pose_difference) {
  EASY_FUNCTION(profiler::colors::Red);

}

void ISAM::AddLandmark(const Landmark &landmark) {
  EASY_FUNCTION(profiler::colors::Red);

}

void ISAM::Optimize() {
  EASY_FUNCTION(profiler::colors::Orange);

}

void ISAM::PruneGraph() {
  EASY_FUNCTION(profiler::colors::Brown);

}

void ISAM::UpdatePoseEstimates(const gtsam::Values &optimization_result) {
  EASY_FUNCTION(profiler::colors::Green);

}

void ISAM::UpdateMapEstimate() {
  
}

Symbol ISAM::MakePoseSymbol(uint64_t pose_id) { return Symbol(pose_symbol_, pose_id); }

Symbol ISAM::MakeLandmarkSymbol(uint64_t landmark_id) { return Symbol(landmark_symbol_, landmark_id); }

void ISAM::PrintEstimateSymbols() {

}

void ISAM::PrintFactorGraph() {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
}
  
} // end namespace slam