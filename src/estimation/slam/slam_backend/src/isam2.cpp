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

#include "slam_backend/isam2.hpp"

namespace slam {

/*
 ******************************************
 ******** INITIALIZATION AND SETUP ********
 ******************************************
 */

ISAM2::ISAM2(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer)
    : SlamBackend(node_handle, visualizer) {
  Init();
}

ISAM2::~ISAM2() {}

void ISAM2::Init() {
  InitGraph();
  AddPosePrior(utils::ZeroPose());
}

void ISAM2::InitGraph() {
  max_edges_ = node_handle_->GetParameter<size_t>("slam_backend.isam2.max_edges_before_pruning", 10000);

  max_edges_after_pruning_ = node_handle_->GetParameter<size_t>("slam_backend.isam2.max_edges_after_pruning", 8000);

  auto pose_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.isam2.pose_symbol", "X");
  pose_symbol_ = pose_symbol_str.empty() ? 'X' : pose_symbol_str[0];

  auto landmark_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.isam2.landmark_symbol", "L");
  landmark_symbol_ = landmark_symbol_str.empty() ? 'L' : landmark_symbol_str[0];

  // Load all noise models
  std::vector<double> noise;
  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam2.pose_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM2: Odometry noise model should have 3 params, but has " << noise.size());
    noise = {1, 1, 1};
  }
  pose_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam2.pose_prior_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM2: Pose prior noise model should have 3 params, but has " << noise.size());
    noise = {1, 1, 1};
  }
  pose_prior_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam2.landmark_prior_noise",
                                                          std::vector<double>{0.02, 0.02});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM2: Landmark prior noise model should have 2 params, but has " << noise.size());
    noise = {0.02, 0.02};
  }
  landmark_prior_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.isam2.landmark_noise",
                                                          std::vector<double>{0.1, 0.7});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "ISAM2: Landmark prior noise model should have 2 params, but has " << noise.size());
    noise = {0.1, 0.7};
  }
  landmark_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  relinearize_threshold_ = node_handle_->GetParameter<double>("slam_backend.isam2.relinearize_threshold", 0.01);

  relinearize_skip_ = node_handle_->GetParameter<int>("slam_backend.isam2.relinearize_skip", 10);

  // Create visualization graph
  graph_visualization_ = std::make_shared<GraphVisualization>();
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "ISAM2: Visualization graph initialized");

  // Assign relinearization params to isam2 params
  isam2_params_.relinearizeThreshold = relinearize_threshold_;
  isam2_params_.relinearizeSkip = relinearize_skip_;

  // Create ISAM2 object
  isam2_ = std::make_unique<gtsam::ISAM2>(isam2_params_);

  enable_pruning_ = false;

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "ISAM2: Graph initialized and noise models set.");
}

void ISAM2::AddPosePrior(const geometry_msgs::msg::PoseStamped &prior_pose) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function

}

void ISAM2::AddLandmarkPriors() {
  EASY_FUNCTION(profiler::colors::Red); // Time this function

}

void ISAM2::SetFixedMap(const ConeMap &fixed_map) {
  
}

void ISAM2::SwitchToLocalizationMode() {

}

/*
 ******************************************
 ************** GRAPH UPDATES *************
 ******************************************
 */

void ISAM2::Update(const geometry_msgs::msg::PoseStamped pose_difference, const std::vector<Landmark> &landmarks) {
  EASY_FUNCTION(profiler::colors::Yellow); // Time this function

}

void ISAM2::AddPose(const geometry_msgs::msg::PoseStamped &pose_difference) {
  EASY_FUNCTION(profiler::colors::Red);

}

void ISAM2::AddLandmark(const Landmark &landmark) {
  EASY_FUNCTION(profiler::colors::Red);

}

void ISAM2::Optimize() {
  EASY_FUNCTION(profiler::colors::Orange);

}

void ISAM2::PruneGraph() {
  EASY_FUNCTION(profiler::colors::Brown); // Time this function

  RCLCPP_INFO_STREAM_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), log_throttle_ms_, "ISAM2: Pruning graph.");

  return;
}

void ISAM2::UpdatePoseEstimates(const gtsam::Values &optimization_result) {
  EASY_FUNCTION(profiler::colors::Green);

}

void ISAM2::UpdateMapEstimate() {

}

Symbol ISAM2::MakePoseSymbol(uint64_t pose_id) { return Symbol(pose_symbol_, pose_id); }

Symbol ISAM2::MakeLandmarkSymbol(uint64_t landmark_id) { return Symbol(landmark_symbol_, landmark_id); }

void ISAM2::PrintEstimateSymbols() {

}

void ISAM2::PrintFactorGraph() {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function (This takes very long and may become a bottleneck for execution time)
}
  
} // end namespace slam