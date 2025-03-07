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

#include "slam_backend/fixed_lag_smoothing_backend.hpp"

namespace slam {

FixedLagSmoothingBackend::FixedLagSmoothingBackend(std::shared_ptr<SlamNode> node_handle,
                                                   std::shared_ptr<Visualizer> visualizer)
    : SlamBackend(node_handle, visualizer) {
  Init();
}

FixedLagSmoothingBackend::~FixedLagSmoothingBackend() {}

void FixedLagSmoothingBackend::Init() {
  LoadNoiseModels();
  LoadOptimizationParameters();
  InitSmoother();
}

void FixedLagSmoothingBackend::LoadNoiseModels() {
  std::vector<double> noise;
  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.pose_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Odometry noise model should have 3 params, but has "
                            << noise.size());
    noise = {1, 1, 1};
  }
  pose_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.pose_prior_noise",
                                                          std::vector<double>{1, 1, 1});
  if (noise.size() != 3) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Pose prior noise model should have 3 params, but has "
                            << noise.size());
    noise = {1, 1, 1};
  }
  pose_prior_noise_ = Diagonal::Sigmas(Vector3(noise[0], noise[1], noise[2]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.landmark_prior_noise",
                                                          std::vector<double>{0.02, 0.02});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Landmark prior noise model should have 2 params, but has "
                            << noise.size());
    noise = {0.02, 0.02};
  }
  landmark_prior_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));

  noise = node_handle_->GetParameter<std::vector<double>>("slam_backend.fixed_lag_smoothing.landmark_noise",
                                                          std::vector<double>{0.1, 0.7});
  if (noise.size() != 2) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),
                        "FixedLagSmoothingBackend: Landmark prior noise model should have 2 params, but has "
                            << noise.size());
    noise = {0.1, 0.7};
  }
  landmark_noise_ = Diagonal::Sigmas(Vector2(noise[0], noise[1]));
}

void FixedLagSmoothingBackend::LoadOptimizationParameters() {
  const auto optimizer_iterations =
      node_handle_->GetParameter<int>("slam_backend.fixed_lag_smoothing.optimizer_iterations", 20);
  const auto optimizer_error_tolerance =
      node_handle_->GetParameter<double>("slam_backend.fixed_lag_smoothing.optimizer_error_tolerance", 0.0001);

  optimization_parameters_.maxIterations = optimizer_iterations;
  optimization_parameters_.relativeErrorTol = optimizer_error_tolerance;
  optimization_parameters_.absoluteErrorTol = optimizer_error_tolerance;
  // optimization_parameters_.setVerbosity("ERROR");

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "FixedLagSmoothingBackend: Optimization parameters loaded.");
}

void FixedLagSmoothingBackend::InitSmoother() {
  const double lag = node_handle_->GetParameter<double>("slam_backend.fixed_lag_smoothing.lag", 750.0);

  auto pose_symbol_str = node_handle_->GetParameter<std::string>("slam_backend.fixed_lag_smoothing.pose_symbol", "X");
  pose_symbol_ = pose_symbol_str.empty() ? 'X' : pose_symbol_str[0];

  auto landmark_symbol_str =
      node_handle_->GetParameter<std::string>("slam_backend.fixed_lag_smoothing.landmark_symbol", "L");
  landmark_symbol_ = landmark_symbol_str.empty() ? 'L' : landmark_symbol_str[0];

  // Initialize smoother
  smoother_ = BatchFixedLagSmoother(lag, optimization_parameters_);

  NonlinearFactorGraph new_factors;
  Values new_values;
  BatchFixedLagSmoother::KeyTimestampMap new_timestamps;

  // Create prior factor for first pose
  const auto prior_key = MakePoseSymbol(0);
  new_factors.push_back(PriorFactor<Pose2>(prior_key, gtsam::Pose2(0, 0, 0), pose_prior_noise_));
  new_values.insert(prior_key, gtsam::Pose2(0, 0, 0));
  new_timestamps[prior_key] = time_;
  time_ += 1.0;

  // Add prior to optimizer
  smoother_.update(new_factors, new_values, new_timestamps);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "FixedLagSmoothingBackend: Graph initialized and noise models set.");
}

void FixedLagSmoothingBackend::SetFixedMap(const ConeMap &fixed_map) {
}

void FixedLagSmoothingBackend::SwitchToLocalizationMode() { }

void FixedLagSmoothingBackend::Update(const geometry_msgs::msg::PoseStamped pose_difference, const std::vector<Landmark> &landmarks) {
  EASY_FUNCTION(profiler::colors::Yellow); // Time this function

}

void FixedLagSmoothingBackend::AddPose(const geometry_msgs::msg::PoseStamped pose_difference, NonlinearFactorGraph &new_factors,
                                       Values &new_values, BatchFixedLagSmoother::KeyTimestampMap &new_timestamps) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  
}

void FixedLagSmoothingBackend::AddLandmarks(const std::vector<Landmark> &landmarks, NonlinearFactorGraph &new_factors,
                                            Values &new_values,
                                            BatchFixedLagSmoother::KeyTimestampMap &new_timestamps) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  
}

void FixedLagSmoothingBackend::AddLandmarkPriors(const ConeMap &fixed_map) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  
}

void FixedLagSmoothingBackend::Optimize() {
  EASY_FUNCTION(profiler::colors::Orange); // Time this function

}

void FixedLagSmoothingBackend::UpdateEstimates(const Values &result) {
  
}

Symbol FixedLagSmoothingBackend::MakePoseSymbol(uint64_t pose_id) const { return Symbol(pose_symbol_, pose_id); }
Symbol FixedLagSmoothingBackend::MakeLandmarkSymbol(uint64_t landmark_id) const {
  return Symbol(landmark_symbol_, landmark_id);
}

} // end namespace slam
