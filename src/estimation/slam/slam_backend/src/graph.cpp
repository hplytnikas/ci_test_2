/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Christoforos Nicolaou <cnicolaou@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "slam_backend/graph.hpp"

namespace slam {

Graph::Graph(size_t max_edges, unsigned char pose_symbol, unsigned char landmark_symbol,
             Diagonal::shared_ptr pose_noise, Diagonal::shared_ptr pose_prior_noise,
             Diagonal::shared_ptr landmark_prior_noise, Diagonal::shared_ptr landmark_noise)
    : max_edges_(max_edges), pose_symbol_(pose_symbol), landmark_symbol_(landmark_symbol), pose_noise_(pose_noise),
      pose_prior_noise_(pose_prior_noise), landmark_prior_noise_(landmark_prior_noise),
      landmark_noise_(landmark_noise) {
  // First pose is (0, 0, 0)
  graph_visualization_ = std::make_shared<GraphVisualization>();
}

Graph::~Graph() {}

void Graph::AddPosePrior(const geometry_msgs::msg::PoseStamped &prior_pose) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function

}

void Graph::AddLandmarkPriors(const ConeMap &fixed_map) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  
}

void Graph::AddPose(const geometry_msgs::msg::PoseStamped &pose_difference) {
  EASY_FUNCTION(profiler::colors::Red); // Time this function
  
}

void Graph::AddLandmark(const Landmark &landmark) {
  EASY_FUNCTION(profiler::colors::Red);         // Time this function
  
}

// Unused for now
void Graph::AddLandmarks(const std::vector<Landmark> &landmarks) {

}

void Graph::PruneGraph() {
  EASY_FUNCTION(profiler::colors::Brown); // Time this function
  
}

void Graph::UpdateEstimates(gtsam::Values optimization_result) {
  
}

std::vector<geometry_msgs::msg::PoseStamped> Graph::UpdatedPoseEstimates() const {
  std::vector<geometry_msgs::msg::PoseStamped> updated_pose_estimates;

  return updated_pose_estimates;
}

ConeMap Graph::UpdatedMapEstimate() const {
  ConeMap updated_map_estimate;

  return updated_map_estimate;
}

gtsam::NonlinearFactorGraph Graph::NonlinearFactorGraph() const { return graph_gtsam_; }

std::shared_ptr<GraphVisualization> Graph::GetGraphVisualization() const {
  EASY_FUNCTION(profiler::colors::Green);
  return graph_visualization_;
}

Values Graph::FilteredInitialEstimate() const {
  // Exclude pruned nodes
  gtsam::Values filtered;

  return filtered;
}

Symbol Graph::MakePoseSymbol(uint64_t pose_id) { return Symbol(pose_symbol_, pose_id); }

Symbol Graph::MakeLandmarkSymbol(uint64_t landmark_id) { return Symbol(landmark_symbol_, landmark_id); }

size_t Graph::NumOdometryNodes() const { return odometry_key_pose_queue_.size(); }

size_t Graph::NumLandmarkNodes() const { return landmark_key_counts_.size(); }

size_t Graph::NumEdges() const { return num_pose_edges + num_landmark_edges; }

// TEST: get size of factor_idx_to_keys_
size_t Graph::FactorIndexToKeysSize() const { return factor_index_to_keys_.size(); }

} // end namespace slam
