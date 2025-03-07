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
#include "slam_frontend/nn_data_association.hpp"

namespace slam {

NearestNeighborDataAssociation::NearestNeighborDataAssociation(std::shared_ptr<SlamNode> node_handle)
    : DataAssociation(node_handle) {
  LoadParameters();
}

NearestNeighborDataAssociation::~NearestNeighborDataAssociation() {}

void NearestNeighborDataAssociation::LoadParameters() {
  min_distance_threshold_m_ =
      node_handle_->GetParameter<double>("data_association.nearest_neighbor.min_distance_threshold_m", 1.2);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "NearestNeighborDataAssociation: Parameters loaded.");
}

std::vector<Association> NearestNeighborDataAssociation::Associate(const ConeMap &observations,
                                                                   const ConeMap &global_map) {
  EASY_FUNCTION(profiler::colors::Green); // Time this function
  // Setup variables
  const size_t num_observed_cones = observations.size();

  // Cone associations between observed cones and global map cones
  std::vector<Association> associations;
  associations.reserve(num_observed_cones);

  return associations;
}

} // end namespace slam
