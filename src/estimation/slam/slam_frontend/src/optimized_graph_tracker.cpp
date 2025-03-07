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

#include "slam_frontend/optimized_graph_tracker.hpp"

namespace slam {

OptimizedGraphTracker::OptimizedGraphTracker(std::shared_ptr<SlamNode> node_handle) : node_handle_(node_handle) {}

OptimizedGraphTracker::~OptimizedGraphTracker() {}

std::vector<geometry_msgs::msg::Point> OptimizedGraphTracker::OdometryPoints() const { return odometry_points_; }

std::vector<geometry_msgs::msg::Point> OptimizedGraphTracker::LandmarkPoints() const { return landmark_points_; }

std::list<std::pair<int, int>> OptimizedGraphTracker::OdometryEdges() const { return odometry_edges_; }

std::list<std::pair<int, int>> OptimizedGraphTracker::LandmarkEdges() const { return landmark_edges_; }

std::list<std::pair<int, int>> OptimizedGraphTracker::Factors() const { 
  std::list<std::pair<int, int>> factors;
  factors.insert(factors.end(), odometry_edges_.begin(), odometry_edges_.end());
  factors.insert(factors.end(), landmark_edges_.begin(), landmark_edges_.end());
  return factors;
}

} // end namespace slam