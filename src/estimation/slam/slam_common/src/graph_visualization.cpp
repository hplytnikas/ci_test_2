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

#include "slam_common/graph_visualization.hpp"

namespace slam {

GraphVisualization::GraphVisualization() {}

GraphVisualization::~GraphVisualization() {}

void GraphVisualization::AddPose(double x, double y) {
  return;
}

void GraphVisualization::AddLandmark(size_t landmark_id, double x, double y) {
  return;
}

void GraphVisualization::PopOdometryEdge() { odometry_edges_.pop_front(); }

void GraphVisualization::PopLandmarkEdge() { landmark_edges_.pop_front(); }

std::vector<geometry_msgs::msg::Point> GraphVisualization::OdometryPoints() const { return odometry_points_; }

std::vector<geometry_msgs::msg::Point> GraphVisualization::LandmarkPoints() const { return landmark_points_; }

std::list<std::pair<int, int>> GraphVisualization::OdometryEdges() const { return odometry_edges_; }

std::list<std::pair<int, int>> GraphVisualization::LandmarkEdges() const { return landmark_edges_; }

} // end namespace slam
