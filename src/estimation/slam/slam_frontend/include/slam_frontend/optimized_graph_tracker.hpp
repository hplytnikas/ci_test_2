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

#pragma once

#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <easy/profiler.h>
#include <list>
#include <vector>

#include "slam_common/slam_node.hpp"
#include "slam_common/utils.hpp"

namespace slam {

/*
 * Class to track the optimized graph output from SLAM backend, for visualization and saving.
 */
class OptimizedGraphTracker {

public:
  /*
   * Constructor
   */
  explicit OptimizedGraphTracker(std::shared_ptr<SlamNode> node_handle);

  /*
   * Destructor.
   */
  ~OptimizedGraphTracker();

  void UpdateOptimizedGraph();

  // Returns a vector of all pose points
  std::vector<geometry_msgs::msg::Point> OdometryPoints() const;
  // Returns a vector of all landmark points
  std::vector<geometry_msgs::msg::Point> LandmarkPoints() const;
  // Returns a list of pose-pose edges
  std::list<std::pair<int, int>> OdometryEdges() const;
  // Returns a list of pose-landmark edges
  std::list<std::pair<int, int>> LandmarkEdges() const;
  // Returns a list of all factors in the graph
  std::list<std::pair<int, int>> Factors() const;
  // Returns the number of pose-pose edges
  inline size_t NumOdometryEdges() const { return odometry_edges_.size(); }
  // Returns the number of pose-landmark edges
  inline size_t NumLandmarkEdges() const { return landmark_edges_.size(); }

private:
  /*
   * Load parameters from the parameter server.
   */
  void LoadParameters();

  // Reference to slam node
  std::shared_ptr<SlamNode> node_handle_;

  // Book-keeping structures
  // Keeps track of pose points
  std::vector<geometry_msgs::msg::Point> odometry_points_;
  // Keeps track of landmark points
  std::vector<geometry_msgs::msg::Point> landmark_points_;
  // Keeps track of pose edges (previous to current)
  std::list<std::pair<int, int>> odometry_edges_;
  // Keeps track of pose to landmark edges (pose to landmark)
  std::list<std::pair<int, int>> landmark_edges_;
};

} // end namespace slam