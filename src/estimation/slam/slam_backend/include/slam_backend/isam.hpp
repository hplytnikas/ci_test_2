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

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <easy/profiler.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "slam_backend/graph.hpp"
#include "slam_backend/slam_backend.hpp"
#include "slam_common/aliases.hpp"
#include "slam_common/graph_visualization.hpp"
#include "slam_common/slam_node.hpp"
#include "slam_common/visualizer.hpp"

namespace slam {

/* 
  * Class for the slam backend algorithm ISAM.
  */
class ISAM : public SlamBackend {

public:
  /*
   * Constructor.
   */
  explicit ISAM(std::shared_ptr<SlamNode> node_handle, std::shared_ptr<Visualizer> visualizer);

  /*
   * Destructor.
   */
  ~ISAM();

  // Set the global map to a fixed map (Skidpad)
  void SetFixedMap(const ConeMap &fixed_map);

  // Enables online switch from mapping to localization mode
  void SwitchToLocalizationMode();

  // Update the backend with the current landmarks
  void Update(const geometry_msgs::msg::PoseStamped pose_difference, const std::vector<Landmark> &landmarks);

  // Optimize graph using GTSAM library and update estimates
  void Optimize();

  // Prune ISAM graph edges by cloning the nonlinear factor graph and removing old edges
  void PruneGraph();

private:
  // Maximum number of edges in the graph
  size_t max_edges_;

  // Maximum number of edges in the graph after pruning
  size_t max_edges_after_pruning_;

  // GTSAM graph symbols
  unsigned char pose_symbol_;
  unsigned char landmark_symbol_;

  // Keep track of the latest odometry symbol
  gtsam::Symbol latest_pose_symbol_;

  // A noise model for odometry measurements
  Diagonal::shared_ptr pose_noise_;

  // A noise model for the odometry prior
  Diagonal::shared_ptr pose_prior_noise_;

  // A noise model for landmark prior (skidpad)
  Diagonal::shared_ptr landmark_prior_noise_;

  // A noise model for landmarks
  Diagonal::shared_ptr landmark_noise_;

  // iSAM pose estimates
  gtsam::Values initial_estimate_;

  // iSAM Optimized estimates
  gtsam::Values optimized_estimate_;

  // iSAM New estimates after pruning
  gtsam::Values new_estimate_;

  // Non-linear factor graph for holding latest data
  gtsam::NonlinearFactorGraph graph_;
  gtsam::NonlinearFactorGraph pruned_graph_with_nulls_;
  
  // Timestamp for previous iteration
  rclcpp::Time previous_timestamp_;

  // Flag to enable pruning
  bool enable_pruning_;

  // Relinerization interval
  int relinearization_interval_;

  // Bookkeeping
  // Keeps track of pose keys and the pose in order
  std::list<std::pair<gtsam::Key, geometry_msgs::msg::PoseStamped>> odometry_key_pose_queue_;
  // Keeps track of all factor indexes of factors connected to each pose
  std::unordered_map<gtsam::Key, std::set<size_t>> odometry_key_to_factor_indexes_;
  // Mapping between factor indexes and their odometry keys (nodes of each edge)
  std::unordered_map<size_t, std::set<gtsam::Key>> factor_index_to_keys_;
  // Keeps track of how many times each landmark is connected somewhere
  std::unordered_map<gtsam::Key, size_t> landmark_key_counts_;
  // Keeps track of the number of poses for symbol creation
  size_t pose_node_count_ = 0;
  // Number of edges between poses
  size_t num_pose_edges_ = 0;
  // Number of pose to landmark edges
  size_t num_landmark_edges_ = 0;

  // Visualization graph
  std::shared_ptr<GraphVisualization> graph_visualization_;

  // Nonlinear ISAM object
  std::unique_ptr<gtsam::NonlinearISAM> isam_;

  // Initialize the ISAM algorithm.
  void Init();

  // Initialize the ISAM graph.
  void InitGraph();

  // Adds the first pose to the graph
  void AddPosePrior(const geometry_msgs::msg::PoseStamped &prior_pose);

  // Adds a new pose to the graph
  void AddPose(const geometry_msgs::msg::PoseStamped &pose_difference);

  // Add priors of landmark locations from global_map_estimate_ to graph
  void AddLandmarkPriors();

  // Adds a landmark to the graph
  void AddLandmark(const Landmark &landmark);

  // Updates optimized solution to the latest pose estimates
  void UpdatePoseEstimates(const gtsam::Values &optimization_result);

  // Updates optimized solution to the latest map estimates
  void UpdateMapEstimate();

  // Creates a GTSAM symbol for the poses
  Symbol MakePoseSymbol(uint64_t pose_id);

  // Creates a GTSAM symbol for the landmarks
  Symbol MakeLandmarkSymbol(uint64_t landmark_id);

  // Debugging methods
  // Print the symbols of pose and landmark estimates
  void PrintEstimateSymbols();

  // Print the factors in the nonlinearfactorgraph
  void PrintFactorGraph();
}; // end class ISAM

} // end namespace slam