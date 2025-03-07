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
#include "slam_common/visualizer.hpp"

namespace slam {

Visualizer::Visualizer(std::shared_ptr<SlamNode> node_handle) : node_handle_(node_handle) { LoadParameters(); }

Visualizer::~Visualizer() {}

void Visualizer::LoadParameters() {
  visualization_enabled_ = node_handle_->GetParameter<bool>("visualization_enabled", false);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "Visualizer: Initialized with visualization enabled:" << visualization_enabled_);
}

bool Visualizer::IsVisualizationEnabled() const { return visualization_enabled_; }

void Visualizer::VisualizeGlobalMap(const ConeMap &cone_map, const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  return;
}

void Visualizer::VisualizePoses(const std::vector<geometry_msgs::msg::PoseStamped> &poses, const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  return;
}

void Visualizer::VisualizeAssociations(const ConeMap &observations, const ConeMap &global_map,
                                       const std::vector<Association> &associations,
                                       const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  return;
}

void Visualizer::VisualizeRawGraph(const std::shared_ptr<GraphVisualization> graph,
                                   const rclcpp::Time &timestamp) const {
  EASY_FUNCTION(profiler::colors::Blue); // Time this function
  return;
}

visualization_msgs::msg::Marker Visualizer::CreateMarker(const rclcpp::Time &timestamp, std::string ns, int32_t type,
                                                         int32_t action) const {
  visualization_msgs::msg::Marker marker;
  
  return marker;
}

} // end namespace slam
