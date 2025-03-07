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
#include "slam_frontend/map_loader.hpp"

namespace slam {

MapLoader::MapLoader(std::shared_ptr<SlamNode> node_handle) : node_handle_(node_handle) { LoadParameters(); }

MapLoader::~MapLoader() {}

void MapLoader::LoadParameters() {
  int mission_type = node_handle_->GetParameter<int>("mission_type", static_cast<int>(MissionType::kAutocross));
  std::string mission_str =
      std::array<std::string, 4>{"acceleration", "skidpad", "autocross", "trackdrive"}[mission_type];

  map_save_file_path_ = node_handle_->GetParameter<std::string>("map_loader.map.save_file_path",
                                                                "src/estimation/slam/slam_frontend/maps/");
  map_save_file_path_ = map_save_file_path_ + mission_str + "_" + utils::GetCurrentTimeString() +
                        ".csv"; // Map wll have same time as rosbag

  // TODO(Christoforos) If I had a suggestion to make, I would have this code automatically select the latest map
  // that was generated unless otherwise specified. One way to do that is to have a default map.csv symlink that's
  // overwritten every time the map is saved. Otherwise you'll need to manually set the map title every time/lose old
  // maps Load path will be different in each discipline param file
  map_load_file_path_ = node_handle_->GetParameter<std::string>("map_loader.map.load_file_path",
                                                                "src/estimation/slam/slam_frontend/maps/skidpad.csv");

  trajectory_save_file_path_ = node_handle_->GetParameter<std::string>("map_loader.trajectory.save_file_path",
                                                                       "src/estimation/slam/slam_frontend/maps/");
  trajectory_save_file_path_ = trajectory_save_file_path_ + "trajectory_" + utils::GetCurrentTimeString() + ".txt";

  // Acceleration Parameters
  track_first_part_length_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.track_first_part_length_m", 75.0);
  track_second_part_length_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.track_second_part_length_m", 75.0);
  num_boundary_cones_ = node_handle_->GetParameter<int>("map_loader.map.acceleration.num_boundary_cones", 14);
  num_boundary_orange_cones_ =
      node_handle_->GetParameter<int>("map_loader.map.acceleration.num_boundary_orange_cones", 15);
  max_track_width_y_m_ = node_handle_->GetParameter<double>("map_loader.map.acceleration.max_track_width_y_m", 6.0);
  estimated_first_cones_distance_x_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.estimated_first_cones_distance_x_m", 3.0);
  estimated_big_orange_cone_distance_x_m_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.estimated_big_orange_cone_distance_x_m", 1.0);
  min_cone_distance_x_m_ = node_handle_->GetParameter<double>("map_loader.map.acceleration.min_cone_distance_x_m", 2.0);
  max_cone_distance_x_m_ = node_handle_->GetParameter<double>("map_loader.map.acceleration.max_cone_distance_x_m", 5.5);
  perception_observation_weight_ =
      node_handle_->GetParameter<double>("map_loader.map.acceleration.perception_observation_weight", 0.1);
  min_num_detected_cones_ = node_handle_->GetParameter<int>("map_loader.map.acceleration.min_num_detected_cones", 2);

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Parameters loaded.");
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Map save path set to " << map_save_file_path_);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "MapLoader: Trajectory save path set to " << trajectory_save_file_path_);
}

ConeMap MapLoader::LoadMap() const {
  ConeMap input_map;
  
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Map loaded successfully from " << map_load_file_path_);
  return input_map;
}

ConeMap MapLoader::GenerateAccelerationMap(const ConeMap &cone_observations) const {
  ConeMap acceleration_map, negatives, positives;

  return acceleration_map;
}

void MapLoader::SortConesX(ConeMap &cones) const {
  std::sort(cones.begin(), cones.end(),
            [this](const autonomous_msgs::msg::Cone &a, const autonomous_msgs::msg::Cone &b) {
              return a.position.x < b.position.x;
            });
}

double MapLoader::FilterCones(const ConeMap &cone_observations, ConeMap &positives, ConeMap &negatives) const {
  double avg_negative_y = 0, avg_positive_y = 0;

  return avg_positive_y - avg_negative_y;
}

double MapLoader::AverageConeDistanceX(ConeMap &positives, ConeMap &negatives) const {
  // Default average
  return estimated_first_cones_distance_x_m_;
}

void MapLoader::GenerateAccelerationBoundaries(ConeMap &acceleration_map, double first_cone_position,
                                               double distance_x_m, double distance_y_m, int num_cones) const {
  return;
}

void MapLoader::GenerateAccelerationBigOrangeCones(ConeMap &acceleration_map, double big_orange_cones_middle_x_m,
                                                   double distance_y_m) const {

}

void MapLoader::SaveMap(const ConeMap &map_to_save) const {
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "MapLoader: Map saved successfully to " << map_save_file_path_);
}

void MapLoader::SaveTrajectory(const std::vector<geometry_msgs::msg::PoseStamped> &trajectory_to_save) const {
  RCLCPP_INFO_STREAM(node_handle_->get_logger(),
                     "MapLoader: Trajectory saved successfully to " << trajectory_save_file_path_);
}

} // end namespace slam
