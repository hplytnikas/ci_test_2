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

#include "slam_common/slam_node.hpp"

namespace slam {

namespace {
// Frames
constexpr const char *kMapFrame = "map";
constexpr const char *kOdomFrame = "odom";
constexpr const char *kBaseLinkFrame = "base_link";
// Params
constexpr const char *kOnlineMapParam = "topics.pub.online_map";
constexpr const char *kAccelerationMapStatusParam = "topics.pub.acceleration_map_status";
constexpr const char *kConeCallbackParam = "topics.sub.cone_callback";
constexpr const char *kLapCounterCallbackParam = "topics.sub.lap_counter_callback";
constexpr const char *kVehicleMovingCallbackParam = "topics.sub.vehicle_moving_callback";
constexpr const char *kMissionFinishedCallbackParam = "topics.sub.mission_finished_callback";
constexpr const char *kGlobalMapVisualizationParam = "visualization_topics.global_map";
constexpr const char *kPosesVisualizationParam = "visualization_topics.poses";
constexpr const char *kRawGraphVisualizationParam = "visualization_topics.raw_graph";
constexpr const char *kAssociationsVisualizationParam = "visualization_topics.associations";
// Default topic
constexpr const char *kOnlineMapTopic = "/estimation/online_map";
constexpr const char *kAccelerationMapStatusTopic = "/estimation/acceleration_map_status";
constexpr const char *kConeCallbackTopic = "/perception/cone_array";
constexpr const char *kLapCounterCallbackTopic = "/lap_count";
constexpr const char *kVehicleMovingCallbackTopic = "/moving";
constexpr const char *kMissionFinishedCallbackTopic = "/vcu_msgs/mission_finished";
constexpr const char *kGlobalMapVisualizationTopic = "/estimation/viz/optimized_global_map";
constexpr const char *kPosesVisualizationTopic = "/estimation/viz/optimized_poses";
constexpr const char *kRawGraphVisualizationTopic = "/estimation/viz/raw_graph";
constexpr const char *kAssociationsVisualizationTopic = "/estimation/viz/association";
} // namespace

SlamNode::SlamNode(const std::string &node_name, const std::string &node_namespace, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, node_namespace, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
      tf_broadcaster_(this), lap_count_(0), is_vehicle_moving_(false), has_mission_finished_(false) {
  InitSubscribers();
  InitPublishers();
}

SlamNode::~SlamNode() {}

void SlamNode::InitSubscribers() {
  std::string topic_name = this->GetParameter<std::string>(kLapCounterCallbackParam, kLapCounterCallbackTopic);
  const int queue_size = 1;
  lap_counter_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      topic_name, queue_size, std::bind(&SlamNode::LapCounterCallback, this, std::placeholders::_1));

  topic_name = this->GetParameter<std::string>(kVehicleMovingCallbackParam, kVehicleMovingCallbackTopic);
  vehicle_moving_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_name, queue_size, std::bind(&SlamNode::VehicleMovingCallback, this, std::placeholders::_1));

  topic_name = this->GetParameter<std::string>(kMissionFinishedCallbackParam, kMissionFinishedCallbackTopic);
  mission_finished_subscriber_ = this->create_subscription<autonomous_msgs::msg::BoolStamped>(
      topic_name, queue_size, std::bind(&SlamNode::MissionFinishedCallback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(this->get_logger(), "SlamNode: Subscribers initialized.");
}

void SlamNode::InitPublishers() {
  const size_t queue_size = 1;
  std::string topic_name = this->GetParameter<std::string>(kOnlineMapParam, kOnlineMapTopic);
  this->online_map_publisher_ = this->create_publisher<autonomous_msgs::msg::ConeArray>(topic_name, queue_size);

  topic_name = this->GetParameter<std::string>(kAccelerationMapStatusParam, kAccelerationMapStatusTopic);
  this->acceleration_map_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(topic_name, queue_size);

  topic_name = this->GetParameter<std::string>(kGlobalMapVisualizationParam, kGlobalMapVisualizationTopic);
  this->global_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, queue_size);

  topic_name = this->GetParameter<std::string>(kPosesVisualizationParam, kPosesVisualizationTopic);
  this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, queue_size);

  topic_name = this->GetParameter<std::string>(kRawGraphVisualizationParam, kRawGraphVisualizationTopic);
  this->raw_graph_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, queue_size);

  topic_name = this->GetParameter<std::string>(kAssociationsVisualizationParam, kAssociationsVisualizationTopic);
  this->association_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, queue_size);

  RCLCPP_INFO_STREAM(this->get_logger(), "SlamNode: Publishers initialized.");
}

// Create subscribers
void SlamNode::CreateConeCallbackSubscriber(std::function<void(const autonomous_msgs::msg::ConeArray)> callback) {
  const std::string topic_name = this->GetParameter<std::string>(kConeCallbackParam, kConeCallbackTopic);
  const size_t queue_size = 1;

  this->cone_callback_subscriber_ =
      this->create_subscription<autonomous_msgs::msg::ConeArray>(topic_name, queue_size, callback);
  RCLCPP_INFO_STREAM(this->get_logger(), "SlamNode: ConeCallback Subscriber initialized.");
}

void SlamNode::LapCounterCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  
}

void SlamNode::VehicleMovingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  
}

void SlamNode::MissionFinishedCallback(const autonomous_msgs::msg::BoolStamped::SharedPtr msg) {

}

// Publisher methods
void SlamNode::PublishOnlineMap(const autonomous_msgs::msg::ConeArray &online_map) const {

}

void SlamNode::PublishAccelerationMapStatus(bool acceleration_map_generated) const {

}

void SlamNode::PublishGlobalMapVisualization(const visualization_msgs::msg::MarkerArray &global_map_markers) const {

}

void SlamNode::PublishPosesVisualization(const geometry_msgs::msg::PoseArray &pose_array) const {

}

void SlamNode::PublishRawGraphVisualization(const visualization_msgs::msg::MarkerArray &raw_graph_marker) const {

}

void SlamNode::PublishAssociationsVisualization(const visualization_msgs::msg::MarkerArray &association_marker) const {

}

// Localization
void SlamNode::BroadcastInitialPose(const rclcpp::Time timestamp) {

}

void SlamNode::BroadcastEstimatedSlamPose(const geometry_msgs::msg::PoseStamped &slam_pose, const geometry_msgs::msg::PoseStamped &odom_to_base_link) {
  EASY_FUNCTION(profiler::colors::Red);

}

std::pair<bool, geometry_msgs::msg::PoseStamped> SlamNode::FetchTF(const rclcpp::Time &timestamp, const std::string &from_frame,
                                               const std::string &to_frame) const {
  EASY_FUNCTION(profiler::colors::Cyan); // Time this function
  return std::make_pair(false, utils::ZeroPose());
}

} // end namespace slam
