/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2025 Author:
 *   - Fred Defokue   <fdefokue@ethz.ch>
 *   
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include "hesai_ros_driver/hesai_ros_driver_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <cstring> // For std::memcpy

DummyHesaiDriver::DummyHesaiDriver()
  : Node("dummy_hesai_ros_driver")
{
  // Declare parameters mimicking a typical configuration
  this->declare_parameter<bool>("subscribe_packets", false);
  this->declare_parameter<bool>("publish_packets", true);
  this->declare_parameter<bool>("publish_pointcloud", true);
  this->declare_parameter<std::string>("frame_id", "hesai_lidar");
  this->declare_parameter<std::string>("packet_topic", "/lidar_packets");
  this->declare_parameter<std::string>("pointcloud_topic", "/lidar_points");

  // Read parameter values
  bool subscribe_packets = this->get_parameter("subscribe_packets").as_bool();
  bool publish_packets = this->get_parameter("publish_packets").as_bool();
  bool publish_pointcloud = this->get_parameter("publish_pointcloud").as_bool();
  std::string packet_topic = this->get_parameter("packet_topic").as_string();
  std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();

  // Conditional subscription to raw packets
  if (subscribe_packets) {
    RCLCPP_INFO(this->get_logger(), "Subscribing to raw packets on topic: %s", packet_topic.c_str());
    raw_packet_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      packet_topic,
      10,
      std::bind(&DummyHesaiDriver::packetCallback, this, std::placeholders::_1)
    );
  }

  // Conditional publisher for raw packets
  if (publish_packets) {
    RCLCPP_INFO(this->get_logger(), "Publishing raw packets on topic: %s", packet_topic.c_str());
    raw_packet_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(packet_topic, 10);
  }

  // Conditional publisher for point clouds
  if (publish_pointcloud) {
    RCLCPP_INFO(this->get_logger(), "Publishing point clouds on topic: %s", pointcloud_topic.c_str());
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
  }

  // Create a timer to publish dummy data at 10 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DummyHesaiDriver::timerCallback, this)
  );
}

void DummyHesaiDriver::packetCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  // In a real driver, you would parse raw packets and convert them to a point cloud.
  RCLCPP_INFO(this->get_logger(), "Received a dummy raw packet with size %zu bytes", msg->data.size());
}

void DummyHesaiDriver::timerCallback()
{
  // 1) Optionally publish a dummy raw packet
  if (raw_packet_pub_) {
    auto packet_msg = std_msgs::msg::UInt8MultiArray();
    packet_msg.data = {0x00, 0x01, 0x02}; // Dummy data
    raw_packet_pub_->publish(packet_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published dummy raw packet");
  }

  // 2) Optionally publish a dummy point cloud
  if (pointcloud_pub_) {
    auto pc_msg = sensor_msgs::msg::PointCloud2();
    pc_msg.header.stamp = this->now();
    pc_msg.header.frame_id = frame_id_;

    // Minimal fields for a dummy point cloud: 1 point
    pc_msg.height = 1;
    pc_msg.width = 1;
    pc_msg.fields.resize(3);
    pc_msg.fields[0].name = "x";
    pc_msg.fields[1].name = "y";
    pc_msg.fields[2].name = "z";
    pc_msg.point_step = 12;  // 3 floats (x, y, z) * 4 bytes each
    pc_msg.row_step = pc_msg.point_step * pc_msg.width;
    pc_msg.is_bigendian = false;
    pc_msg.is_dense = false;
    pc_msg.data.resize(pc_msg.row_step * pc_msg.height, 0u);

    // Example: one point at (1, 1, 1)
    float x = 1.0f, y = 1.0f, z = 1.0f;
    std::memcpy(&pc_msg.data[0], &x, sizeof(float));
    std::memcpy(&pc_msg.data[4], &y, sizeof(float));
    std::memcpy(&pc_msg.data[8], &z, sizeof(float));

    pointcloud_pub_->publish(pc_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published dummy point cloud with 1 point");
  }
}
