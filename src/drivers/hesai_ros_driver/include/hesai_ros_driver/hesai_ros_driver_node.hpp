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

#ifndef DUMMY_HESAI_DRIVER_HPP_
#define DUMMY_HESAI_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstring>  // For std::memcpy

class DummyHesaiDriver : public rclcpp::Node
{
public:
  DummyHesaiDriver();

private:
  // Callback for incoming raw packet messages
  void packetCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  // Timer callback to publish dummy data periodically
  void timerCallback();

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_packet_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_packet_sub_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // Frame ID for point cloud messages
  std::string frame_id_;
};

#endif // DUMMY_HESAI_DRIVER_HPP_
