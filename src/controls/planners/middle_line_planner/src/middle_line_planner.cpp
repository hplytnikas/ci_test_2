/*******************************************************************************
 * AMZ Driverless Project                                                      *
 * Copyright (c) 2024-2025                                                     *
 * Authors:                                                                    *
 *   - Nazim Ozan Yasar <nyasar@ethz.ch>                                       *
 *   - Mattia Mangili <mangilim@ethz.ch>                                       *
 *   - Kevin Gabriel <kegabriel@ethz.ch>                                       *
 *   - Vincent Rasse <vrasse@ethz.ch>                                          *
 *   - Audrey Kubler <akubler@ethz.ch>                                         *
 *   - Sinan Laloui <slaloui@ethz.ch>                                          *
 *   - Alexander Terrail <aterrail@ethz.ch>                                    *
 *                                                                             *
 * All rights reserved.                                                        *
 *                                                                             *
 * Unauthorized copying of this file, via any medium, is strictly prohibited.  *
 * Proprietary and confidential.                                               *
 ******************************************************************************/

#include "middle_line_planner/middle_line_planner.hpp"

MiddleLinePlanner::MiddleLinePlanner() : Node("middle_line_planner_node") {

  // Define parameters
  this->declare_parameter("bounded_path_topic", "/estimation/bounded_path");
  this->declare_parameter("reference_topic", "/planning/reference");
  this->declare_parameter("vx_ref", 5.0);

  // Load parameters
  bounded_path_topic_ = this->get_parameter("bounded_path_topic").as_string();
  reference_topic_ = this->get_parameter("reference_topic").as_string();
  vx_ref_ = this->get_parameter("vx_ref").as_double();

  subscription_ = this->create_subscription<autonomous_msgs::msg::Boundary>(
      bounded_path_topic_, 1, std::bind(&MiddleLinePlanner::boundary_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<control_msgs::msg::ControllerRef>(reference_topic_, 1);
};

void MiddleLinePlanner::boundary_callback(const autonomous_msgs::msg::Boundary::SharedPtr msg) {
  
  control_msgs::msg::ControllerRef reference_msg;
  reference_msg.header.frame_id = msg->header.frame_id;
  reference_msg.header.stamp = msg->header.stamp;

  auto middle_line = msg->middle_line;
  for (auto it = middle_line.begin(); it != middle_line.end(); it++) {
    control_msgs::msg::ReferenceState ref_state;
    ref_state.position.x = it->position.x;
    ref_state.position.y = it->position.y;
    ref_state.boundary_left = 1.5;
    ref_state.boundary_right = 1.5;
    ref_state.vx_ref = vx_ref_;
    reference_msg.reference_trajectory.push_back(ref_state);
    RCLCPP_INFO(this->get_logger(), "Added reference state at (%.2f, %.2f)", ref_state.position.x,
                ref_state.position.y);
  }
  publisher_->publish(reference_msg);
};

