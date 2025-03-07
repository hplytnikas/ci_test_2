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

#pragma once
#include "autonomous_msgs/msg/boundary.hpp"
#include "control_msgs/msg/controller_ref.hpp"
#include "control_msgs/msg/reference_state.hpp"
#include "rclcpp/rclcpp.hpp"

/*******************************************************************************
  * class MiddleLinePlanner                                                     
  ******************************************************************************/
 /**
  * @brief ROS node that publishes the middle line as the reference trajectory for the controller
  *
  * ```
  * Subscribes to:
  * - bounded_path_topic:          /vcu_msgs/velocity_estimation
  *
  * Publishes to:
  * - reference_topic:             /planning/reference
  * ```
  * @param bounded_path_topic        Topic name for velocity feedback
  * @param reference_topic           Topic name for the reference trajectory
  * @param vx_ref                    Reference velocity(assumed constant)
  * 
  */ 
class MiddleLinePlanner: public rclcpp::Node{
  public:
    // Default destructor
    ~MiddleLinePlanner() {};
    //Construct a new MiddleLinePlanner object
    explicit MiddleLinePlanner();
  private:
    // Topic names
    std::string bounded_path_topic_;
    std::string reference_topic_;
    // Reference velocity
    double vx_ref_;
    // SUBSCRIBERS
    rclcpp::Subscription<autonomous_msgs::msg::Boundary>::SharedPtr subscription_;
    // PUBLISHERS
    rclcpp::Publisher<control_msgs::msg::ControllerRef>::SharedPtr publisher_;
    // Callback functions
    void boundary_callback(const autonomous_msgs::msg::Boundary::SharedPtr msg);
};

