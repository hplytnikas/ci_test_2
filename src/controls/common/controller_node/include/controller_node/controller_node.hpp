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

 #include <Eigen/Dense>
 #include <easy/arbitrary_value.h>
 #include <easy/profiler.h>
 #include <memory>
 #include <string>
 #include <variant>
 #include <vector>
 
 #include "rclcpp/rclcpp.hpp"
 #include "tf2/exceptions.h"
 #include "tf2/utils.h"
 #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
 #include "tf2_ros/buffer.h"
 #include "tf2_ros/transform_listener.h"
 
 #include "autonomous_msgs/msg/bool_stamped.hpp"
 #include "autonomous_msgs/msg/boundary.hpp"
 #include "autonomous_msgs/msg/double_stamped.hpp"
 
 #include "geometry_msgs/msg/point32.hpp"
 #include "geometry_msgs/msg/transform_stamped.hpp"
 
 #include "vcu_msgs/msg/car_command.hpp"
 #include "vcu_msgs/msg/four_wheel_car_command.hpp"
 #include "vcu_msgs/msg/res_state.hpp"
 #include "vcu_msgs/msg/velocity_estimation.hpp"
 
 #include "control_msgs/msg/controller_ref.hpp"
 #include "control_msgs/msg/reference_state.hpp"
  
 /*******************************************************************************
  * class ControllerNode                                                     
  ******************************************************************************/
 /**
  * @brief ROS node that publishes the control command to the car
  *
  * ```
  * Subscribes to:
  * - velocity_estimation_topic:   /vcu_msgs/velocity_estimation
  * - steering_topic:              /vcu_msgs/steering_feedback
  * - reference_topic:             /planning/reference
  * - res_state_topic:             /vcu_msgs/res_state
  *
  * Publishes to:
  * - car_command_topic:            /control/car_command
  * - four_wheel_car_command_topic: /control/four_wheel_car_command
  * ```
  * @param velocity_estimation_topic Topic name for velocity feedback
  * @param steering_topic            Topic name for steering angle feedback
  * @param reference_topic           Topic name for the reference trajectory
  * @param car_command_topic         Topic name for the car command
  * @param four_wheel_car_command_topic Topic name for the four wheel car command
  * @param res_state_topic           Topic name for the res state
  * @param controller_frequency      Frequency at which the controller runs
  * @param controller_timeout_reset  Number of controller steps before resetting the controller
  * @param controller_timeout_fatal  Number of controller steps before stopping the controller
  * @param braking_acceleration      Braking acceleration
  * @param has_res_go_been_pressed   Flag to check if the res go button has been pressed
  * 
  * 
  * @throws ControllerStepException Thrown, if the controller step fails
  * 
  */ 
 class ControllerNode : public rclcpp::Node {
 public:
   // Default destructor
   ~ControllerNode() {}
   /**
    * @brief Construct a new ControllerNode object
    * 
    * @param[in] node_name The name of the node
    * @param[in] controller The primary controller to be used
    * @param[in] backup_controller The backup controller to be used in case the primary controller fails
    */
   explicit ControllerNode(const std::string node_name);

   /**
    * @brief This exception is thrown when the controller step fails
    * The node will catch this exception and not publish the car command
    * The error policy is that if there the controller has failed for longer than
    * ```
    * controller_timeout_reset_ The controller will be reset
    * controller_timeout_fatal_ The controller will be stopped
    * ```
    */
   class ControllerStepException : public std::runtime_error {
   public:
     explicit ControllerStepException(const std::string &what_arg) : std::runtime_error(what_arg) {}
   };
 
 private:
   // Topic names
 
   std::string velocity_estimation_topic_;
   std::string steering_topic_;
   std::string reference_topic_;
   std::string res_state_topic_;
 
   std::string car_command_topic_;
   std::string four_wheel_car_command_topic_;
 
   // Others
 
   double controller_frequency_;
   unsigned int controller_timeout_reset_;
   unsigned int controller_timeout_fatal_;
   double braking_acceleration_;
   bool has_res_go_been_pressed_;
 
   // PUBLISHERS
 
   rclcpp::Publisher<vcu_msgs::msg::CarCommand>::SharedPtr car_command_publisher_;
   rclcpp::Publisher<vcu_msgs::msg::FourWheelCarCommand>::SharedPtr four_wheel_car_command_publisher_;
 
   // SUBSCRIBERS
 
   rclcpp::Subscription<vcu_msgs::msg::VelocityEstimation>::SharedPtr velocity_estimation_subscriber_;
   rclcpp::Subscription<autonomous_msgs::msg::DoubleStamped>::SharedPtr steering_subscriber_;
   rclcpp::Subscription<control_msgs::msg::ControllerRef>::SharedPtr reference_subscriber_;
   rclcpp::Subscription<vcu_msgs::msg::ResState>::SharedPtr res_state_subscriber_;
 
   // Callback functions
 
   void CallbackVelocityEstimation(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg);
   void CallbackSteering(const autonomous_msgs::msg::DoubleStamped::SharedPtr msg);
   void CallbackReference(const control_msgs::msg::ControllerRef::SharedPtr msg);
   void CallbackResState(const vcu_msgs::msg::ResState::SharedPtr msg);
 
   // TF2
 
   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
   std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
 
   // TIMERS
   // Timer to run the controller step at specified frequency
   rclcpp::TimerBase::SharedPtr timer_;
 
   // Controller step, called by the timer at the specified frequency
   void RunControllerStep();
 
   // LAST MESSAGES BUFFER
 
   vcu_msgs::msg::VelocityEstimation latest_velocity_estimation_msg_;
   autonomous_msgs::msg::DoubleStamped latest_steering_msg_;
   control_msgs::msg::ControllerRef latest_reference_msg_;
  
   bool latest_velocity_estimation_msg_received_ = false;
   bool latest_steering_msg_received_ = false;
   bool latest_reference_msg_received_ = false;
 
   // Number of consectutive controller step failures
   // Used to check if the controller has failed for too long
   unsigned int controller_step_failures_ = 0;
 
   // Has the controller been braked / commanded to stop
   bool controller_braking_ = false;

 };