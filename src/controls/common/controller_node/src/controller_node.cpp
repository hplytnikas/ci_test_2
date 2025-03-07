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

 #include "controller_node/controller_node.hpp"

  ControllerNode::ControllerNode(const std::string node_name) : Node(node_name) {
 
   // PARAMETERS
   // Topic names
   this->declare_parameter("velocity_estimation_topic", "/vcu_msgs/velocity_estimation");
   this->declare_parameter("steering_topic", "/vcu_msgs/steering_feedback");
   this->declare_parameter("reference_topic", "/planning/reference");
   this->declare_parameter("car_command_topic", "/control/car_command");
   this->declare_parameter("four_wheel_car_command_topic", "/control/four_wheel_car_command");
   this->declare_parameter("res_state_topic", "/vcu_msgs/res_state");
 
   // Others
   this->declare_parameter("controller_frequency", 40.0);
   this->declare_parameter("controller_timeout_reset", 100);
   this->declare_parameter("controller_timeout_fatal", 400);
   this->declare_parameter("braking_acceleration", 10.0);
   this->declare_parameter("profiling_enabled", false);
 
   // Load parameters
   velocity_estimation_topic_ = this->get_parameter("velocity_estimation_topic").as_string();
   steering_topic_ = this->get_parameter("steering_topic").as_string();
 
   reference_topic_ = this->get_parameter("reference_topic").as_string();
   res_state_topic_ = this->get_parameter("res_state_topic").as_string();
 
   car_command_topic_ = this->get_parameter("car_command_topic").as_string();
   four_wheel_car_command_topic_ = this->get_parameter("four_wheel_car_command_topic").as_string();
 
   controller_frequency_ = this->get_parameter("controller_frequency").as_double();
   braking_acceleration_ = this->get_parameter("braking_acceleration").as_double();
 
   controller_timeout_reset_ = this->get_parameter("controller_timeout_reset").as_int();
   controller_timeout_fatal_ = this->get_parameter("controller_timeout_fatal").as_int();
 
   if (controller_frequency_ <= 0) {
     throw std::runtime_error("Controller frequency must be positive");
   }
 
   // SUBSCRIBERS
   velocity_estimation_subscriber_ = this->create_subscription<vcu_msgs::msg::VelocityEstimation>(
       velocity_estimation_topic_, 1,
       std::bind(&ControllerNode::CallbackVelocityEstimation, this, std::placeholders::_1));
 
   steering_subscriber_ = this->create_subscription<autonomous_msgs::msg::DoubleStamped>(
       steering_topic_, 1, std::bind(&ControllerNode::CallbackSteering, this, std::placeholders::_1));
 
   reference_subscriber_ = this->create_subscription<control_msgs::msg::ControllerRef>(
       reference_topic_, 1, std::bind(&ControllerNode::CallbackReference, this, std::placeholders::_1));
 
   res_state_subscriber_ = this->create_subscription<vcu_msgs::msg::ResState>(
       res_state_topic_, 1, std::bind(&ControllerNode::CallbackResState, this, std::placeholders::_1));
 
   // PUBLISHERS
   // Car command from the controller can be either for bicycle model or four wheel model
   car_command_publisher_ = this->create_publisher<vcu_msgs::msg::CarCommand>(car_command_topic_, 1);
   four_wheel_car_command_publisher_ =
       this->create_publisher<vcu_msgs::msg::FourWheelCarCommand>(four_wheel_car_command_topic_, 1);
 
   // TF2
   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
   tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
 
   // TIMER
   timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / controller_frequency_),
                                    std::bind(&ControllerNode::RunControllerStep, this));
 
   // Initialize watchdog
   controller_step_failures_ = 0;
 
   // Other
   has_res_go_been_pressed_ = false;

  }

  void ControllerNode::CallbackVelocityEstimation(const vcu_msgs::msg::VelocityEstimation::SharedPtr msg) {
    latest_velocity_estimation_msg_ = *msg;
    latest_velocity_estimation_msg_received_ = true;
  }
  
  void ControllerNode::CallbackSteering(const autonomous_msgs::msg::DoubleStamped::SharedPtr msg) {
    latest_steering_msg_ = *msg;
    latest_steering_msg_received_ = true;
  }
  
  void ControllerNode::CallbackReference(const control_msgs::msg::ControllerRef::SharedPtr msg) {
    latest_reference_msg_ = *msg;
    latest_reference_msg_received_ = true;
  }
  
  void ControllerNode::CallbackResState(const vcu_msgs::msg::ResState::SharedPtr msg) {
    if (msg->push_button && !has_res_go_been_pressed_) {
      RCLCPP_INFO(this->get_logger(), "Res go has been pressed, starting controller.");
      has_res_go_been_pressed_ = true;
    }
  }

  void ControllerNode::RunControllerStep() {  
    // This function is called to update the control command at the specified frequency
    // Do not send control command if res go has not been pressed
    //if (!has_res_go_been_pressed_ && false) {
    //  return;
    //}

    // Car command from the controller can be either for bicycle model or four wheel model
    vcu_msgs::msg::CarCommand car_command;

    const double ax = 0.0;
    const double steering_angle = 0.0;
    const double yaw_rate = 0.0;

    car_command.a_x[0] = car_command.a_x[1] = car_command.a_x[2] = ax;
    car_command.steering_angle[0] = car_command.steering_angle[1] = car_command.steering_angle[2] = steering_angle;
    car_command.yaw_rate[0] = car_command.yaw_rate[1] = car_command.yaw_rate[2] = yaw_rate;

    car_command_publisher_->publish(car_command);
  }