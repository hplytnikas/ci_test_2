/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024 Authors:
 *   - Tristan Gabl <trgabl@student.ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */


#pragma once

#include "visualization_msgs/msg/marker_array.hpp"
#include <autonomous_msgs/msg/cone.hpp>
#include <autonomous_msgs/msg/cone_array.hpp>
#include <autonomous_msgs/msg/cone_type_prob.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <perception_msgs/msg/pipeline_runtime.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <easy/arbitrary_value.h>
#include <easy/profiler.h>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <vector>

namespace cone_fusion {
// shorthands for message_filters synchronization policy (we use approximate
// time policy) types
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<autonomous_msgs::msg::ConeArray, autonomous_msgs::msg::ConeArray>;
using Sync = message_filters::Synchronizer<SyncPolicy>;

enum Pipeline { camera_only = 1, lidar_only = 2, sensor_fusion = 4 };

class ConeFusion : public rclcpp::Node {

public:
    ConeFusion();

private:

    // Parameters
    std::shared_ptr<Sync> sync_;

    Pipeline active_pipeline_;

    // Topic Names
    std::string topic_camera_cone_array_;   
    std::string topic_lidar_cone_array_;    
    std::string topic_sf_cone_array_;           
    std::string topic_output_cone_array_;   
    std::string topic_output_cone_markers_; 
    std::string topic_runtime_;             

    // Subscribers
    rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr sub_camera_cone_array_; 
    rclcpp::Subscription<autonomous_msgs::msg::ConeArray>::SharedPtr sub_lidar_cone_array_;  
    message_filters::Subscriber<autonomous_msgs::msg::ConeArray> sub_lidar_cone_array2_;     
    message_filters::Subscriber<autonomous_msgs::msg::ConeArray> sub_sf_cone_array_;         

    // Publishers
    rclcpp::Publisher<perception_msgs::msg::PipelineRuntime>::SharedPtr pub_runtime_;       
    rclcpp::Publisher<autonomous_msgs::msg::ConeArray>::SharedPtr pub_output_cone_array_;   
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cone_markers_;   

    /**
     * Loads ROS parameters
     */
    void LoadParameters();

    /**
     * Creates subscribers for the topics that the node needs to listen to.
     */
    void SubscribeTopics();

    /**
     * Creates publishers for the topics that the node needs to publish.
     */
    void AdvertiseTopics();

    /**
     * Callback function for the camera cone array subscriber
     * @param camera_cone_msg The message containing the cones detected by the
     * camera.
     */
    void PassCameraCones(const autonomous_msgs::msg::ConeArray::SharedPtr camera_cone_msg);

    /**
     * Callback function for the LiDAR cone array subscriber
     * @param lidar_cones The message containing the cones detected by the LiDAR.
     */
    void PassLiDARCones(const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cones);

    /**
     * Concatenates the cones detected by the camera and the LiDAR.
     * @param sf_cone_msg The cones detected by the camera.
     * @param lidar_cone_msg The cones detected by the LiDAR.
     */

    /**
     * Concatenates the cones detected by the camera and the LiDAR.
     * @param sf_cone_msg The cones detected by the camera.
     * @param lidar_cone_msg The cones detected by the LiDAR.
     */
    void ConcatenateLiDARtoFusion(const autonomous_msgs::msg::ConeArray::SharedPtr sf_cone_msg,
                                    const autonomous_msgs::msg::ConeArray::SharedPtr lidar_cone_msg);

    /**
     *  
     * @param all_cones
     */
    void CommonPostProcessor(const autonomous_msgs::msg::ConeArray &all_cones);

    /**
     * Publishes the cone markers for the visualization of the cones in RViz.
     * @param cone_array The array of cones to be visualized.
     */
    void PublishConeMarkers(const autonomous_msgs::msg::ConeArray &cone_array);

    /**
     * Publishes the pipeline runtime.
     * @param start_time Timestamp when the image/PCL was received.
     * @param end_time The time when the perception pipeline ended.
     */
    void PublishRuntime(const rclcpp::Time &start_time, const rclcpp::Time &end_time);

    autonomous_msgs::msg::ConeArray dummy_cones() {
        // begin ---- dummy cone_array
        autonomous_msgs::msg::ConeArray dummy_cone_array;
        dummy_cone_array.header.frame_id = "";
        dummy_cone_array.header.stamp = this->now();

        autonomous_msgs::msg::Cone dummy_cone_1;
        dummy_cone_1.position.x = 1.0;
        dummy_cone_1.position.y = 2.0;
        dummy_cone_1.position.z = 0.0;
        dummy_cone_1.prob_type.blue = 1.0;
        dummy_cone_1.prob_type.yellow = 0.0;
        dummy_cone_1.prob_type.orange = 0.0;
        dummy_cone_1.prob_type.orange_big = 0.0;
        dummy_cone_1.pipeline = 2;

        autonomous_msgs::msg::Cone dummy_cone_2;
        dummy_cone_2.position.x = 2.0;
        dummy_cone_2.position.y = 1.0;
        dummy_cone_2.position.z = 0.0;
        dummy_cone_2.prob_type.blue = 0.0;
        dummy_cone_2.prob_type.yellow = 1.0;
        dummy_cone_2.prob_type.orange = 0.0;
        dummy_cone_2.prob_type.orange_big = 0.0;
        dummy_cone_2.pipeline = 2;
        // end ---- dummy cone_array

        dummy_cone_array.cones.push_back(dummy_cone_1);
        dummy_cone_array.cones.push_back(dummy_cone_2);

        return dummy_cone_array;
    }
};

} 