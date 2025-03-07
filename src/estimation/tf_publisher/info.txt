# TF Publisher

## Explanation(s):
The code subscribes to the `/vcu_msgs/velocity_estimation` topic to receive velocity data, which represents the output of the Kalman Filter algorithm computed within the VCU. By integrating these velocity estimates, the publisher is able to continuously update and publish the current transformation between the `odom` frame and the `base_link` frame, which represents the car's current position and orientation.

## Subscriptions:
**/vcu_msgs/velocity_estimation** - VelocityEstimation.msg 
std_msgs/Header header
geometry_msgs/Pose2D vel 
geometry_msgs/Pose2D acc 
autonomous_msgs/CovariancePose vel_cov

## Publications:
tf - geometry_msgs/msg/TransformStamped.msg - Published at 200Hz(?)

## Visualization(s):
To visualize the VE integrated pose go to the config/default.yaml and set the mode to 'debug'. THe following topics will be published: `/tf_publisher/pose` and `/tf_publisher/pose_array`, and can be visualized using software as RVIZ or FoxGlove
