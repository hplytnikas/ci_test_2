# Lap Counter

## Explanation(s):
This node uses the map -> baselink transforms, to compute the number of laps completed by the car. Publishes the laps completed,
the time taken per lap, the distance travelled inbetween poses measurements and if the car is currently in motion.

## Subscriptions:
map -> base_link

## Publications:
/lap_count          - std_msgs::msg::Int32    - Publication triggered by end of lap
/lap_time           - std_msgs::msg::Float32  - Publication triggered by end of lap
/distance_travelled - std_msgs::msg::Float32  - Published at 200Hz
/moving             - std_msgs::msg::Bool     - Published at 4Hz

## Visualization(s):
The circle marker can be visualized in RViz / Foxglove.