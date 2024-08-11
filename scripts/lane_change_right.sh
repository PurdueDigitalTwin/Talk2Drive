#!/bin/bash

# Sample bash script to change to the left lane in the highway scenario
# Autoware.AI is required to be installed and sourced

# $1: velocity
# $2: lookahead_distance
echo $1 $2

# Restart waypoint following module
rosnode kill /waypoint_loader /waypoint_marker_publisher /waypoint_replanner
timeout 1s rostopic pub /config/waypoint_follower autoware_config_msgs/ConfigWaypointFollower "{\"param_flag\": 1, \"velocity\": $1, \"lookahead_distance\": $2, \"lookahead_ratio\": 2.0}"
roslaunch waypoint_maker waypoint_loader.launch load_csv:=True multi_lane_csv:=/home/autoware/runway_test/runway_test_right_lane1.csv replanning_mode:=False realtime_tuning_mode:=False resample_mode:=True resample_interval:=1 replan_curve_mode:=False overwrite_vmax_mode:=False replan_endpoint_mode:=True velocity_max:=20 radius_thresh:=20 radius_min:=6 velocity_min:=4 accel_limit:=0.5 decel_limit:=0.3 velocity_offset:=4 braking_distance:=5 end_point_offset:=1 use_decision_maker:=False