#!/bin/bash

# Sample bash script to change the vehicle speed in the highway scenario
# Autoware.AI is required to be installed and sourced

# $1: velocity
# $2: lookahead_distance
echo $1 $2

# Publish new velocity and lookahead distance
timeout 2s rostopic pub /config/waypoint_follower autoware_config_msgs/ConfigWaypointFollower "{\"param_flag\": 1, \"velocity\": $1, \"lookahead_distance\": $2, \"lookahead_ratio\": 2.0}"

