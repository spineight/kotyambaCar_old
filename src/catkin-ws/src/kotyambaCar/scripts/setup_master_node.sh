#!/usr/bin/env bash
export ROS_MASTER_IP="$(getent hosts commandCenter.local | awk '{ print $1 }')"
export ROS_MASTER_URI="http://$ROS_MASTER_IP:11311"
export ROS_IP="$ROS_MASTER_IP"