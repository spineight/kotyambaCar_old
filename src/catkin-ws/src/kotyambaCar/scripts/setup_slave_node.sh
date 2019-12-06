#!/usr/bin/env bash

source /home/pi/ros_catkin_ws/install_isolated/setup.bash
source /home/pi/kotyambaCar/src/catkin-ws/install/setup.bash
export ROS_MASTER_IP="$(getent ahosts commandCenter.local | awk 'NR==1{ print $1 }')"
echo "ROS_MASTER_IP: $ROS_MASTER_IP"
export ROS_MASTER_URI="http://$ROS_MASTER_IP:11311"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
export ROS_IP="$(getent ahosts raspberrypi.local | awk 'NR==1{ print $1 }')"
echo "ROS_IP $ROS_IP"

export KOTYAMBA_PATH=~/dev/kotyambaCar/

exec "$@"
