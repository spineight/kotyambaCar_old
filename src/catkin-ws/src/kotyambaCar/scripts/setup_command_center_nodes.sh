#!/usr/bin/env bash
source ~/.bashrc

export ROS_IP="$(getent ahosts commandCenter.local | awk 'NR==1{ print $1 }')"
echo "ROS_IP $ROS_IP"

export ROS_MASTER_IP="$(getent ahosts raspberrypi.local | awk 'NR==1{ print $1 }')"
echo "ROS_MASTER_IP: $ROS_MASTER_IP"
export ROS_MASTER_URI="http://$ROS_MASTER_IP:11311"

exec "$@"