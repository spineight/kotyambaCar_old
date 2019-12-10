#!/usr/bin/env bash
source ~/.bashrc

export ROS_IP="$(getent ahosts commandCenter.local | awk 'NR==1{ print $1 }')"
echo "ROS_IP $ROS_IP"

echo "Allow ssh to RaspberryPI"
export ROSLAUNCH_SSH_UNKNOWN=1