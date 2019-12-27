#!/usr/bin/env bash

source ~/.bashrc

echo "SETTING UP Master Node IP"
export ROS_MASTER_IP="$(getent ahosts raspberrypi.local | awk 'NR==1{ print $1 }')"
if [ "$ROS_MASTER_IP" == "" ]; then
    echo "Can't find out raspberry pi IP address\n"
    echo "Raspberry PI should be started before command center and they should be within the same network.\n"
    echo "Exiting"
    return 1
fi
echo "ROS_MASTER_IP: $ROS_MASTER_IP"
export ROS_MASTER_URI="http://$ROS_MASTER_IP:11311"
echo "ROS_MASTER_URI $ROS_MASTER_URI"
export ROS_IP="$(getent ahosts commandCenter.local | awk 'NR==1{ print $1 }')"
echo "ROS_IP $ROS_IP"

exec "$@"