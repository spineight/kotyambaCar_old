#!/bin/bash

echo "This script should be run on command_center machine"
echo "This script should be sourced\n\n"
echo "setting path for kotyambaCar repo folders on Raspberry, will be used by ROS  *.launch files\n\n"


echo "installing required packages:"
sudo apt-get update
sudo apt-get install ros-melodic-web-video-server

echo "#### Setting up environment variables ####"
export KOTYAMBA_REPO_COMMAND_CENTER="$(pwd)"
echo "setting KOTYAMBA_REPO_COMMAND_CENTER to $KOTYAMBA_REPO_COMMAND_CENTER"

read -p "Enter fullpath to kotyambaCar repo on RaspberryPI: " fullpath

export KOTYAMBA_REPO_RASPBERRY="$fullpath"
echo "KOTYAMBA_REPO_RASPBERRY was set to: $KOTYAMBA_REPO_RASPBERRY"
echo "check it! echo $KOTYAMBA_REPO_RASPBERRY"

echo "Enabling ssh on Ubuntu 18.04"

sudo apt-get update
sudo apt-get install openssh-server -y

# To verify that the installation was successful and SSH service is running 
# type the following command which will print the SSH server status:
sudo systemctl status ssh

# Ubuntu comes with a firewall configuration tool called UFW. 
# If the firewall is enabled on your system, make sure to open the SSH port:
sudo ufw allow ssh

echo "##### 3. Using system service to start scripts that setup ROS network vars"
sudo cp /home/oleg/dev/kotyambaCar/src/catkin-ws/src/kotyambaCar/scripts/setup_command_center_ros_network.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable setup_command_center_ros_network.service --now

# check service status:  
systemctl status setup_command_center_ros_network.servic