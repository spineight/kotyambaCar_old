#!/bin/bash

echo "####################################################################"
echo "####   This script should be run ONCE on command_center machine ####"
echo "####   This script should be sourced\n\n                        ####"
echo "####################################################################"


echo "Preparing command_center machine for kotyambaCar:"

echo "Step 1. Install additional packages"
sudo apt-get update -y
sudo apt-get install ros-melodic-web-video-server -y

echo "Step 2. Make sure SSH is working"
sudo apt-get install openssh-server -y
# Ubuntu comes with a firewall configuration tool called UFW. 
# If the firewall is enabled on your system, make sure to open the SSH port:
https://www.digitalocean.com/community/tutorials/how-to-set-up-a-firewall-with-ufw-on-ubuntu-18-04
status="`sudo ufw status`"
if [ "$status" != "Status: inactive" ]; then
    echo "ufw is active, enabling ssh"
    sudo ufw allow ssh
else
    echo "ufw is not active"
fi
echo "Status should be ready:"
sudo systemctl status ssh

### TODO: do I need it????
echo "Step 2. Set path to the repo directory, we are referencing it in scripts"
export KOTYAMBA_REPO_COMMAND_CENTER="$(pwd)"
echo "setting KOTYAMBA_REPO_COMMAND_CENTER to $KOTYAMBA_REPO_COMMAND_CENTER"

# read -p "Enter fullpath to kotyambaCar repo on RaspberryPI: " fullpath

# export KOTYAMBA_REPO_RASPBERRY="$fullpath"
# echo "KOTYAMBA_REPO_RASPBERRY was set to: $KOTYAMBA_REPO_RASPBERRY"
# echo "check it! echo $KOTYAMBA_REPO_RASPBERRY"

echo "Step 3. Using systemd service to set up scripts to be started on boot"
echo "##### 3. Using system service to start scripts that setup ROS network vars"
sudo cp setup_command_center_nodes.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable setup_command_center_nodes.service --now

# check service status:  
systemctl status setup_command_center_nodes.service