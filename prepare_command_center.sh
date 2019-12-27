#!/bin/bash

echo "####################################################################"
echo "####   This script should be run ONCE on command_center machine ####"
echo "####   This script should be sourced\n\n                        ####"
echo "####   ex. source prepare_command_center.sh                     ####"
echo "####################################################################"


echo "Preparing command_center machine for kotyambaCar:"
echo $KOTYAMBA_REPO_COMMAND_CENTER
if [ "$KOTYAMBA_REPO_COMMAND_CENTER" != "" ]; then
    echo "You already sourced prepare_command_center\n"
    echo "This should be done only once on your machine.\n"
    echo "Exiting"
    return 1
fi

echo "##__Step 1.__## Installing required packages"
sudo apt-get update -y
sudo apt-get install ros-melodic-web-video-server -y

echo "##__Step 2.__## Makeing sure SSH is working"
sudo apt-get install openssh-server -y
# Ubuntu comes with a firewall configuration tool called UFW. 
# If the firewall is enabled on your system, make sure to open the SSH port:
https://www.digitalocean.com/community/tutorials/how-to-set-up-a-firewall-with-ufw-on-ubuntu-18-04
status="`sudo ufw status`"
if [ "$status" != "Status: inactive" ]; then
    echo "ufw firewall is active, enabling ssh"
    sudo ufw allow ssh
else
    echo "ufw is not active"
fi
echo "Checking SSH status. \nStatus should be ready:"
sudo systemctl status ssh


echo "##__Step 3__.## Adding env vars to ~/.bashrc"
echo "Setting KOTYAMBA_REPO_COMMAND_CENTER to path to the repository"
echo "export KOTYAMBA_REPO_COMMAND_CENTER=$(pwd)" >> ~/.bashrc
echo "Check that this is correct KOTYAMBA_REPO_COMMAND_CENTER=$(pwd)"
source ~/.bashrc
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