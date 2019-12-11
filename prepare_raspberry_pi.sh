#!/bin/bash

echo "Important! This script should be sourced: source ./prepare.sh"
echo "Otherwise no environment variables will be set"
echo "Preparing kotyambaCar"

# TODO consider using virtualenv, but how to reboot for changes to take effect
echo "#### 1. Install all required packages ####"

#echo "preparing environment"
sudo apt-get update -y
sudo apt install python-pip -y
python -m pip install tornado
python -m pip install RPi.GPIO
sudo apt-get install motion -y


pip install pyyaml #yaml files are used for motors configs
# https://stackoverflow.com/questions/55551191/module-yaml-has-no-attribute-fullloader
pip install -U PyYAML

#pip --version
#pip install virtualenv
#echo 'PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
#. ~/.bashrc
#virtualenv env
#. /env/bin/activate

echo "#### 2. Using systemd to setup starting of Tornado web server on boot ####"
sudo cp src/control_server/start_tornado_webserver.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable start_tornado_webserver.service --now

# check service status:  
systemctl status start_tornado_webserver.service

echo "##### 3. Using system service to start scripts that setup ROS network vars"
sudo cp /home/pi/kotyambaCar/src/catkin-ws/src/kotyambaCar/scripts/setup_raspberry_pi_ros_network.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable setup_raspberry_pi_ros_network.service --now

# check service status:  
systemctl status setup_raspberry_pi_ros_network.service

echo "#### 3. Setting up environment variables ####"
export KOTYAMBA_REPO_RASPBERRY="$(pwd)"
echo "setting KOTYAMBA_REPO_RASPBERRY to $KOTYAMBA_REPO_RASPBERRY"
echo "export KOTYAMBA_REPO_RASPBERRY='$(pwd)' " >> ~/.bashrc

read -p "Enter fullpath to kotyambaCar repo on command_center machine: " fullpath
export KOTYAMBA_REPO_COMMAND_CENTER="$fullpath"
echo "KOTYAMBA_REPO_COMMAND_CENTER was set to: $KOTYAMBA_REPO_COMMAND_CENTER"
echo "export KOTYAMBA_REPO_COMMAND_CENTER='$fullpath' " >> ~/.bashrc
echo "check it!: echo '$'KOTYAMBA_REPO_COMMAND_CENTER"

echo "#### 4. allow ssh to command center ####"
export ROSLAUNCH_SSH_UNKNOWN=1
echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> ~/.bashrc