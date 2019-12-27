#!/bin/bash

echo "####################################################################"
echo "####   This script should be run ONCE on raspberry pi           ####"
echo "####   This script should be sourced\n\n                        ####"
echo "####   ex. source prepare_command_center.sh                     ####"
echo "####################################################################"


echo "Preparing raspberry pi machine for kotyambaCar:"
echo $KOTYAMBA_REPO_RASPBERRY
if [ "$KOTYAMBA_REPO_RASPBERRY" != "" ]; then
    echo "You already sourced prepare_raspberry_pi\n"
    echo "This should be done only once on your machine.\n"
    echo "Exiting"
    return 1
fi


echo "##__Step 1.__## Installing required packages"
sudo apt-get update -y
sudo apt install python-pip -y
python -m pip install tornado
python -m pip install RPi.GPIO

pip install pyyaml #yaml files are used for motors configs
# https://stackoverflow.com/questions/55551191/module-yaml-has-no-attribute-fullloader
pip install -U PyYAML

echo "##__Step 2__.## Adding env vars to ~/.bashrc"
echo "Setting KOTYAMBA_REPO_RASPBERRY to path to the repository"
echo "export KOTYAMBA_REPO_RASPBERRY=$(pwd)" >> ~/.bashrc
echo "Check that this is correct KOTYAMBA_REPO_RASPBERRY=$(pwd)"
source ~/.bashrc

echo "##__Step 3__.## Using systemd to setup starting of Tornado web server on boot ####"
sudo cp  /home/pi/kotyambaCar/src/control_server/start_tornado_webserver.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable start_tornado_webserver.service --now
# check service status:  
systemctl status start_tornado_webserver.service