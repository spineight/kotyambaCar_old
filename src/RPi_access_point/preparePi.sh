#!/bin/bash

## this script should be run manually once, to prepare raspberry pi for docker container execution  
## it setups docker image run on system startup
echo "backup /etc/dhcpcd.conf"
sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.backup
echo "edit /etc/dhcpcd.conf to disable dhcp for ap0 on the host"
sudo echo "deny interfaces ap0" >> /etc/dhcpcd.conf 
echo "enabling service for starting docker container(network setup) on boot"
sudo cp services/start_RPi_Docker_Container.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable start_RPi_Docker_Container.service --now
echo "check that operations succeded:"
service start_RPi_Docker_Container status
sleep 5
docker ps
