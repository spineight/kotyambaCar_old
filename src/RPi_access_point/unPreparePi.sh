#!/bin/bash

## this script undo effects of the script: preparePy.sh and revert system to initial state
echo "restoring /etc/dhcpcd.conf.backup --> /etc/dhcpcd.conf"
sudo mv /etc/dhcpcd.conf.backup /etc/dhcpcd.conf
echo "disabeling service for starting docker container(network setup) on boot"
sudo rm  /etc/systemd/system/start_RPi_Docker_Container.service
sudo systemctl disable start_RPi_Docker_Container.service

echo "stopping docker container, named RPi_access_point_Container"
docker stop RPi_access_point_Container

echo "check that operations succeded:"
service start_RPi_Docker_Container status
docker ps

echo "Reboot for changes to take effect"
sudo reboot
