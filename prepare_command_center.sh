#!/bin/bash

echo "This script should be run on command_center machine"
echo "This script should be sourced\n\n"
echo "setting path for kotyambaCar repo folders on Raspberry, will be used by ROS  *.launch files\n\n"

echo "#### Setting up environment variables ####"
export KOTYAMBA_REPO_COMMAND_CENTER="$(pwd)"
echo "setting KOTYAMBA_REPO_COMMAND_CENTER to $KOTYAMBA_REPO_COMMAND_CENTER"

read -p "Enter fullpath to kotyambaCar repo on RaspberryPI: " fullpath

export KOTYAMBA_REPO_RASPBERRY="$fullpath"
echo "KOTYAMBA_REPO_RASPBERRY was set to: $KOTYAMBA_REPO_RASPBERRY"
echo "check it!"
