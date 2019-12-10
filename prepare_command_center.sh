#!/bin/bash

echo "This script should be run on command_center machine"
echo "This script should be sourced"
echo "setting path for kotyambaCar repo folders on Raspberry, will be used by ROS  *.launch files"

read -p "Enter fullpath to kotyambaCar repo " fullpath

export KOTYAMBA_REPO_FOLDER_RASPBERRY="$fullpath"
