#!/bin/bash
echo "Running RPi docker container!" && docker run --name RPi_access_point_Container --rm --privileged --net host spineight/rpi_access_point:1.0
