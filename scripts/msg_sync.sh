#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs custom messages between the ROS2 and micro-ROS workspaces

rsync -avc --delete ~/robot_ws/src/robot_interfaces ~/firmware/teensy_pio/extra_packages

cd ~/firmware/teensy_pio
pio run --target clean_microros
pio pkg install
pio run
