#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Set up the base station as an upstream git remote for the robot

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for a "-u <username>" argument
while getopts ":u:" opt; do
  case $opt in
    u)
      ROBOT_USERNAME=$OPTARG
      ;;
  esac
done

# Check for an SSH connection to the robot
checkConnection # defined in base_common.sh

# Set up the upstream git remote for the robot
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0 && \
    git remote add base $(whoami)@$(hostname -I | awk '{print $1}'):agrobot_2.0"
