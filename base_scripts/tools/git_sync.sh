#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Pulls the base station's local git branch onto the robot

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

# Check for the git upstream 'base'
if ! ssh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0 && git remote -v" | grep -q "base"
then
    printError "No upstream git remote 'base' found on the robot"
    echo "Please run 'bash setup_git.sh' to set up the upstream git remote"

    exit 1
fi

# Get the current git branch name
export current_branch=$(git branch --show-current)

ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0 && git checkout $current_branch"
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0 && git pull base ${current_branch}"
