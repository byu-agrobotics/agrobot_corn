#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Common functions and variables for base station scripts

ROBOT_IP_ADDRESS=192.168.0.1
ROBOT_USERNAME=agrobot
ROBOT_PASSWORD=agrobot

function printInfo {
  # print blue
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  # print yellow
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  # print red
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

function checkConnection {
  # Check for an SSH connection to the robot
  if ! ssh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "echo" &> /dev/null
  then
      printError "No available SSH connection to the robot"
      echo "Here's some debugging suggestions:"
      echo "  - Ensure the robot is powered on"
      echo "  - Run 'bash setup_ssh.sh' to set up SSH keys"

      exit 1
  fi
}
