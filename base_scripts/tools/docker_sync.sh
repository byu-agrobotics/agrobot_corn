#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Pulls the the latest Docker image onto the robot

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

# Check the architecture of the robot
robot_arch=$(ssh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "uname -m")
if [[ "$robot_arch" == "aarch64" ]]; then
    robot_id="linux/arm64"
else
    robot_id="linux/amd64"
fi

# Pull the platform-specific version of the Docker image
printInfo "Pulling the $robot_id version of the Docker image onto the base station..."
docker pull --platform $robot_id byuagrobotics1/agrobot_2.0:latest
docker image list

printInfo "Saving the $robot_id Docker image to a zip file for transfer (this takes a while)..."
docker save byuagrobotics1/agrobot_2.0:latest | gzip > agrobot_2.0.tar.gz

printInfo "Sending the Docker image to the robot..."
scp agrobot_2.0.tar.gz $ROBOT_USERNAME@$ROBOT_IP_ADDRESS:~/agrobot_2.0/docker
rm agrobot_2.0.tar.gz

printInfo "Unzipping the Docker image on the robot..."
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0/docker && gunzip agrobot_2.0.tar.gz"

printInfo "Loading the Docker image on the robot..."
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0/docker && docker load < agrobot_2.0.tar"
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0/docker && rm agrobot_2.0.tar"
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "yes | docker image prune"
ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "docker image list"

printInfo "Pulling the linux/amd64 Docker image onto the base station..."
docker pull --platform linux/amd64 byuagrobotics1/agrobot_2.0:latest
yes | docker image prune
docker image list
