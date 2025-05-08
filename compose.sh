#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Simple script to set up a quick robot development environment

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

script_dir=$(dirname "$(readlink -f "$0")")

case $1 in
  	"down")
    	printWarning "Stopping the agrobot-ct container..."
    	docker compose -f docker/docker-compose.yaml down
    	;;
  	*)
    	printInfo "Loading the agrobot-ct container..."
    	docker compose -f docker/docker-compose.yaml up -d

		# Check if a 'robot_dev' tmux session already exists
		if [ "$(docker exec -it agrobot-ct tmux list-sessions | grep robot_dev)" == "" ]; then

			# If not, create a new 'robot_dev' tmux session
			printWarning "Creating a new 'robot_dev' tmux session..."
			envsubst < $script_dir/base_scripts/tmuxp/robot_dev.yaml > $script_dir/base_scripts/tmuxp/tmp/robot_dev.yaml
			docker exec -it agrobot-ct tmuxp load -d /home/agrobot-docker/.tmuxp/robot_dev.yaml
		fi
		# Attach to the 'robot_dev' tmux session
		docker exec -it agrobot-ct tmux attach -t robot_dev
    ;;
esac
