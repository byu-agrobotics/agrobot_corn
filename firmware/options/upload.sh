#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads stored hex files to the Teensy board

# Usage check
if [ -z "$1" ]; then
  echo "Usage: $0 <hex_file>"
  exit 1
fi

cd ~/firmware/options
tycmd upload $1