#!usr/bin/env bash

source ~/trionix/devel/setup.bash
export LC_ALL=C
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.101
export ROS_IP=192.168.1.101
export VEHICLE_NAME="trionix"

exec "$@"
