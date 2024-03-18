#!usr/bin/env bash

source ~/trionix/devel/setup.bash
export LC_ALL=C
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.104
export ROS_IP=192.168.1.104
export VEHICLE_NAME="trionix"

# remove args that melodic doesn't support
for arg do
  shift
  [ "$arg" = "--sigint-timeout" ] && continue
  [ "$arg" = "--sigterm-timeout" ] && continue
  [[ "$arg" == [0-9]*.[0-9] ]] && continue
  set -- "$@" "$arg"
done

exec "$@"

